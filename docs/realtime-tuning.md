# Real-time deployment tuning

`voice-coil` runs a 1 ms DC/SYNC0-synchronized EtherCAT cyclic loop
(`fieldbus_run_cyclic()` in [`control_loop.c`](../control_loop.c)). The application does
what it can on its own — `mlockall()`, prefaulted buffers, `SCHED_FIFO` + CPU affinity +
minimal timer slack around the loop (see `main()` in [`main.c`](../main.c)) —
but real-time behavior on Linux also depends on host and kernel settings this program
cannot set for itself. This doc covers those, with concrete values for the current target
machine.

## Target machine

| | |
|---|---|
| CPU | Intel Core i5-3550, 4 cores (0–3), no hyperthreading |
| Kernel | `6.8.1-realtime` (PREEMPT_RT) — already the right kernel, see [Kernel choice](#kernel-choice) |
| EtherCAT NIC | `enp1s2` (Intel `e1000`, PCI `0000:01:02.0`, IRQ 18) |
| Launched as | `sudo ./voice-coil enp1s2` |

CPU core assignment:

| core | role |
|---|---|
| 0 | OS housekeeping, NIC IRQ |
| **1** | **reserved for the RT cyclic loop** — must match `RT_CPU_CORE` in [`main.h`](../main.h) |
| 2, 3 | OS housekeeping, everything else |

The application pins the cyclic loop to `RT_CPU_CORE` via `sched_setaffinity()`. That only
helps if the kernel also keeps everything else off that core — the rest of this doc is
about making core 1 genuinely idle except for the loop.

## Kernel command line

Isolate core 1, stop its periodic tick, offload its RCU work, keep IRQs off it, and cap
idle latency. Edit `/etc/default/grub` (needs `sudo`) and set:

```
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash isolcpus=managed_irq,domain,1 nohz_full=1 rcu_nocbs=1 irqaffinity=0,2,3 intel_idle.max_cstate=1 processor.max_cstate=1 nmi_watchdog=0 skew_tick=1 nosoftlockup tsc=nowatchdog"
```

| token | effect |
|---|---|
| `isolcpus=managed_irq,domain,1` | keep the scheduler and driver-managed IRQs off core 1. Modern syntax — no `isolcpus=1` deprecation warning. The app's explicit `sched_setaffinity()` still runs the loop on core 1. |
| `nohz_full=1` | stop the periodic scheduling-clock tick on core 1 (cores 0/2/3 stay as housekeeping) |
| `rcu_nocbs=1` | run core 1's RCU callbacks on another core |
| `irqaffinity=0,2,3` | default IRQ affinity excludes core 1 |
| `intel_idle.max_cstate=1` + `processor.max_cstate=1` | cap idle at C1. C3/C6 exit latency on this Ivy Bridge part is tens of µs — the single biggest jitter source here. |
| `nmi_watchdog=0` | drop a periodic per-CPU NMI |
| `skew_tick=1` | de-synchronise remaining ticks across cores, avoids lock-contention spikes |
| `nosoftlockup` | disable the soft-lockup detector's per-CPU work |
| `tsc=nowatchdog` | stop the periodic TSC-vs-clocksource watchdog (reads HPET on all cores) |

Deliberately **not** included: `idle=poll` (pins every core at 100%, needless heat) and
`intel_pstate=disable` (the governor below handles frequency).

Apply and reboot:

```bash
sudo nano /etc/default/grub      # edit the line above
sudo update-grub
sudo reboot
```

Verify after reboot:

```bash
cat /proc/cmdline
cat /sys/devices/system/cpu/isolated       # -> 1
cat /sys/devices/system/cpu/nohz_full      # -> 1
```

## CPU frequency scaling and idle states

Idle states are capped by the kernel command line above. Frequency scaling still needs the
`performance` governor pinned so the core doesn't ramp down between cycles (`cpupower
frequency-set` does not survive a reboot). Install it as a service:

```bash
sudo systemctl edit --force --full rt-perf.service
```

```ini
[Unit]
Description=Pin CPU governor to performance for RT
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/usr/bin/cpupower frequency-set -g performance

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable --now rt-perf.service
cat /sys/devices/system/cpu/cpu1/cpufreq/scaling_governor   # -> performance
```

Optionally disable turbo as well, so the core frequency is genuinely constant (trades a
little headroom for lower jitter):

```bash
echo 1 | sudo tee /sys/devices/system/cpu/intel_pstate/no_turbo
```

Deep C-states can also be disabled in BIOS/UEFI for a belt-and-braces guarantee.

## NIC interrupt affinity

`enp1s2` uses IRQ **18**, a legacy IO-APIC line **shared with the `i801_smbus`
controller** — the two cannot be separated. It currently lands on core 0, which is already
off the RT core, so the requirement (keep NIC interrupts away from core 1) is met by
`irqaffinity=0,2,3` above. To pin it explicitly:

```bash
grep -E 'enp1s2|i801' /proc/interrupts        # confirm the IRQ number (18)
echo 0 | sudo tee /proc/irq/18/smp_affinity_list
```

`irqbalance` is inactive on this machine, so nothing will move it back. If it gets enabled,
ban core 1 in `/etc/default/irqbalance`:

```
IRQBALANCE_BANNED_CPUS=00000002
```

(bit 1 set = core 1). Note SOEM here does its packet TX/RX inline on the loop thread —
there is no separate SOEM RX thread to prioritise.

## Memory locking

Already satisfied on this machine: the program runs as root (`sudo`), so `mlockall()`
succeeds, and `/etc/security/limits.d/realtime.conf` grants the `realtime` group
`memlock unlimited`.

To run it **without** `sudo`, grant just the locking capability and make sure the user's
`memlock` limit is high enough:

```bash
sudo setcap cap_ipc_lock,cap_net_raw+ep ./voice-coil   # net_raw also needed for the NIC
ulimit -l                                               # must be large / unlimited
```

`cap_net_raw` is required because SOEM opens a raw socket on the NIC.

## Kernel choice

Already done — the target runs a `PREEMPT_RT` kernel (`uname -r` →
`6.8.1-realtime`). Vanilla Linux with the tuning above still shows occasional
multi-millisecond scheduling-latency spikes under load; `PREEMPT_RT` is what gets
worst-case jitter into the sub-100 µs range this loop needs.

## Measuring

After the tuning is in place, the loop's own instrumentation is the check: every exported
`data/voice_coil_log_*.csv` has a `cycle_jitter_us` column (signed offset of actual vs.
scheduled cycle time), and the run prints a `missed deadlines: N (max jitter X us)`
summary line. A well-tuned run of this machine should show max jitter well under the 1 ms
cycle budget with zero missed deadlines.

For a system-level baseline independent of the application, run `cyclictest` (from
`rt-tests`) on the isolated core while the EtherCAT link is active:

```bash
sudo cyclictest -m -p 99 -i 1000 -a 1 -t 1 -D 60
```

## Development environment caveat

The primary dev environment for this project is WSL2 (Hyper-V-virtualized Linux). WSL2
cannot be used to validate any of the above: it doesn't expose raw EtherCAT NIC access in
the way SOEM needs, and Hyper-V's own scheduling adds latency and jitter outside Linux's
control. Real-time behavior — and the `cycle_jitter_us` values in the exported CSV — must
be measured on the actual bare-metal target machine, not in WSL2.
