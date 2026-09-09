# Real-time deployment tuning

`voice-coil` runs a 1 ms DC/SYNC0-synchronized EtherCAT cyclic loop
(`fieldbus_run_cyclic()` in [`control_loop.c`](../control_loop.c)). The application does
what it can on its own — `mlockall()`, prefaulted buffers, `SCHED_FIFO` + CPU affinity +
minimal timer slack around the loop (see `main()` in [`voice-coil.c`](../voice-coil.c)) —
but real-time behavior on Linux also depends on host and kernel settings this program
cannot set for itself. This doc covers those.

## CPU isolation

The application pins the cyclic loop to `RT_CPU_CORE` (defined in
[`voice_coil.h`](../voice_coil.h), default core `1`) via `sched_setaffinity()`. That only
helps if the kernel also keeps everything else off that core. Add to the kernel command
line (e.g. via GRUB, `/etc/default/grub` → `GRUB_CMDLINE_LINUX`):

```
isolcpus=1 nohz_full=1 rcu_nocbs=1
```

Adjust the core number to match `RT_CPU_CORE`. `isolcpus` keeps the general scheduler off
the core, `nohz_full` stops the periodic scheduling-clock tick from interrupting it, and
`rcu_nocbs` offloads RCU callback processing to another core.

## NIC interrupt affinity

Pin the EtherCAT NIC's interrupt to a **different** core than `RT_CPU_CORE`, so incoming
network interrupts don't preempt the RT loop's core:

```
cat /proc/interrupts | grep <nic-name>   # find the IRQ number
echo <mask-excluding-RT_CPU_CORE> > /proc/irq/<n>/smp_affinity
```

Or exclude `RT_CPU_CORE` from `irqbalance`'s ban list (`/etc/default/irqbalance`,
`IRQBALANCE_BANNED_CPUS`) if it's running.

## CPU frequency scaling and idle states

Frequency scaling and deep C-states add wake-up latency measured in tens to hundreds of
microseconds — significant against a 1 ms cycle budget.

```
cpupower frequency-set -g performance
```

Disable deep C-states for the RT core either in BIOS/UEFI, or per-core via
`/sys/devices/system/cpu/cpu<RT_CPU_CORE>/cpuidle/state*/disable`.

## Memory locking

`mlockall()` requires `CAP_IPC_LOCK` or running as root, and a sufficient
`RLIMIT_MEMLOCK`. Today a failure just prints a warning and the program continues
degraded (pages can be swapped, causing latency spikes). To make it succeed:

- Run as root, or grant the capability: `setcap cap_ipc_lock+ep ./voice-coil`.
- Raise the memlock limit in `/etc/security/limits.conf`:
  ```
  <user>  hard  memlock  unlimited
  <user>  soft  memlock  unlimited
  ```

## Kernel choice

Vanilla Linux (even with the tuning above) can still exhibit occasional multi-millisecond
scheduling latency spikes under load. If the application needs sub-100 µs worst-case
jitter guarantees, deploy on a `PREEMPT_RT`-patched kernel.

## Development environment caveat

The primary dev environment for this project is WSL2 (Hyper-V-virtualized Linux). WSL2
cannot be used to validate any of the above: it doesn't expose raw EtherCAT NIC access in
the way SOEM needs, and Hyper-V's own scheduling adds latency and jitter outside Linux's
control. Real-time behavior — and the `cycle_jitter_us` values in the exported CSV — must
be measured on the actual bare-metal target machine, not in WSL2.
