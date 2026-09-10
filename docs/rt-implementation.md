# Real-time Implementation

This document describes the real-time (RT) improvements made to `voice-coil` — both the application-level code changes and the system/kernel configuration required to achieve predictable sub-millisecond cycle latency on the EtherCAT fieldbus control loop.

## Application code changes

### Deadline-critical cyclic loop isolation

The real-time scheduling and CPU affinity are applied in a narrow window around the cyclic loop only, not the entire program lifetime. This keeps the system from starving non-RT work during unbounded SDO configuration and CiA402 bring-up phases.

**File: `main.c` `main()`**

- `mlockall(MCL_CURRENT | MCL_FUTURE)` is called early to lock all process memory, preventing page faults during the loop.
- Buffers (`samples`, `faults`) are pre-allocated and prefaulted via `memset()` before entering RT mode, so the first write to each page doesn't trigger a fault.
- Immediately before `fieldbus_run_cyclic()`:
  - `sched_setaffinity()` pins the loop to `RT_CPU_CORE` (core 1 by default).
  - `sched_setscheduler()` raises scheduling to `SCHED_FIFO` at maximum priority.
  - `prctl(PR_SET_TIMERSLACK, 1)` minimizes timer coalescing.
- Immediately after the loop returns, normal scheduling and affinity are restored so file I/O and CSV export don't block the whole system.

### Blocking I/O removed from the critical path

`printf()` and `fprintf()` calls block on I/O syscalls and add unbounded latency inside the loop. All such calls have been moved out:

**File: `control_loop.c` `fieldbus_run_cyclic()`**

- Error detection (WKC errors, state drift) now only records flags and logs to the in-memory fault buffer.
- Error messages are deferred and printed by `export_csv()` after the loop completes.
- Per-cycle timing jitter is measured and recorded in every sample, not printed live.

**File: `logging.c` `log_fault()` and `export_csv()`**

- `log_fault()` only writes to the in-memory buffer; no console output.
- `export_csv()` iterates the fault buffer and prints all events to console and CSV after the loop is done, preserving the same user-facing output at a non-time-critical point.

### Jitter instrumentation

Every sample now captures the signed offset between actual and scheduled cycle completion time:

**File: `main.h`**

- `sample_log_entry_t` gains `cycle_jitter_us` (offset in microseconds; negative = early / ahead of deadline, positive = late) and `pdo_exchange_us` (see next section).

**File: `control_loop.c` `fieldbus_run_cyclic()`**

- `cycle_jitter_us` is computed after every PDO exchange and before the next sleep, via the
  `timespec_diff_us()` helper: `timespec_diff_us(&now, &next_cycle)`.
- Cycles past the deadline (jitter > 0) increment a missed-deadline counter.
- Max jitter is tracked and reported in a summary line after the loop.

**File: `logging.c` `export_csv()`**

- CSV output includes `cycle_jitter_us` (column 9) and `pdo_exchange_us` (column 10).
- Summary line prints: `missed deadlines: N (max jitter X us, max PDO exchange Y us)`.

### Split timing: PDO exchange vs. loop thread

`cycle_jitter_us` alone can't say *where* a slow cycle spent its time. `pdo_exchange_us`
measures only `ecx_send_processdata()` + `ecx_receive_processdata()` — the frame
round-trip (NIC driver, wire, slave response, housekeeping-core IRQ servicing). Comparing
the two localizes a spike:

| `pdo_exchange_us` spike | `cycle_jitter_us` spike | Interpretation |
|---|---|---|
| yes | yes (matching) | Bus / slave / NIC-driver / housekeeping-core IRQ latency — points *outward* |
| no | yes | The loop thread itself was stalled — IPI, SMI, preemption, cache/memory contention |

Compute time per cycle = `(CYCLE_TIME_MS * 1000 + cycle_jitter_us) - pdo_exchange_us`.

**File: `control_loop.c` `fieldbus_run_cyclic()`**

```c
clock_gettime(CLOCK_MONOTONIC, &pdo_start);
ecx_send_processdata(context);
wkc = ecx_receive_processdata(context, EC_TIMEOUTRET);
clock_gettime(CLOCK_MONOTONIC, &pdo_end);
pdo_exchange_us = timespec_diff_us(&pdo_end, &pdo_start);
```

### Cycle 0 deadline fix

The first cycle's jitter measurement was biased: `next_cycle` was initialized to loop-start time, so cycle 0 measured the PDO round-trip duration against the start instant, not a real deadline. Fixed by advancing `next_cycle` by one cycle before the loop starts.

**File: `control_loop.c` line ~67**

```c
clock_gettime(CLOCK_MONOTONIC, &next_cycle);
add_timespec(&next_cycle, cycle_ns / 1000);  /* Advance to first real deadline */
```

Now cycle 0 is measured against `start + 1 ms` like all other cycles, and the spurious "missed deadline" flag disappears.

## System configuration

### Target machine

- **CPU:** Intel Core i5-3550 (4 cores, no hyperthreading)
- **Kernel:** 6.8.1-realtime (PREEMPT_RT)
- **EtherCAT NIC:** Intel 82579LM (`e1000e`) on `eno1` — MSI IRQ 26, PCH-integrated (see *EtherCAT NIC selection* below)
- **Core allocation:**
  - Core 1: reserved for the RT loop
  - Cores 0, 2, 3: OS housekeeping and interrupts

### Kernel command line

Added to `/etc/default/grub` `GRUB_CMDLINE_LINUX_DEFAULT`:

```
isolcpus=managed_irq,domain,1 nohz_full=1 rcu_nocbs=1 irqaffinity=0,2,3 \
intel_idle.max_cstate=1 processor.max_cstate=1 nmi_watchdog=0 skew_tick=1 \
nosoftlockup tsc=nowatchdog
```

| Parameter | Effect |
|---|---|
| `isolcpus=managed_irq,domain,1` | Keep scheduler and driver IRQs off core 1 |
| `nohz_full=1` | Stop periodic tick on core 1 |
| `rcu_nocbs=1` | Offload RCU callbacks from core 1 |
| `irqaffinity=0,2,3` | Default IRQ affinity excludes core 1 |
| `intel_idle.max_cstate=1` | Cap idle at C1 (not C3/C6) |
| `processor.max_cstate=1` | Limit ACPI C-states as well |
| `nmi_watchdog=0` | Disable NMI watchdog |
| `skew_tick=1` | De-synchronize ticks across cores |
| `nosoftlockup` | Disable soft-lockup detector |
| `tsc=nowatchdog` | Disable TSC watchdog |

### CPU frequency governor

Pinned to `performance` via a persistent systemd service:

**File: `/etc/systemd/system/rt-perf.service`**

```ini
[Unit]
Description=Pin CPU governor to performance and disable turbo for RT
After=multi-user.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/bin/cpupower frequency-set -g performance
ExecStart=/bin/sh -c 'echo 1 > /sys/devices/system/cpu/intel_pstate/no_turbo'

[Install]
WantedBy=multi-user.target
```

Turbo is explicitly disabled (`no_turbo=1`) for clock frequency consistency.

### EtherCAT NIC selection

This machine has two Intel NICs. The fieldbus runs on the one with the better
host-side latency path:

| | `eno1` — 82579LM | `enp1s2` — 82544GC |
|---|---|---|
| Driver | `e1000e` | `e1000` |
| Location | PCH LAN-on-motherboard, DMI-integrated | add-in card on legacy 33/66 MHz PCI |
| Interrupt | **dedicated MSI vector** (IRQ 26) | **legacy IO-APIC line 18, shared with `i801_smbus`** |
| Coalescing control | yes (`rx-usecs`) | none exposed |

`eno1` wins on the two things that matter for PDO jitter: a private MSI vector
(no shared-handler demux on every SMBus transaction) and an integrated DMA path
(no legacy-PCI bus arbitration). `enp1s2` now carries internet instead — its
shared IRQ is harmless there.

### NIC interrupt affinity

The EtherCAT NIC's MSI IRQ (26 for `eno1`) is kept on the housekeeping cores by
`irqaffinity=0,2,3` above, off the RT core 1. Verify after any cabling change:

```bash
grep eno1 /proc/interrupts                 # confirm the IRQ number + MSI
cat /proc/irq/<N>/smp_affinity_list        # must not include core 1
```

### NIC RT tuning

`scripts/setup-ethercat-nic.sh` dedicates the interface to the fieldbus and pins
its link/offload settings: unmanages it from NetworkManager (no DHCP retries),
disables IPv6, EEE/LPI, pause frames and interrupt coalescing, turns off the
segmentation/receive offloads (batching adds jitter; TSO on the 82579 is also
tied to "Detected Hardware Unit Hang"), and fixes the link at 100/full so it
can't spend 2–4 s renegotiating mid-run. `--install-service` persists it across
reboots via `ethercat-nic@.service`.

```bash
sudo ./scripts/setup-ethercat-nic.sh --iface eno1 --install-service
```

## Measured performance

Full 30-second / 30,000-cycle run after kernel reboot with all tuning active:

| Metric | Value |
|---|---|
| Cycles completed | 30,000 |
| Jitter std dev | 9.1 µs |
| Mean lateness before deadline | −949.6 µs |
| Worst-case lateness | −767.2 µs |
| Real missed deadlines | 0 / 30,000 |
| Total faults | 0 |

Every cycle completes 767–963 µs before its 1 ms deadline. The distribution is tight: ~4,600 of 5,000 cycles in one 10 µs bucket; tail bounded at ~200 µs excursion from mean.

Pre-tuning (vanilla kernel, no RT isolation), the same workload showed:
- Jitter std dev: 20.3 µs (2.2× worse)
- Worst-case excursion: ~420 µs (2.3× worse)
- Zero missed deadlines, but larger variance

## Validation and deployment

### On this machine

```bash
make clean && make
sudo ./voice-coil eno1
tail -1 data/voice_coil_log_*.csv
# Check: missed deadlines: 0; cycle_jitter_us all negative with margin;
#        pdo_exchange_us small and stable (a spike here = bus/slave, not the loop)
```

### On another target machine

1. Edit [`main.h`](../main.h) to set `RT_CPU_CORE` to the desired isolated core.
2. Follow the kernel command-line tuning in [realtime-tuning.md](realtime-tuning.md) for that machine's CPU topology.
3. Pick the EtherCAT NIC by interrupt path (MSI + integrated bus over shared legacy IRQ), then run
   `sudo ./scripts/setup-ethercat-nic.sh --iface <IFACE> --install-service`.
4. Build and run with the tuned kernel.
5. Validate using the `cycle_jitter_us` CSV column and the summary line.

For a system-level baseline independent of the application:

```bash
sudo apt install rt-tests
sudo cyclictest -m -p 99 -i 1000 -a <RT_CPU_CORE> -t 1 -D 60
```

## References

- **Jitter field:** [`sample_log_entry_t`](../main.h) in `main.h`
- **Loop implementation:** [`fieldbus_run_cyclic()`](../control_loop.c) in `control_loop.c`
- **RT mode setup:** [`main()`](../main.c) in `main.c`
- **System tuning guide:** [realtime-tuning.md](realtime-tuning.md)
- **Kernel boot params:** `/etc/default/grub` `GRUB_CMDLINE_LINUX_DEFAULT`
- **Governor persistence:** `/etc/systemd/system/rt-perf.service`
- **NIC dedication + tuning:** [`scripts/setup-ethercat-nic.sh`](../scripts/setup-ethercat-nic.sh), persisted via `ethercat-nic@.service`
