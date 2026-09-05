# Plan: CST current-mode cyclic control + logging for `voice-coil.c`

**Decisions from your answers:** compile-time `#define` constants, physical Amps, DC/SYNC0 hardware-synced loop, minimal custom PDO mapping, timestamped CSV in `./data/`.

## Test Setup

### Physical setup
- **Master:** Linux PC with SOEM library, connected to the slave via 100BASE-TX EtherCAT NIC.
- **Slave:** AMC DigiFlex Performance servo drive (vendor ID 0xBD), **no feedback device** attached (encoder/resolver). The drive will run in current-loop-only mode.
- **Actuator:** Linear voice coil motor connected to the servo drive's power output (3-phase or single-phase depending on drive model).
- **Inputs:** Two analog sensors (e.g., position/load cells, or arbitrary analog sources) connected to the drive's Analog Input 1 and 2 (0–10V range, digitized to 16-bit unsigned raw values).

### Logical setup
- **EtherCAT communication cycle:** 1 ms (configurable via `CYCLE_TIME_MS`).
- **Drive mode:** Cyclic Synchronous Torque/Current mode (CST, object 6060h = 0x0A). The master sends a target current command every cycle; the drive interpolates linearly between commands and closes an internal current loop.
- **PDO mappings (minimal custom):**
  - **RxPDO** (master → drive): ControlWord (6040h) + Target Current (6071h).
  - **TxPDO** (drive → master): StatusWord (6041h) + Actual Current (6077h) + Analog Input 1 raw (2022.01h) + Analog Input 2 raw (2022.02h).
- **Current range:** Determined by drive's peak current rating (`KP`, read from 20D8.0Ch at runtime). Default design assumes typical servo drive with KP ~ 20 A. The master command is a 10 Hz sine wave with 1 A amplitude (configurable).
- **Distributed Clocks (DC):** Synchronized via SYNC0 at the configured cycle time for precise timing across the network.
- **State machine:** After EtherCAT reaches Operational, the master brings the drive through the CiA402 state machine (Shutdown → Switch On → Enable Operation) over PDO ControlWord/StatusWord before starting the cyclic command loop.

## Goal

Extend `voice-coil.c` from a minimal EtherCAT network initialization example into a full CST (Cyclic Synchronous Torque) current-control application that:
1. Auto-configures an AMC servo drive to CST mode with user-defined cycle time and PDO mappings via an SDO hook during PreOp→SafeOp.
2. Brings the drive's CiA402 state machine through to Operation Enabled via PDO exchanges.
3. Runs a 30-second hardware-synchronized (DC/SYNC0) real-time loop, each cycle:
   - Generates a configurable-frequency/amplitude sine-wave target current command.
   - Writes target current via RxPDO.
   - Reads actual current and two analog inputs via TxPDO.
   - Logs each sample with absolute timestamp.
4. Monitors for faults and anomalies (WKC errors, drive ALstatuscode, CiA402 state changes, PDO watchdog) and logs diagnostics with timestamps; attempts recovery or graceful shutdown on unrecoverable faults.
5. Exports timestamped samples and fault log to CSV files for post-analysis (e.g., frequency response, linearity, jitter, fault timing).
6. Gracefully shuts down the drive and closes the EtherCAT network.

## Steps

**Phase A — Build wiring**
1. `CMakeLists.txt` currently builds `SOEM-test.c`. Replace by `voice-coil.c` (executable target `voice-coil` mirrors the existing `SOEM-test` target).

**Phase B — Slave configuration hook** (*parallel with A*)
2. Add compile-time constants: `CYCLE_TIME_MS` (1.0), `SINE_FREQ_HZ` (10.0), `SINE_AMPLITUDE_A` (1.0), `RUN_DURATION_S` (30.0), `CSV_DIR` ("data").
3. Implement a `PO2SOconfig` hook `amc_slave_config()` that, via SDO during the PreOp→SafeOp transition: sets 6060h=0x0A (Cyclic Synchronous Torque/Current Mode), writes 60C2.01h/.02h (interpolation period mantissa/exponent derived from `CYCLE_TIME_MS`), rewrites 1600h (RxPDO: ControlWord 6040h + Target Current 6071h) and 1A00h (TxPDO: StatusWord 6041h + Actual Current 6077h + Analog Input 1/2 raw 2022.01h/.02h), and reads 20D8.0Ch (Maximum Peak Current, `KP`) for later current scaling.
4. In `fieldbus_start()`, before `ecx_config_map_group()`: set `context->userdata` to the `Fieldbus*`, and assign the hook to any slave with `eep_man == 0xBD` (AMC's vendor ID) rather than hardcoding a slave index.

**Phase C — Process data overlay + DC/SYNC0** (*depends on B*)
5. Define packed `rx_pdo_t` (ControlWord, Target Current) / `tx_pdo_t` (StatusWord, Actual Current, AI1 raw, AI2 raw) structs overlaying the slave's outputs/inputs.
6. Call `ecx_dcsync0()` on the AMC slave with the configured cycle time (ns).
7. Add the standard SOEM `ec_sync()` drift-compensation helper and `add_timespec()` for absolute-time `clock_nanosleep` scheduling.

**Phase D — CANopen drive state machine bring-up** (*depends on C*)
8. After EtherCAT reaches Operational, drive the CiA402 state machine over PDO: Shutdown (0x06) → Switch On (0x07) → Enable Operation (0x0F), each step polling StatusWord (masked `0x6F`) for the matching state, with WKC-based error recovery reusing `fieldbus_check_state()`.
9. Optional: SDO-read 6061h to confirm mode 0x0A took effect.

**Phase E — Real-time cyclic loop** (*depends on D*)
10. Preallocate a sample buffer sized for `RUN_DURATION_S/CYCLE_TIME_MS` and a fault log buffer.
11. For 30s, DC-synced per cycle: compute sine target current (A) → convert to raw Int16 via `KP` (scale 2^15/KP) → write PDO out; exchange process data; read AI1/AI2 raw + Actual Current (convert via 2^13/KP) → log timestamped sample.

**Phase F — Automated fault diagnostics** (*integrated into Phase E*)
12. During the cyclic loop, detect and log faults:
    - **WKC mismatch:** Compare actual WKC to expected; log event with timestamp and consecutive error count. Attempt recovery via `fieldbus_check_state()` (existing pattern). Halt loop and initiate shutdown if WKC errors exceed a threshold (e.g., 5 consecutive).
    - **ALstatuscode change:** After each `ecx_readstate()` in error recovery, if ALstatuscode is non-zero or differs from last known value, log the code with timestamp and slave state. Decode via `ec_ALstatuscode2string()`.
    - **CiA402 state drift:** Poll StatusWord (masked 0x6F) against expected operation state; if unexpected state detected (e.g., dropped to SAFE_OP or FAULT), log event, record timestamp, and trigger safe shutdown (set ControlWord to Disable Voltage).
    - **Drive-reported faults:** SDO-read 2002h (Drive Status, sub-indices .01h–.07h for Bridge, Protection, System status) periodically (e.g., every 100 cycles) during operation; compare to baseline; log any new flags with timestamp.
    - **Fault log entry format:** `fault_timestamp_s`, `fault_type` (enum: WKC_ERROR, ALstatuscode_CHANGE, STATE_DRIFT, DRIVE_STATUS_FLAG), `fault_detail` (human-readable message or code), `recovery_action` (NONE, AUTO_RECOVER, SHUTDOWN_INITIATED).
13. Store all fault events in a separate in-memory buffer (preallocated); export fault log to CSV alongside sample data.

**Phase G — Shutdown & CSV export** (*depends on F*)
14. Disable voltage via ControlWord, then existing `fieldbus_stop()`.
15. Create `data/` if needed; write two CSV files:
    - `data/voice_coil_log_<timestamp>.csv`: header `time_s,ai1_raw,ai2_raw,actual_current_A`, all logged samples.
    - `data/voice_coil_faults_<timestamp>.csv`: header `fault_timestamp_s,fault_type,fault_detail,recovery_action`, all fault events. (Empty if no faults occurred.)

## Relevant files
- `voice-coil.c` — all logic above
- `CMakeLists.txt` — new build target

## Verification
1. Build succeeds with new target.
2. On hardware: console shows successful state transitions (EtherCAT + CiA402).
3. Read-back of 6061h == 0x0A.
4. `voice_coil_log_<timestamp>.csv` has ~30000 rows (at 1ms) with a visible 10Hz/1A sine in `actual_current_A`.
5. WKC remains at expected value throughout (no comm errors printed).
6. `voice_coil_faults_<timestamp>.csv` is either empty (no faults) or contains only expected transient recoveries (e.g., state machine transitions); no unrecoverable faults (SHUTDOWN_INITIATED entries indicate a problem).

## Decisions
- CST mode value 0x0A, scaling formulas from Appendix A (KP read live from 20D8.0Ch), AMC slave detected via vendor ID 0xBD rather than hardcoded index.

## Further Considerations
1. Real-time scheduling (SCHED_FIFO/mlockall) not included by default — can add later if jitter must be tightened further.
2. Assumes a single AMC slave on the bus.
