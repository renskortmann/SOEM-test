/** \file
 * \brief CST current-mode cyclic control + logging for AMC servo drive
 *
 * Usage: voice-coil IFNAME1
 * IFNAME1 is the NIC interface name, e.g. 'eth0'
 *
 * Runs a 30-second hardware-synchronized control loop commanding
 * a sine-wave current to a voice-coil motor, logging samples and faults to CSV.
 */

/* Enable GNU extensions and POSIX.1b features */
#define _GNU_SOURCE
#define _POSIX_C_SOURCE 199309L

#include "soem/soem.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <sys/stat.h>

/** \brief Runtime configuration constants (modify via recompilation) */
#define CYCLE_TIME_MS       1.0  /**< EtherCAT cycle period in milliseconds */
#define SINE_FREQ_HZ        10.0 /**< Target current waveform frequency in Hz */
#define SINE_AMPLITUDE_A    1.0  /**< Target current waveform amplitude in Amps */
#define RUN_DURATION_S      30.0 /**< Total runtime in seconds */
#define CSV_DIR             "data" /**< Output directory for CSV logs */
#define MAX_SAMPLES         ((int)(RUN_DURATION_S / (CYCLE_TIME_MS / 1000.0)) + 100)
#define MAX_FAULTS          1000

#define AMC_VENDOR_ID       0xBD
#define CST_MODE            0x0A
#define STATUS_WORD_MASK    0x6F
#define STATE_READY_TO_SWITCH_ON  0x21
#define STATE_SWITCHED_ON         0x23
#define STATE_OPERATION_ENABLED   0x27
#define CTRL_SHUTDOWN       0x0006
#define CTRL_SWITCH_ON      0x0007
#define CTRL_ENABLE_OP      0x000F
#define CTRL_DISABLE_VOLT   0x0000

/** \brief Master-to-slave process data: CiA402 ControlWord and CST target current command */
typedef struct OSAL_PACKED
{
   uint16_t controlword;     /**< CiA402 control bits (Shutdown/Switch On/Enable Operation) */
   int16_t target_current;   /**< Desired motor current, scaled by KP (DC2 units) */
} rx_pdo_t;

/** \brief Slave-to-master process data: CiA402 StatusWord, actual current, and analog sensor inputs */
typedef struct OSAL_PACKED
{
   uint16_t statusword;      /**< CiA402 status bits (state machine state + fault bits) */
   int16_t actual_current;   /**< Measured motor current from drive, scaled by KP (DC1 units) */
   uint16_t ai1_raw;         /**< Analog Input 1 raw ADC value (0-65535) */
   uint16_t ai2_raw;         /**< Analog Input 2 raw ADC value (0-65535) */
} tx_pdo_t;

/** \brief Fault categories for diagnostic logging */
typedef enum
{
   FAULT_WKC_ERROR = 0,            /**< Working Counter validation failure */
   FAULT_ALstatuscode_CHANGE = 1,  /**< Drive reported non-zero ALstatuscode (reserved for expansion) */
   FAULT_STATE_DRIFT = 2,          /**< Unexpected CiA402 state transition detected */
   FAULT_DRIVE_STATUS_FLAG = 3     /**< Drive status flags (2002h) indicate fault condition */
} fault_type_t;

/** \brief Recovery actions taken in response to detected faults */
typedef enum
{
   RECOVERY_NONE = 0,              /**< Fault logged; operation continues */
   RECOVERY_AUTO_RECOVER = 1,      /**< Automatic recovery attempt made */
   RECOVERY_SHUTDOWN_INITIATED = 2 /**< Unrecoverable fault; controlled shutdown initiated */
} recovery_action_t;

/** \brief A single fault event logged during operation */
typedef struct
{
   double timestamp_s;                 /**< Absolute time when fault was detected */  
   fault_type_t fault_type;            /**< Type of fault that occurred */
   uint32_t fault_detail;              /**< Additional detail (e.g., ALstatuscode value or WKC mismatch count) */ 
   recovery_action_t recovery_action;  /**< Action taken in response to the fault (if any) */
} fault_log_entry_t;

/** \brief Timestamped data sample from one cycle of the real-time loop */
typedef struct
{
   double timestamp_s;       /**< Absolute time when sample was acquired */
   uint16_t ai1_raw;         /**< Analog input 1 raw value at this timestamp */
   uint16_t ai2_raw;         /**< Analog input 2 raw value at this timestamp */
   double actual_current_A;  /**< Actual motor current in physical Amps at this timestamp */
} sample_log_entry_t;

/** \brief Master state container: EtherCAT protocol context, drive parameters, and sample/fault buffers */
typedef struct
{
   ecx_contextt context;           /**< SOEM EtherCAT context with slave list and I/O mapping */
   char *iface;                    /**< Network interface name (e.g., "eth0") */
   uint8 group;                    /**< I/O group index (0 for single-group setup) */
   int roundtrip_time;             /**< Last measured PDO roundtrip time in microseconds */
   uint8 map[4096];                /**< I/O mapping buffer for ecx_config_map_group() */
   double kp_amps;                 /**< Drive peak current rating (read from object 20D8.0Ch); used for current scaling */
   uint16_t amc_slave_index;       /**< Slave index of detected AMC drive (1-based) */
   sample_log_entry_t *samples;    /**< Preallocated buffer for cyclic samples */
   int sample_count;               /**< Number of samples logged so far */
   fault_log_entry_t *faults;      /**< Preallocated buffer for fault events */
   int fault_count;                /**< Number of fault events logged so far */
} Fieldbus;


/** \brief Initialize Fieldbus structure with zero values and interface name
 *  \param fieldbus Pointer to Fieldbus structure to initialize
 *  \param iface Network interface name (e.g., "eth0")
 */
static void
fieldbus_initialize(Fieldbus *fieldbus, char *iface)
{
   memset(fieldbus, 0, sizeof(*fieldbus));
   fieldbus->iface = iface;
   fieldbus->group = 0;
   fieldbus->roundtrip_time = 0;
   fieldbus->kp_amps = 0.0;
   fieldbus->amc_slave_index = 0;
   fieldbus->sample_count = 0;
   fieldbus->fault_count = 0;
}

/** \brief Send and receive one cycle of process data, measure roundtrip time
 *  \param fieldbus Fieldbus context
 *  \return Working Counter (WKC) from ecx_receive_processdata()
 */
static int
fieldbus_roundtrip(Fieldbus *fieldbus)
{
   ecx_contextt *context = &fieldbus->context;
   ec_timet start, end, diff;
   int wkc;

   start = osal_current_time();
   ecx_send_processdata(context);
   wkc = ecx_receive_processdata(context, EC_TIMEOUTRET);
   end = osal_current_time();
   osal_time_diff(&start, &end, &diff);
   fieldbus->roundtrip_time = (int)(diff.tv_sec * 1000000 + diff.tv_nsec / 1000);

   return wkc;
}

/** \brief Configure AMC servo drive during PreOp→SafeOp transition (PO2SOconfig hook)
 *  Sets Cyclic Synchronous Torque (CST) mode, interpolation period, PDO mappings, and reads peak current (KP).
 *  Called automatically by ecx_config_map_group() for any slave with PO2SOconfig assigned.
 *  \param context SOEM EtherCAT context
 *  \param slave Slave index (1-based)
 *  \return 1 on success, 0 on failure
 */
static int
amc_slave_config(ecx_contextt *context, uint16 slave)
{
   Fieldbus *fieldbus = (Fieldbus *)context->userdata;
   int retval = 0;
   int psize;
   int8 mode = CST_MODE;
   uint16 ctrl_word = CTRL_SHUTDOWN;
   uint32 kp_raw;
   double cycle_s = CYCLE_TIME_MS / 1000.0;
   uint8 mantissa;
   int8 exponent;
   uint32 map_1600[3];
   uint32 map_1a00[5];
   int i;

   printf("Configuring AMC slave %d via SDO...\n", slave);

   /* Set mode to CST (0x0A) */
   psize = sizeof(mode);
   retval += ecx_SDOwrite(context, slave, 0x6060, 0x00, FALSE, psize, &mode, EC_TIMEOUTSAFE);
   printf("  Set mode 6060h = 0x%02X: %s\n", mode, retval > 0 ? "OK" : "FAILED");

   /* Compute and write interpolation time period (60C2h) */
   if (cycle_s >= 0.1)
   {
      mantissa = (uint8)(cycle_s / 0.1);
      exponent = -1;
   }
   else if (cycle_s >= 0.01)
   {
      mantissa = (uint8)(cycle_s / 0.01);
      exponent = -2;
   }
   else
   {
      mantissa = (uint8)(cycle_s / 0.001);
      exponent = -3;
   }
   psize = sizeof(mantissa);
   retval += ecx_SDOwrite(context, slave, 0x60C2, 0x01, FALSE, psize, &mantissa, EC_TIMEOUTSAFE);
   psize = sizeof(exponent);
   retval += ecx_SDOwrite(context, slave, 0x60C2, 0x02, FALSE, psize, &exponent, EC_TIMEOUTSAFE);
   printf("  Set interpolation period 60C2h: mantissa=0x%02X, exponent=%d: %s\n", 
          mantissa, exponent, retval > 0 ? "OK" : "FAILED");

   /* Configure RxPDO mapping (1600h): ControlWord + Target Current */
   psize = 1;
   uint8 zero = 0;
   retval += ecx_SDOwrite(context, slave, 0x1600, 0x00, FALSE, psize, &zero, EC_TIMEOUTSAFE);
   
   map_1600[0] = 3;
   map_1600[1] = (0x6040 << 16) | (0x00 << 8) | 0x10;
   map_1600[2] = (0x6071 << 16) | (0x00 << 8) | 0x10;
   
   for (i = 0; i < 2; i++)
   {
      psize = sizeof(uint32);
      retval += ecx_SDOwrite(context, slave, 0x1600, 0x01 + i, FALSE, psize, 
                             &map_1600[i + 1], EC_TIMEOUTSAFE);
   }
   psize = 1;
   uint8 two = 2;
   retval += ecx_SDOwrite(context, slave, 0x1600, 0x00, FALSE, psize, &two, EC_TIMEOUTSAFE);
   printf("  Configured RxPDO 1600h: ControlWord + Target Current: %s\n", retval > 0 ? "OK" : "FAILED");

   /* Configure TxPDO mapping (1A00h): StatusWord + Actual Current + AI1 raw + AI2 raw */
   psize = 1;
   retval += ecx_SDOwrite(context, slave, 0x1A00, 0x00, FALSE, psize, &zero, EC_TIMEOUTSAFE);
   
   map_1a00[0] = 4;
   map_1a00[1] = (0x6041 << 16) | (0x00 << 8) | 0x10;
   map_1a00[2] = (0x6077 << 16) | (0x00 << 8) | 0x10;
   map_1a00[3] = (0x2022 << 16) | (0x01 << 8) | 0x10;
   map_1a00[4] = (0x2022 << 16) | (0x02 << 8) | 0x10;
   
   for (i = 0; i < 4; i++)
   {
      psize = sizeof(uint32);
      retval += ecx_SDOwrite(context, slave, 0x1A00, 0x01 + i, FALSE, psize, 
                             &map_1a00[i + 1], EC_TIMEOUTSAFE);
   }
   psize = 1;
   uint8 four = 4;
   retval += ecx_SDOwrite(context, slave, 0x1A00, 0x00, FALSE, psize, &four, EC_TIMEOUTSAFE);
   printf("  Configured TxPDO 1A00h: StatusWord + Actual Current + AI1/AI2 raw: %s\n", retval > 0 ? "OK" : "FAILED");

   /* Read Maximum Peak Current (20D8.0Ch) for current scaling */
   psize = sizeof(uint32);
   retval += ecx_SDOread(context, slave, 0x20D8, 0x0C, FALSE, &psize, &kp_raw, EC_TIMEOUTSAFE);
   fieldbus->kp_amps = kp_raw / 10.0;
   printf("  Read Maximum Peak Current (KP): raw=0x%04X -> %.1f A\n", kp_raw, fieldbus->kp_amps);

   fieldbus->amc_slave_index = slave;
   return (retval > 0) ? 1 : 0;
}

/** \brief Initialize EtherCAT network: detect slaves, assign config hook, map I/O, configure DC, reach Operational
 *  Detects AMC servo drive by vendor ID (0xBD) and assigns amc_slave_config PO2SOconfig hook.
 *  \param fieldbus Fieldbus context
 *  \return TRUE on success, FALSE if any step fails
 */
static boolean
fieldbus_start(Fieldbus *fieldbus)
{
   ecx_contextt *context = &fieldbus->context;
   ec_groupt *grp;
   ec_slavet *slave;
   int i;

   printf("Initializing SOEM on '%s'... ", fieldbus->iface);
   if (!ecx_init(context, fieldbus->iface))
   {
      printf("no socket connection\n");
      return FALSE;
   }
   printf("done\n");

   printf("Finding autoconfig slaves... ");
   if (ecx_config_init(context) <= 0)
   {
      printf("no slaves found\n");
      return FALSE;
   }
   printf("%d slaves found\n", context->slavecount);

   /* Set userdata for PO2SOconfig hook access */
   context->userdata = (void *)fieldbus;

   /* Assign PO2SOconfig hook to AMC slave (vendor ID 0xBD) */
   for (i = 1; i <= context->slavecount; i++)
   {
      slave = context->slavelist + i;
      if (slave->eep_man == AMC_VENDOR_ID)
      {
         printf("Found AMC slave at index %d, assigning PO2SOconfig hook\n", i);
         slave->PO2SOconfig = amc_slave_config;
      }
   }

   printf("Sequential mapping of I/O... ");
   ecx_config_map_group(context, fieldbus->map, fieldbus->group);
   grp = context->grouplist + fieldbus->group;
   printf("mapped %dO+%dI bytes\n", grp->Obytes, grp->Ibytes);

   printf("Configuring distributed clock... ");
   ecx_configdc(context);
   printf("done\n");

   printf("Waiting for all slaves in safe operational... ");
   ecx_statecheck(context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
   printf("done\n");

   printf("Send a roundtrip to make outputs in slaves happy... ");
   fieldbus_roundtrip(fieldbus);
   printf("done\n");

   printf("Setting operational state...");
   slave = context->slavelist;
   slave->state = EC_STATE_OPERATIONAL;
   ecx_writestate(context, 0);
   for (i = 0; i < 10; ++i)
   {
      printf(".");
      fieldbus_roundtrip(fieldbus);
      ecx_statecheck(context, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE / 10);
      if (slave->state == EC_STATE_OPERATIONAL)
      {
         printf(" all slaves are now operational\n");
         return TRUE;
      }
   }
   printf(" failed\n");
   return FALSE;
}

/** \brief Shut down EtherCAT network: request Init state on all slaves and close socket
 *  \param fieldbus Fieldbus context
 */
static void
fieldbus_stop(Fieldbus *fieldbus)
{
   ecx_contextt *context = &fieldbus->context;
   ec_slavet *slave = context->slavelist;

   printf("Requesting init state on all slaves... ");
   slave->state = EC_STATE_INIT;
   ecx_writestate(context, 0);
   printf("done\n");

   printf("Close socket... ");
   ecx_close(context);
   printf("done\n");
}

/** \brief Progress drive through CiA402 state machine to Operation Enabled
 *  Executes state transitions: Shutdown→Ready to Switch On→Switched On→Operation Enabled
 *  Each step polls StatusWord via PDO exchange until target state reached or timeout.
 *  \param fieldbus Fieldbus context
 *  \return TRUE when Operation Enabled reached, FALSE on timeout
 */
static boolean
cia402_bring_up(Fieldbus *fieldbus)
{
   ecx_contextt *context = &fieldbus->context;
   ec_groupt *grp = context->grouplist + fieldbus->group;
   rx_pdo_t *rx = (rx_pdo_t *)grp->outputs;
   tx_pdo_t *tx = (tx_pdo_t *)grp->inputs;
   uint16_t current_state;
   int timeout_cycles = 3000;
   int cycles = 0;
   int wkc, expected_wkc;

   expected_wkc = grp->outputsWKC * 2 + grp->inputsWKC;

   printf("\nBringing drive through CiA402 state machine:\n");

   /* Shutdown → Ready to Switch On (0x06 → 0x21) */
   printf("  [1/3] Shutdown → Ready to Switch On...");
   rx->controlword = CTRL_SHUTDOWN;
   cycles = 0;
   while (cycles < timeout_cycles)
   {
      ecx_send_processdata(context);
      wkc = ecx_receive_processdata(context, EC_TIMEOUTRET);
      current_state = tx->statusword & STATUS_WORD_MASK;
      if (current_state == STATE_READY_TO_SWITCH_ON)
      {
         printf(" OK (StatusWord=0x%04X)\n", tx->statusword);
         break;
      }
      cycles++;
      osal_usleep(1000);
   }
   if (cycles >= timeout_cycles)
   {
      printf(" TIMEOUT\n");
      return FALSE;
   }

   /* Switch On (0x07 → 0x23) */
   printf("  [2/3] Switch On → Switched On...");
   rx->controlword = CTRL_SWITCH_ON;
   cycles = 0;
   while (cycles < timeout_cycles)
   {
      ecx_send_processdata(context);
      wkc = ecx_receive_processdata(context, EC_TIMEOUTRET);
      current_state = tx->statusword & STATUS_WORD_MASK;
      if (current_state == STATE_SWITCHED_ON)
      {
         printf(" OK (StatusWord=0x%04X)\n", tx->statusword);
         break;
      }
      cycles++;
      osal_usleep(1000);
   }
   if (cycles >= timeout_cycles)
   {
      printf(" TIMEOUT\n");
      return FALSE;
   }

   /* Enable Operation (0x0F → 0x27) */
   printf("  [3/3] Enable Operation...");
   rx->controlword = CTRL_ENABLE_OP;
   cycles = 0;
   while (cycles < timeout_cycles)
   {
      ecx_send_processdata(context);
      wkc = ecx_receive_processdata(context, EC_TIMEOUTRET);
      current_state = tx->statusword & STATUS_WORD_MASK;
      if (current_state == STATE_OPERATION_ENABLED)
      {
         printf(" OK (StatusWord=0x%04X)\n", tx->statusword);
         return TRUE;
      }
      cycles++;
      osal_usleep(1000);
   }
   printf(" TIMEOUT\n");
   return FALSE;
}

/** \brief State for distributed clock drift compensation integral controller */
struct ec_sync_t
{
   int64_t last_DCtime;  /**< Last observed DC time for drift calculation */
   int32_t sync_error;   /**< Accumulated drift error in DC time units */
};

/** \brief Global instance of distributed clock sync state */
static struct ec_sync_t sync_obj = {0, 0};

/** \brief Update drift compensation state (not actively used in this implementation, but provided for future real-time tuning)
 *  \param context SOEM EtherCAT context
 *  \param cycletime Cycle time (unused in this version)
 *  \param sync Sync state to update
 */
static void
ec_sync(ecx_contextt *context, ec_timet *cycletime, struct ec_sync_t *sync)
{
   int32_t delta = (int32_t)((context->DCtime - sync->last_DCtime) / 1000);
   sync->sync_error += delta;
   sync->last_DCtime = context->DCtime;
}

/** \brief Add microseconds to a struct timespec, handling nanosecond overflow
 *  \param ts Pointer to timespec to modify in-place
 *  \param addus Microseconds to add
 */
static void
add_timespec(struct timespec *ts, int64_t addus)
{
   ts->tv_nsec += addus * 1000;
   while (ts->tv_nsec >= 1000000000)
   {
      ts->tv_sec++;
      ts->tv_nsec -= 1000000000;
   }
}

/** \brief Record one timestamped sample to the in-memory sample buffer
 *  Converts actual current from raw Int16 (DC1 scaling) to physical Amps using KP.
 *  \param fieldbus Fieldbus context (buffer and count updated)
 *  \param timestamp_s Absolute time in seconds
 *  \param tx Pointer to received TxPDO data
 */
static void
log_sample(Fieldbus *fieldbus, double timestamp_s, const tx_pdo_t *tx)
{
   if (fieldbus->sample_count < MAX_SAMPLES)
   {
      double actual_current_A = (tx->actual_current * fieldbus->kp_amps) / 8192.0;
      fieldbus->samples[fieldbus->sample_count].timestamp_s = timestamp_s;
      fieldbus->samples[fieldbus->sample_count].ai1_raw = tx->ai1_raw;
      fieldbus->samples[fieldbus->sample_count].ai2_raw = tx->ai2_raw;
      fieldbus->samples[fieldbus->sample_count].actual_current_A = actual_current_A;
      fieldbus->sample_count++;
   }
}

/** \brief Record one fault event to the in-memory fault buffer and print to console
 *  \param fieldbus Fieldbus context (fault buffer and count updated)
 *  \param timestamp_s Absolute time when fault was detected
 *  \param fault_type Category of fault (WKC_ERROR, STATE_DRIFT, etc.)
 *  \param fault_detail Fault-specific data (WKC value, state code, etc.)
 *  \param recovery_action What action was/will be taken (NONE, AUTO_RECOVER, SHUTDOWN_INITIATED)
 */
static void
log_fault(Fieldbus *fieldbus, double timestamp_s, fault_type_t fault_type, 
          uint32_t fault_detail, recovery_action_t recovery_action)
{
   if (fieldbus->fault_count < MAX_FAULTS)
   {
      fieldbus->faults[fieldbus->fault_count].timestamp_s = timestamp_s;
      fieldbus->faults[fieldbus->fault_count].fault_type = fault_type;
      fieldbus->faults[fieldbus->fault_count].fault_detail = fault_detail;
      fieldbus->faults[fieldbus->fault_count].recovery_action = recovery_action;
      fieldbus->fault_count++;
      
      const char *fault_name[] = {"WKC_ERROR", "ALstatuscode_CHANGE", "STATE_DRIFT", "DRIVE_STATUS_FLAG"};
      const char *recovery_name[] = {"NONE", "AUTO_RECOVER", "SHUTDOWN_INITIATED"};
      printf("[FAULT] t=%.3fs type=%s detail=0x%04X recovery=%s\n", 
             timestamp_s, fault_name[fault_type], fault_detail, recovery_name[recovery_action]);
   }
}

/** \brief Run the real-time control loop: generate sine-wave commands, exchange PDO, monitor faults
 *  Operates for RUN_DURATION_S seconds at CYCLE_TIME_MS intervals, synchronized via DC SYNC0.
 *  Detects WKC errors and CiA402 state drift; logs samples and faults to in-memory buffers.
 *  \param fieldbus Fieldbus context (sample and fault buffers populated)
 *  \return TRUE on completion (normal or fault-triggered shutdown), FALSE on internal error
 */
static boolean
fieldbus_run_cyclic(Fieldbus *fieldbus)
{
   ecx_contextt *context = &fieldbus->context;
   ec_groupt *grp = context->grouplist + fieldbus->group;
   rx_pdo_t *rx = (rx_pdo_t *)grp->outputs;
   tx_pdo_t *tx = (tx_pdo_t *)grp->inputs;
   int expected_wkc = grp->outputsWKC * 2 + grp->inputsWKC;
   
   struct timespec next_cycle, now;
   int64_t cycle_ns = (int64_t)(CYCLE_TIME_MS * 1000000);
   double elapsed_s = 0.0;
   double sine_phase;
   double target_current_A;
   int16_t target_current_raw;
   int wkc;
   int wkc_error_count = 0;
   int cycle_count = 0;
   uint16_t last_statusword = 0;
   uint16_t current_state;
   int fault_check_counter = 0;

   printf("\nStarting 30-second cyclic loop...\n");
   
   /* Configure DC SYNC0 */
   ecx_dcsync0(context, fieldbus->amc_slave_index, TRUE, (uint32_t)cycle_ns, 0);

   clock_gettime(CLOCK_MONOTONIC, &next_cycle);

   while (elapsed_s < RUN_DURATION_S)
   {
      /* Compute target current (sine wave) */
      sine_phase = 2.0 * M_PI * SINE_FREQ_HZ * elapsed_s;
      target_current_A = SINE_AMPLITUDE_A * sin(sine_phase);
      
      /* Convert to raw Int16 (scale: 2^15 / KP) */
      target_current_raw = (int16_t)round((target_current_A * 32768.0) / fieldbus->kp_amps);
      if (target_current_raw > 32767) target_current_raw = 32767;
      if (target_current_raw < -32768) target_current_raw = -32768;
      
      rx->target_current = target_current_raw;

      /* Send and receive process data */
      ecx_send_processdata(context);
      wkc = ecx_receive_processdata(context, EC_TIMEOUTRET);

      /* Working Counter validation: detect frame loss or slave response failure */
      if (wkc < expected_wkc)
      {
         wkc_error_count++;
         log_fault(fieldbus, elapsed_s, FAULT_WKC_ERROR, wkc, RECOVERY_AUTO_RECOVER);
         if (wkc_error_count > 5)
         {
            printf("ERROR: WKC errors exceeded threshold, initiating shutdown\n");
            log_fault(fieldbus, elapsed_s, FAULT_WKC_ERROR, wkc, RECOVERY_SHUTDOWN_INITIATED);
            break;
         }
      }
      else
      {
         wkc_error_count = 0;
      }

      /* State machine validation: ensure drive remains in Operation Enabled */
      current_state = tx->statusword & STATUS_WORD_MASK;
      if (current_state != STATE_OPERATION_ENABLED)
      {
         log_fault(fieldbus, elapsed_s, FAULT_STATE_DRIFT, current_state, RECOVERY_SHUTDOWN_INITIATED);
         printf("ERROR: Drive dropped out of OPERATION_ENABLED state (0x%04X), initiating shutdown\n", 
                tx->statusword);
         rx->controlword = CTRL_DISABLE_VOLT;
         break;
      }

      /* Periodic drive status polling (placeholder for future SDO-based health checks) */
      fault_check_counter++;
      if (fault_check_counter >= 100)
      {
         fault_check_counter = 0;
         /* TODO: SDO-read 2002h (Drive Status: Bridge/Protection/System flags) for comprehensive diagnostics */
      }

      /* Log sample */
      log_sample(fieldbus, elapsed_s, tx);

      /* Wait for next cycle using absolute-time sleep */
      clock_gettime(CLOCK_MONOTONIC, &now);
      if (now.tv_sec > next_cycle.tv_sec || 
          (now.tv_sec == next_cycle.tv_sec && now.tv_nsec > next_cycle.tv_nsec))
      {
         printf("WARNING: Cycle %d missed deadline\n", cycle_count);
      }
      else
      {
         clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_cycle, NULL);
      }
      
      add_timespec(&next_cycle, cycle_ns / 1000);
      elapsed_s = (double)cycle_count * CYCLE_TIME_MS / 1000.0;
      cycle_count++;
   }

   printf("Cyclic loop finished. Samples: %d, Faults: %d\n", fieldbus->sample_count, fieldbus->fault_count);
   return TRUE;
}

/** \brief Write sample and fault buffers to timestamped CSV files in CSV_DIR
 *  Creates two files:
 *    - voice_coil_log_YYYYMMDD_HHMMSS.csv: samples (time_s, ai1_raw, ai2_raw, actual_current_A)
 *    - voice_coil_faults_YYYYMMDD_HHMMSS.csv: fault events (timestamp, type, detail, action)
 *  \param fieldbus Fieldbus context with populated buffers
 */
static void
export_csv(Fieldbus *fieldbus)
{
   time_t now = time(NULL);
   struct tm *tm_info = localtime(&now);
   char timestamp[32];
   char sample_file[256];
   char fault_file[256];
   FILE *fp;
   int i;

   strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", tm_info);

   /* Create data directory */
   mkdir(CSV_DIR, 0755);

   /* Write sample log */
   snprintf(sample_file, sizeof(sample_file), "%s/voice_coil_log_%s.csv", CSV_DIR, timestamp);
   fp = fopen(sample_file, "w");
   if (fp)
   {
      fprintf(fp, "time_s,ai1_raw,ai2_raw,actual_current_A\n");
      for (i = 0; i < fieldbus->sample_count; i++)
      {
         fprintf(fp, "%.6f,%u,%u,%.6f\n",
                 fieldbus->samples[i].timestamp_s,
                 fieldbus->samples[i].ai1_raw,
                 fieldbus->samples[i].ai2_raw,
                 fieldbus->samples[i].actual_current_A);
      }
      fclose(fp);
      printf("Wrote %d samples to %s\n", fieldbus->sample_count, sample_file);
   }
   else
   {
      printf("ERROR: Could not open %s for writing\n", sample_file);
   }

   /* Write fault log */
   snprintf(fault_file, sizeof(fault_file), "%s/voice_coil_faults_%s.csv", CSV_DIR, timestamp);
   fp = fopen(fault_file, "w");
   if (fp)
   {
      fprintf(fp, "fault_timestamp_s,fault_type,fault_detail,recovery_action\n");
      const char *fault_names[] = {"WKC_ERROR", "ALstatuscode_CHANGE", "STATE_DRIFT", "DRIVE_STATUS_FLAG"};
      const char *recovery_names[] = {"NONE", "AUTO_RECOVER", "SHUTDOWN_INITIATED"};
      
      for (i = 0; i < fieldbus->fault_count; i++)
      {
         fprintf(fp, "%.6f,%s,0x%04X,%s\n",
                 fieldbus->faults[i].timestamp_s,
                 fault_names[fieldbus->faults[i].fault_type],
                 fieldbus->faults[i].fault_detail,
                 recovery_names[fieldbus->faults[i].recovery_action]);
      }
      fclose(fp);
      printf("Wrote %d fault events to %s\n", fieldbus->fault_count, fault_file);
   }
   else
   {
      printf("ERROR: Could not open %s for writing\n", fault_file);
   }
}

/** \brief Main function for the voice-coil application
 * \param argc Number of command-line arguments
 * \param argv Array of command-line arguments
 * \return 0 on success, 1 on failure
 */
int main(int argc, char *argv[])
{
   Fieldbus fieldbus;
   ec_adaptert *adapter;

   if (argc != 2)
   {
      printf("Usage: voice-coil IFNAME\n");
      printf("IFNAME is the NIC interface name, e.g. 'eth0'\n\n");
      printf("Available adapters:\n");
      adapter = ec_find_adapters();
      while (adapter)
      {
         printf("  - %s  (%s)\n", adapter->name, adapter->desc);
         adapter = adapter->next;
      }
      return 1;
   }

   fieldbus_initialize(&fieldbus, argv[1]);
   
   /* Allocate buffers */
   fieldbus.samples = (sample_log_entry_t *)malloc(MAX_SAMPLES * sizeof(sample_log_entry_t));
   fieldbus.faults = (fault_log_entry_t *)malloc(MAX_FAULTS * sizeof(fault_log_entry_t));
   
   if (!fieldbus.samples || !fieldbus.faults)
   {
      printf("ERROR: Memory allocation failed\n");
      return 1;
   }

   if (fieldbus_start(&fieldbus))
   {
      if (cia402_bring_up(&fieldbus))
      {
         fieldbus_run_cyclic(&fieldbus);
      }
      else
      {
         printf("ERROR: Failed to bring up CiA402 state machine\n");
      }
      
      /* Disable voltage and shutdown */
      rx_pdo_t *rx = (rx_pdo_t *)fieldbus.context.grouplist[fieldbus.group].outputs;
      rx->controlword = CTRL_DISABLE_VOLT;
      fieldbus_roundtrip(&fieldbus);
      
      fieldbus_stop(&fieldbus);
   }

   export_csv(&fieldbus);

   free(fieldbus.samples);
   free(fieldbus.faults);

   return 0;
}
