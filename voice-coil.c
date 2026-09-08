/** \file
 * \brief CST current-mode cyclic control + logging for AMC servo drive
 *
 * Usage: voice-coil IFNAME1
 * IFNAME1 is the NIC interface name, e.g. 'eth0'
 *
 * Runs a RUN_DURATION_S-second hardware-synchronized control loop commanding
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
#include <sched.h>      /* sched_setscheduler, sched_get_priority_max, SCHED_FIFO */
#include <sys/mman.h>   /* mlockall, MCL_CURRENT, MCL_FUTURE */
#include <errno.h>      /* strerror(errno) for error messages */

/** \brief Runtime configuration constants (modify via recompilation) */
#define CYCLE_TIME_MS       1.0  /**< EtherCAT cycle period in milliseconds */
#define SINE_FREQ_HZ        10.0 /**< Target current waveform frequency in Hz */
#define SINE_AMPLITUDE_A    2.0  /**< Target current waveform amplitude in Amps */
#define RUN_DURATION_S      5.0 /**< Total runtime in seconds */
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

/** \brief Slave-to-master process data: CiA402 StatusWord, actual current, current demand feedback, and analog sensor inputs */
typedef struct OSAL_PACKED
{
   uint16_t statusword;      /**< CiA402 status bits (state machine state + fault bits) */
   int16_t actual_current;   /**< Measured motor current from drive, scaled by KP (DC1 units) */
   int16_t current_demand;   /**< Drive's internal current demand feedback (2010.02h), DC2 units */
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
   double target_current_A_sent;     /**< Target current sent by master (before PDO encoding) */
   double target_current_A_received; /**< Target current as received by slave from 2010.01h (DC2) */
   double actual_current_A;  /**< Actual motor current in physical Amps at this timestamp */
   double current_demand_A;  /**< Drive's reported current demand in Amps at this timestamp (2010.02h decoded) */
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
   uint32 map_1600[2];
   uint32 map_1a00[5];
   int i;

   printf("Configuring AMC slave %d via SDO...\n", slave);

   /* Set mode to CST (0x0A) */
   retval += ecx_SDOwrite(context, slave, 0x6060, 0x00, FALSE, (int)sizeof(mode), &mode, EC_TIMEOUTSAFE);
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
   // Write mantissa and exponent to 60C2.01h and 60C2.02h
   retval += ecx_SDOwrite(context, slave, 0x60C2, 0x01, FALSE, sizeof(mantissa), &mantissa, EC_TIMEOUTSAFE);
   retval += ecx_SDOwrite(context, slave, 0x60C2, 0x02, FALSE, sizeof(exponent), &exponent, EC_TIMEOUTSAFE);
   printf("  Set interpolation period 60C2h: mantissa=0x%02X, exponent=%d: %s\n", 
          mantissa, exponent, retval > 0 ? "OK" : "FAILED");

   /* Configure RxPDO mapping (1600h): ControlWord + Target Current */
   // Clear existing mapping by writing 0 to 1600.00h
   retval += ecx_SDOwrite(context, slave, 0x1600, 0x00, FALSE, sizeof(uint8), &(uint8){0}, EC_TIMEOUTSAFE);
   
   // Define new mapping entries for 1600h: ControlWord (6040.00h) and Target Current (6071.00h)
   map_1600[0] = (0x6040 << 16) | (0x00 << 8) | 0x10;
   map_1600[1] = (0x6071 << 16) | (0x00 << 8) | 0x10;
   
   // Write mapping entries to 1600.01h (1st application object) and 1600.02h (2nd application object)
   for (i = 0; i < 2; i++)
      retval += ecx_SDOwrite(context, slave, 0x1600, 0x01 + i, FALSE, sizeof(map_1600[0]), map_1600 + i, EC_TIMEOUTSAFE);

   // Set number of mapped objects to 2 by writing 2 to 1600.00h
   retval += ecx_SDOwrite(context, slave, 0x1600, 0x00, FALSE, sizeof(uint8), &(uint8){2}, EC_TIMEOUTSAFE);
   printf("  Configured RxPDO 1600h: ControlWord + Target Current: %s\n", retval > 0 ? "OK" : "FAILED");

   // Read back 1600.00h to verify number of mapped objects
   uint8 count = 0;
   psize = sizeof(count);
   ecx_SDOread(context, slave, 0x1600, 0x00, FALSE, &psize, &count, EC_TIMEOUTSAFE);
   printf("1600.00h count = %d (expect 2)\n", count);

   for (int i = 0; i < count; i++) {
      uint32 entry = 0;
      psize = sizeof(entry);
      ecx_SDOread(context, slave, 0x1600, 0x01 + i, FALSE, &psize, &entry, EC_TIMEOUTSAFE);
      printf("1600.%02Xh = %08Xh  -> index=%04Xh sub=%02Xh len=%u bits\n",
            0x01 + i, entry, entry >> 16, (entry >> 8) & 0xFF, entry & 0xFF);
   }

   /* Configure TxPDO mapping (1A00h): StatusWord + Actual Current + Target Current + AI1 raw + AI2 raw */
   // Clear existing mapping by writing 0 to 1A00.00h
   retval += ecx_SDOwrite(context, slave, 0x1A00, 0x00, FALSE, sizeof(uint8), &(uint8){0}, EC_TIMEOUTSAFE);
   
   // Define new mapping entries for 1A00h (TxPDO): StatusWord (6041.00h), Actual Current (6077.00h), Target Current (6071.00h), AI1 raw (2022.01h), AI2 raw (2022.02h)
   // uint32 map_1a00_ext[5];
   map_1a00[0] = (0x6041 << 16) | (0x00 << 8) | 0x10;
   map_1a00[1] = (0x6077 << 16) | (0x00 << 8) | 0x10;
   map_1a00[2] = (0x6071 << 16) | (0x00 << 8) | 0x10;
   map_1a00[3] = (0x2022 << 16) | (0x01 << 8) | 0x10;
   map_1a00[4] = (0x2022 << 16) | (0x02 << 8) | 0x10;
   
   // Write mapping entries to 1A00.01h (1st application object) through 1A00.05h (5th application object)
   for (i = 0; i < 5; i++)
      retval += ecx_SDOwrite(context, slave, 0x1A00, 0x01 + i, FALSE, (int)sizeof(uint32), map_1a00 + i, EC_TIMEOUTSAFE);

   // Set number of mapped objects to 5 by writing 5 to 1A00.00h (number of mapped objects)
   retval += ecx_SDOwrite(context, slave, 0x1A00, 0x00, FALSE, 1, &(uint8){5}, EC_TIMEOUTSAFE);
   printf("  Configured TxPDO 1A00h: StatusWord + Actual Current + Target Current + AI1/AI2 raw: %s\n", retval > 0 ? "OK" : "FAILED");

   // Read back 1A00.00h to verify number of mapped objects
   count = 0;
   psize = sizeof(count);
   ecx_SDOread(context, slave, 0x1A00, 0x00, FALSE, &psize, &count, EC_TIMEOUTSAFE);
   printf("1A00.00h count = %d (expect 5)\n", count);

   for (int i = 0; i < count; i++) {
      uint32 entry = 0;
      psize = sizeof(entry);
      ecx_SDOread(context, slave, 0x1A00, 0x01 + i, FALSE, &psize, &entry, EC_TIMEOUTSAFE);
      printf("1A00.%02Xh = %08Xh  -> index=%04Xh sub=%02Xh len=%u bits\n",
            0x01 + i, entry, entry >> 16, (entry >> 8) & 0xFF, entry & 0xFF);
   }

   /* Read Maximum Peak Current (20D8.0Ch) for current scaling */
   // psize = sizeof(uint32);
   retval += ecx_SDOread(context, slave, 0x20D8, 0x0C, FALSE, &psize, &kp_raw, EC_TIMEOUTSAFE);
   fieldbus->kp_amps = kp_raw / 10.0;
   printf("  Read Maximum Peak Current (KP): raw=0x%08X -> %.1f A\n", kp_raw, fieldbus->kp_amps);

   /* === EXPLORATORY READS: Watchdog & Synchronization Diagnostics === */
   printf("\n  Diagnostic Reads (Synchronization & Watchdog Configuration):\n");

   /* Read Device Type (1000h) */
   uint32_t device_type = 0;
   psize = sizeof(device_type);
   if (ecx_SDOread(context, slave, 0x1000, 0x00, FALSE, &psize, &device_type, EC_TIMEOUTSAFE) > 0)
   {
      printf("    1000h Device Type: 0x%08X", device_type);
      if ((device_type & 0xFFFF) == 0x0192)
         printf(" [CiA402 Servo Drive]");
      printf("\n");
   }

   /* Read Identity Object (1018h sub-indices) */
   uint32_t vendor_id = 0, product_code = 0, revision = 0, serial = 0;
   psize = sizeof(vendor_id);
   if (ecx_SDOread(context, slave, 0x1018, 0x01, FALSE, &psize, &vendor_id, EC_TIMEOUTSAFE) > 0)
      printf("    1018.01h Vendor ID: 0x%02X\n", vendor_id);
   
   psize = sizeof(product_code);
   if (ecx_SDOread(context, slave, 0x1018, 0x02, FALSE, &psize, &product_code, EC_TIMEOUTSAFE) > 0)
      printf("    1018.02h Product Code: 0x%08X\n", product_code);
   
   psize = sizeof(revision);
   if (ecx_SDOread(context, slave, 0x1018, 0x03, FALSE, &psize, &revision, EC_TIMEOUTSAFE) > 0)
      printf("    1018.03h Revision: 0x%08X\n", revision);
   
   psize = sizeof(serial);
   if (ecx_SDOread(context, slave, 0x1018, 0x04, FALSE, &psize, &serial, EC_TIMEOUTSAFE) > 0)
      printf("    1018.04h Serial Number: 0x%08X\n", serial);

   /* Read Sync Manager Communication Type (1C00h) */
   uint8_t sm_channels = 0;
   psize = sizeof(sm_channels);
   if (ecx_SDOread(context, slave, 0x1C00, 0x00, FALSE, &psize, &sm_channels, EC_TIMEOUTSAFE) > 0)
      printf("    1C00.00h Sync Manager Channels: %d\n", sm_channels);

   /* Read Event Action for Comm Channel Error (2065.21h) - watchdog behavior */
   uint16_t comm_error_action = 0;
   psize = sizeof(comm_error_action);
   if (ecx_SDOread(context, slave, 0x2065, 0x21, FALSE, &psize, &comm_error_action, EC_TIMEOUTSAFE) > 0)
   {
      printf("    2065.21h Comm Channel Error Action: 0x%04X", comm_error_action);
      if (comm_error_action == 0)
         printf(" [No Action]");
      else if (comm_error_action == 1)
         printf(" [Fault]");
      else if (comm_error_action == 2)
         printf(" [Shutdown]");
      printf("\n");
   }

   /* Verify Interpolation Time Period (60C2.01h) is set correctly */
   uint8_t interp_mantissa = 0;
   int8_t interp_exponent = 0;
   psize = sizeof(interp_mantissa);
   if (ecx_SDOread(context, slave, 0x60C2, 0x01, FALSE, &psize, &interp_mantissa, EC_TIMEOUTSAFE) > 0)
   {
      psize = sizeof(interp_exponent);
      if (ecx_SDOread(context, slave, 0x60C2, 0x02, FALSE, &psize, &interp_exponent, EC_TIMEOUTSAFE) > 0)
      {
         double interp_time_s = interp_mantissa * pow(10.0, interp_exponent);
         printf("    60C2.01h Interpolation Period: %u × 10^%d = %.3f ms", 
                interp_mantissa, interp_exponent, interp_time_s * 1000.0);
         if (fabs(interp_time_s - cycle_s) < 0.0001)
            printf(" [✓ Matches Cycle Period]");
         printf("\n");
      }
   }
   printf("\n");
   /* === END EXPLORATORY READS === */

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
      if (cycles % 500 == 0) printf(" [wkc=%d sw=0x%04X]\n", wkc, tx->statusword);
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
      printf(" TIMEOUT [wkc=%d sw=0x%04X] \n", wkc, tx->statusword);
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
 *  Converts target and actual currents from raw values to physical Amps using KP scaling.
 *  \param fieldbus Fieldbus context (buffer and count updated)
 *  \param timestamp_s Absolute time in seconds
 *  \param target_current_A_sent Target current computed by master (before PDO encoding)
 *  \param target_current_A_received Target current read from 2010.01h via SDO (or 0.0 if not yet read)
 *  \param tx Pointer to received TxPDO data
 */
static void
log_sample(Fieldbus *fieldbus, double timestamp_s, double target_current_A_sent, double target_current_A_received, const tx_pdo_t *tx)
{
   if (fieldbus->sample_count < MAX_SAMPLES)
   {
      double actual_current_A = (tx->actual_current * fieldbus->kp_amps) / 32768.0;
      double current_demand_A = (tx->current_demand * fieldbus->kp_amps) / 32768.0;
      fieldbus->samples[fieldbus->sample_count].timestamp_s = timestamp_s;
      fieldbus->samples[fieldbus->sample_count].ai1_raw = tx->ai1_raw;
      fieldbus->samples[fieldbus->sample_count].ai2_raw = tx->ai2_raw;
      fieldbus->samples[fieldbus->sample_count].target_current_A_sent = target_current_A_sent;
      fieldbus->samples[fieldbus->sample_count].target_current_A_received = target_current_A_received;
      fieldbus->samples[fieldbus->sample_count].actual_current_A = actual_current_A;
      fieldbus->samples[fieldbus->sample_count].current_demand_A = current_demand_A;
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

/** \brief Read diagnostic objects (2002h, 2003h, 200Fh, 2021h) via SDO after cyclic loop exits
 *  Called only when fault_detected is true, after PDO exchange has stopped.
 *  Logs each non-zero diagnostic value as FAULT_DRIVE_STATUS_FLAG with encoded detail.
 *  \param fieldbus Fieldbus context
 *  \param timestamp_s Elapsed time when fault was detected
 */
static void
read_drive_status_sdo(Fieldbus *fieldbus, double timestamp_s)
{
   ecx_contextt *context = &fieldbus->context;
   int psize;
   uint16_t value16;
   int16_t value_signed16;
   int32_t value32;
   uint8_t sub;
   int32_t temp_raw;
   double temp_c;

   printf("Reading drive diagnostic objects via SDO...\n");

   /* Read 2002h.01h–.07h (Drive Status: current active fault flags) */
   printf("  2002h (Drive Status):\n");
   for (sub = 0x01; sub <= 0x07; sub++)
   {
      psize = sizeof(value16);
      if (ecx_SDOread(context, fieldbus->amc_slave_index, 0x2002, sub, FALSE, 
                      &psize, &value16, EC_TIMEOUTSAFE) > 0 && value16 != 0)
      {
         printf("    .%02Xh = 0x%04X\n", sub, value16);
         log_fault(fieldbus, timestamp_s, FAULT_DRIVE_STATUS_FLAG, 
                   (0x2000 | (sub << 8) | value16), RECOVERY_NONE);
      }
   }

   /* Read 2003h.01h–.07h (Drive Status History: events that ever occurred) */
   printf("  2003h (Status History):\n");
   for (sub = 0x01; sub <= 0x07; sub++)
   {
      psize = sizeof(value16);
      if (ecx_SDOread(context, fieldbus->amc_slave_index, 0x2003, sub, FALSE, 
                      &psize, &value16, EC_TIMEOUTSAFE) > 0 && value16 != 0)
      {
         printf("    .%02Xh = 0x%04X (history)\n", sub, value16);
         log_fault(fieldbus, timestamp_s, FAULT_DRIVE_STATUS_FLAG, 
                   (0x3000 | (sub << 8) | value16), RECOVERY_NONE);
      }
   }

   /* Read 200Fh.01h (DC Bus Voltage, Integer16, DV1 units) */
   printf("  200Fh (DC Bus Voltage):\n");
   psize = sizeof(value_signed16);
   if (ecx_SDOread(context, fieldbus->amc_slave_index, 0x200F, 0x01, FALSE, 
                   &psize, &value_signed16, EC_TIMEOUTSAFE) > 0)
   {
      printf("    .01h = %d (DV1 units)\n", value_signed16);
      if (value_signed16 < 200)  /* Abnormally low voltage indicator */
      {
         log_fault(fieldbus, timestamp_s, FAULT_DRIVE_STATUS_FLAG, 
                   (0xF001 | (value_signed16 & 0xFF)), RECOVERY_NONE);
      }
   }

   /* Read 2021h.01h (Motor Temperature, Integer32, formula: value/65536 = °C) */
   printf("  2021h (Motor Temperature):\n");
   psize = sizeof(value32);
   if (ecx_SDOread(context, fieldbus->amc_slave_index, 0x2021, 0x01, FALSE, 
                   &psize, &value32, EC_TIMEOUTSAFE) > 0)
   {
      temp_c = value32 / 65536.0;
      printf("    .01h = %d raw → %.2f °C\n", value32, temp_c);
      if (temp_c > 80.0)  /* Abnormally high temperature indicator */
      {
         log_fault(fieldbus, timestamp_s, FAULT_DRIVE_STATUS_FLAG, 
                   (0x2100 | ((int)(temp_c) & 0xFF)), RECOVERY_NONE);
      }
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
   int32_t target_current_raw;
   int wkc;
   int wkc_error_count = 0;
   int cycle_count = 0;
   uint16_t last_statusword = 0;
   uint16_t current_state;
   boolean fault_detected = FALSE;

   printf("\nStarting %.0f-second cyclic loop...\n", RUN_DURATION_S);
   
   /* Configure DC SYNC0 */
   ecx_dcsync0(context, fieldbus->amc_slave_index, TRUE, (uint32_t)cycle_ns, 0);

   clock_gettime(CLOCK_MONOTONIC, &next_cycle);

   while (elapsed_s < RUN_DURATION_S)
   {
      /* Compute target current (sine wave) */
      sine_phase = 2.0 * M_PI * SINE_FREQ_HZ * elapsed_s;
      target_current_A = SINE_AMPLITUDE_A * sin(sine_phase);
      
      /* Convert to raw Int32 (scale: 2^15 / KP) */
      target_current_raw = (int32_t)round((target_current_A * 32768.0) / fieldbus->kp_amps);
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
            fault_detected = TRUE;
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
         fault_detected = TRUE;
         break;
      }

      /* Log sample with target_current_A_received=0.0 (will be updated via SDO reads post-fault) */
      log_sample(fieldbus, elapsed_s, target_current_A, 0.0, tx);

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

   /* Read drive diagnostic objects via SDO after cyclic loop exits (PDO idle) */
   if (fault_detected)
   {
      read_drive_status_sdo(fieldbus, elapsed_s);
   }

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
      fprintf(fp, "time_s,target_A_sent,target_A_received,actual_current_A,current_demand_A,ai1_raw,ai2_raw\n");
      for (i = 0; i < fieldbus->sample_count; i++)
      {
         fprintf(fp, "%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u\n",
                 fieldbus->samples[i].timestamp_s,
                 fieldbus->samples[i].target_current_A_sent,
                 fieldbus->samples[i].target_current_A_received,
                 fieldbus->samples[i].actual_current_A,
                 fieldbus->samples[i].current_demand_A,
                 fieldbus->samples[i].ai1_raw,
                 fieldbus->samples[i].ai2_raw);
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
   struct sched_param param;

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

   /* Lock memory to prevent page-fault latency */
   if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
   {
      printf("WARNING: mlockall() failed (requires CAP_IPC_LOCK or root): %s\n", strerror(errno));
   }

   /* Enable SCHED_FIFO real-time scheduling at maximum priority */
   param.sched_priority = sched_get_priority_max(SCHED_FIFO);
   if (sched_setscheduler(0, SCHED_FIFO, &param) != 0)
   {
      printf("WARNING: sched_setscheduler(SCHED_FIFO) failed (requires root): %s\n", strerror(errno));
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
