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

#define SERVO_DRIVE_TYPE    0x0192

#define IO_MAP_SIZE        4096

#define DC1_SCALE          8192.0  // Scaling factor 2^13 for CiA402 DC1 current units (16-bit signed)
#define DC2_SCALE          32768.0 // Scaling factor 2^15 for CiA402 DC2 position/velocity units (32-bit signed)

// PDO object indices for CiA402 current control
#define ACTUAL_CURRENT_INDEX           0x6077    // Actual current (DC1) in 16-bit signed integer format
#define TARGET_CURRENT_INDEX           0x6071    // Target current (DC1) in 16-bit signed integer format
#define TARGET_POSITION_INDEX          0x607A    // Target position (DC2) in 32-bit signed integer format
#define TARGET_VELOCITY_INDEX          0x60FF    // Target velocity (DC2) in 32-bit signed integer format
#define CURRENT_VALUES_INDEX           0x2010    // Current values (DC1) in 16-bit signed integer format
#define CONTROL_WORD_INDEX             0x6040    // ControlWord (16-bit) for CiA402 state machine
#define STATUS_WORD_INDEX              0x6041    // StatusWord (16-bit) for CiA402 state machine
#define MODE_OF_OPERATION_INDEX        0x6060    // Mode (16-bit) for CiA402 mode of operation
#define INTERPOLATION_TIME_INDEX       0x60C2    // Interpolation Time Period (32-bit) for CST mode
#define AI_RAW_INDEX                   0x2022    // Analog Input raw ADC value (16-bit unsigned)
#define AI_VALUE_INDEX                 0x201A    // Analog Input scaled value (16-bit signed)
#define POWER_BOARD_INFORMATION_INDEX  0x20D8 // Power Board Information (32-bit unsigned) for reading KP
#define COMM_CHANNEL_ERROR_ACTION_INDEX 0x2065 // Event Action for Comm Channel Error (16-bit unsigned)
#define SYNC_MANAGER_COMM_TYPE_INDEX   0x1C00 // Sync Manager Communication Type (8-bit unsigned)
#define DEVICE_TYPE_INDEX              0x1000 // Device Type (32-bit unsigned) for identifying CiA402 drive
#define IDENTITY_OBJECT_INDEX          0x1018 // Identity Object (32-bit unsigned) for vendor/product info
#define WATCHDOG_TIMEOUT_INDEX         0x2065 // Watchdog Timeout (16-bit unsigned) for drive watchdog configuration
#define WATCHDOG_ACTION_INDEX          0x2065 // Watchdog Action (16-bit unsigned) for drive watchdog configuration
#define SYNC_MANAGER_CHANNELS_INDEX    0x1C00 // Sync Manager Channels (8-bit unsigned) for drive synchronization
#define TxPDO_INDEX                    0x1A00 // Transmit PDO mapping index for CiA402 drive
#define RxPDO_INDEX                    0x1600 // Receive PDO mapping index for CiA402 drive

#define MANTISSA_SUBINDEX                    0x01 // Sub-index for mantissa in interpolation time period object
#define EXPONENT_SUBINDEX                    0x02 // Sub-index for exponent in interpolation time period object
#define COMM_CHANNEL_ERROR_ACTION_SUBINDEX   0x21 // Sub-index for comm channel error action object
#define MAX_PEAK_CURRENT_SUBINDEX            0x0C // Sub-index for maximum peak current in power board information object
#define VENDOR_ID_SUBINDEX                   0x01 // Sub-index for vendor ID in identity object
#define PRODUCT_CODE_SUBINDEX                0x02 // Sub-index for product code in identity object
#define REVISION_NUMBER_SUBINDEX             0x03 // Sub-index for revision number in identity object
#define SERIAL_NUMBER_SUBINDEX               0x04 // Sub-index for serial number in identity object
#define AI1_RAW_SUBINDEX                     0x01 // Sub-index for Analog Input 1 raw value
#define AI2_RAW_SUBINDEX                     0x02 // Sub-index for Analog Input 2 raw value
#define CURRENT_DEMAND_SUBINDEX              0x02 // Sub-index for Current Demand in the current values object
#define AI1_VALUE_SUBINDEX                   0x01 // Sub-index for Analog Input 1 scaled value
#define AI2_VALUE_SUBINDEX                   0x02 // Sub-index for Analog Input 2 scaled value

// static const uint16_t TxPDOcontents[] = {STATUS_WORD_INDEX, ACTUAL_CURRENT_INDEX, TARGET_CURRENT_INDEX, AI_RAW_INDEX, AI_RAW_INDEX, 
//    CURRENT_DEMAND_INDEX, AI_VALUE_INDEX, AI_VALUE_INDEX};
//static const uint16_t RxPDOcontents[] = {CONTROL_WORD_INDEX, TARGET_POSITION_INDEX, TARGET_VELOCITY_INDEX, TARGET_CURRENT_INDEX};

#define TxPDO_CONTENTS_SIZE (sizeof(TxPDOcontents) / sizeof(TxPDOcontents[0]))
//#define RxPDO_CONTENTS_SIZE (sizeof(RxPDOcontents) / sizeof(RxPDOcontents[0]))

/** \brief Master-to-slave process data: CiA402 ControlWord and CST target current command */
typedef struct OSAL_PACKED
{
   uint16_t controlword;     /**< CiA402 control bits (Shutdown/Switch On/Enable Operation) */
   int32_t target_position;  /**< Desired motor position (encoder units) */
   int32_t target_velocity;  /**< Desired motor velocity (encoder units/s) */
   int16_t target_current;   /**< Desired motor current, scaled by KP (DC1 units) */
} rx_pdo_t;

/** \brief Slave-to-master process data: CiA402 StatusWord, actual current, target current, and analog sensor inputs */
typedef struct OSAL_PACKED
{
   uint16_t statusword;      /**< CiA402 status bits (state machine state + fault bits) */
   int16_t actual_current;   /**< Measured motor current from drive, scaled by KP (DC1 units) */
   int16_t target_current;   /**< Desired motor current, scaled by KP (DC1 units) */
   uint16_t ai1_raw;         /**< Analog Input 1 raw ADC value (0-65535) */
   uint16_t ai2_raw;         /**< Analog Input 2 raw ADC value (0-65535) */
   int16_t ai1_value;        /**< Analog Input 1 scaled value */
   int16_t ai2_value;        /**< Analog Input 2 scaled value */
   int16_t demand_current;    /**< Current Demand from drive (DC1 units) */
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
   int16_t ai1_value;        /**< Analog input 1 scaled value at this timestamp */
   int16_t ai2_value;        /**< Analog input 2 scaled value at this timestamp */
   double actual_current_A;  /**< Actual motor current in physical Amps at this timestamp */
   double target_current_A;  /**< Drive's reported target current in Amps at this timestamp (6071h, DC1) */
   double demand_current_A;  /**< Drive's reported current demand in Amps at this timestamp (2010h.02, DC1) */
} sample_log_entry_t;

/** \brief Master state container: EtherCAT protocol context, drive parameters, and sample/fault buffers */
typedef struct
{
   ecx_contextt context;           /**< SOEM EtherCAT context with slave list and I/O mapping */
   char *iface;                    /**< Network interface name (e.g., "eth0") */
   uint8 group;                    /**< I/O group index (0 for single-group setup) */
   int roundtrip_time;             /**< Last measured PDO roundtrip time in microseconds */
   uint8 map[IO_MAP_SIZE];                /**< I/O mapping buffer for ecx_config_map_group() */
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
 *  \return 1 on success, 0 on failure (not used in this implementation)
 */
static int
amc_slave_config(ecx_contextt *context, uint16 slave)
{
   Fieldbus *fieldbus = (Fieldbus *)context->userdata;
   int wkc_write = 0;
   int wkc_read = 0;
   int psize;

   printf("Configuring AMC slave %d via SDO...\n", slave);

   /* Set mode of operation (6060h) to CST (0x0A) */
   uint8 mode = CST_MODE;
   psize = sizeof(mode);
   wkc_write = ecx_SDOwrite(context, slave, MODE_OF_OPERATION_INDEX, 0x00, FALSE, psize, &mode, EC_TIMEOUTSAFE);
   printf("  Wrote mode of operation (6060h) = 0x%02X. psize = %d. wkc_write = %d.\n", mode, psize, wkc_write);
   mode = 0;
   wkc_read = ecx_SDOread(context, slave, MODE_OF_OPERATION_INDEX, 0x00, FALSE, &psize, &mode, EC_TIMEOUTSAFE);
   printf("  Read mode of operation (6060h) = 0x%02X. psize = %d. wkc_read = %d.\n", mode, psize, wkc_read);
   int8 mode_display = 0;
   psize = sizeof(mode_display);
   wkc_read = ecx_SDOread(context, slave, 0x6061, 0x00, FALSE, &psize, &mode_display, EC_TIMEOUTSAFE);
   printf("  Read mode of operation display (6061h) = 0x%02X. psize = %d. wkc_read = %d.\n", mode_display, psize, wkc_read);

   /* Compute and write interpolation time period (60C2h) */
   double cycle_s = CYCLE_TIME_MS / 1000.0;
   uint8 mantissa;
   int8 exponent;
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
   else if (cycle_s >= 0.001)
   {
      mantissa = (uint8)(cycle_s / 0.001);
      exponent = -3;
   }
   else
   {
      mantissa = (uint8)(cycle_s / 0.0001);
      exponent = -4;
   }

   // Write mantissa and exponent to 60C2.01h and 60C2.02h, respectively
   psize = sizeof(mantissa);
   wkc_write = ecx_SDOwrite(context, slave, INTERPOLATION_TIME_INDEX, MANTISSA_SUBINDEX, FALSE, psize, &mantissa, EC_TIMEOUTSAFE);
   printf("  Wrote interpolation period mantissa (60C2.01h) = 0x%02X. psize = %d. wkc_write = %d.\n", mantissa, psize, wkc_write);
   mantissa = 0;
   wkc_read = ecx_SDOread(context, slave, INTERPOLATION_TIME_INDEX, MANTISSA_SUBINDEX, FALSE, &psize, &mantissa, EC_TIMEOUTSAFE);
   printf("  Read interpolation period mantissa (60C2.01h) = 0x%02X. psize = %d, wkc_read = %d\n", mantissa, psize, wkc_read);
   
   psize = sizeof(exponent);
   wkc_write = ecx_SDOwrite(context, slave, INTERPOLATION_TIME_INDEX, EXPONENT_SUBINDEX, FALSE, psize, &exponent, EC_TIMEOUTSAFE);
   printf("  Wrote interpolation period exponent (60C2.02h) = %d. psize = %d. wkc_write = %d.\n", exponent, psize, wkc_write);
   exponent = 0;
   wkc_read = ecx_SDOread(context, slave, INTERPOLATION_TIME_INDEX, EXPONENT_SUBINDEX, FALSE, &psize, &exponent, EC_TIMEOUTSAFE);
   printf("  Read interpolation period exponent (60C2.02h) = %d. psize = %d, wkc_read = %d\n", exponent, psize, wkc_read);

   /* Configure RxPDO mapping (1600h): ControlWord + Target Current */
   // Clear existing mapping by writing 0 to 1600.00h
   wkc_write = ecx_SDOwrite(context, slave, RxPDO_INDEX, 0x00, FALSE, sizeof(uint8), &(uint8){0}, EC_TIMEOUTSAFE);
   
   // Define new mapping entries for RxPDO (1600h)
   printf("  Creating map_RxPDO array.\n");
   uint32 map_RxPDO[4];
   map_RxPDO[0] = (0x6040 << 16) | (0x00 << 8) | 0x10;
   map_RxPDO[1] = (0x607A << 16) | (0x00 << 8) | 0x20;
   map_RxPDO[2] = (0x60FF << 16) | (0x00 << 8) | 0x20;
   map_RxPDO[3] = (0x6071 << 16) | (0x00 << 8) | 0x10;
   
   int8 number_of_mapped_Rx_objects = sizeof(map_RxPDO) / sizeof(map_RxPDO[0]);
   // Write mapping entries objects to 1600.01h through 1600.<RxPDO_CONTENTS_SIZE>h
   for (int i = 0; i < number_of_mapped_Rx_objects; i++){
      psize = sizeof(map_RxPDO[i]);
      wkc_write = ecx_SDOwrite(context, slave, RxPDO_INDEX, 0x01 + i, FALSE, psize, map_RxPDO + i, EC_TIMEOUTSAFE);
      printf("  Wrote RxPDO mapping entry %d (1600.%02Xh) = %08Xh. psize = %d, wkc_write = %d\n", 
         i, 0x01 + i, map_RxPDO[i], psize, wkc_write);
      if (wkc_write <= 0)
         printf("%s\n", ecx_elist2string(context));
      wkc_read = ecx_SDOread(context, slave, RxPDO_INDEX, 0x01 + i, FALSE, &psize, &map_RxPDO[i], EC_TIMEOUTSAFE);
      printf("  Read RxPDO mapping entry %d (1600.%02Xh) = %08Xh. psize = %d, wkc_read = %d\n", 
         i, 0x01 + i, map_RxPDO[i], psize, wkc_read);
   }
   // Set number of mapped objects to RxPDO_CONTENTS_SIZE by writing to 1600.00h
   wkc_write = ecx_SDOwrite(context, slave, RxPDO_INDEX, 0x00, FALSE, sizeof(uint8), &number_of_mapped_Rx_objects, EC_TIMEOUTSAFE);
   
   // Read back RxPDO mapping (1600.00h) to verify number of mapped objects
   uint8 count = 0;
   psize = sizeof(count);
   wkc_read = ecx_SDOread(context, slave, RxPDO_INDEX, 0x00, FALSE, &psize, &count, EC_TIMEOUTSAFE);
   // printf("  Read RxPDO (1600.00h) mapping with %d objects (expected %d). psize = %d, wkc_read = %d\n", count, (int)RxPDO_CONTENTS_SIZE, psize, wkc_read);
   printf("  Read RxPDO mapping (1600.00h) with %d objects. psize = %d, wkc_read = %d\n", count, psize, wkc_read);

   for (int i = 0; i < count; i++) {
      uint32 entry = 0;
      psize = sizeof(entry);
      wkc_read = ecx_SDOread(context, slave, RxPDO_INDEX, 0x01 + i, FALSE, &psize, &entry, EC_TIMEOUTSAFE);
      printf("  1600.%02Xh = %08Xh  -> index=%04Xh sub=%02Xh len=%u bits. psize = %d, wkc_read = %d\n",
            0x01 + i, entry, entry >> 16, (entry >> 8) & 0xFF, entry & 0xFF, psize, wkc_read);
   }

   /* Configure TxPDO mapping (1A00h) */
   // Clear existing mapping by writing 0 to 1A00.00h
   wkc_write = ecx_SDOwrite(context, slave, TxPDO_INDEX, 0x00, FALSE, sizeof(uint8), &(uint8){0}, EC_TIMEOUTSAFE);
   
   // Define new mapping entries for TxPDO (1A00h)
   printf("  Creating map_TxPDO array\n");
   uint32 map_TxPDO[8];
   map_TxPDO[0] = (STATUS_WORD_INDEX << 16) | (0x00 << 8) | 0x10;
   map_TxPDO[1] = (ACTUAL_CURRENT_INDEX << 16) | (0x00 << 8) | 0x10;
   map_TxPDO[2] = (TARGET_CURRENT_INDEX << 16) | (0x00 << 8) | 0x10;
   map_TxPDO[3] = (AI_RAW_INDEX << 16) | (AI1_RAW_SUBINDEX << 8) | 0x10;
   map_TxPDO[4] = (AI_RAW_INDEX << 16) | (AI2_RAW_SUBINDEX << 8) | 0x10;
   map_TxPDO[5] = (AI_VALUE_INDEX << 16) | (AI1_VALUE_SUBINDEX << 8) | 0x10;
   map_TxPDO[6] = (AI_VALUE_INDEX << 16) | (AI2_VALUE_SUBINDEX << 8) | 0x10;
   map_TxPDO[7] = (CURRENT_VALUES_INDEX << 16) | (CURRENT_DEMAND_SUBINDEX << 8) | 0x10;
   
   int number_of_mapped_objects = sizeof(map_TxPDO) / sizeof(map_TxPDO[0]);
   // Write mapping entries to 1A00.01h (1st application object) through 1A00.<number_of_mapped_objects>h (last application object)
   for (int i = 0; i < number_of_mapped_objects; i++) {
      wkc_write = ecx_SDOwrite(context, slave, TxPDO_INDEX, 0x01 + i, FALSE, (int)sizeof(uint32), map_TxPDO + i, EC_TIMEOUTSAFE);
      printf("  Wrote TxPDO mapping entry %d (1600.%02Xh) = %08Xh. psize = %d, wkc_write = %d\n", 
         i+1, 0x01 + i, map_TxPDO[i], psize, wkc_write);
      if (wkc_write <= 0)
         printf("%s\n", ecx_elist2string(context));
   }
   // Set number of mapped objects to TxPDO_CONTENTS_SIZE by writing to 1A00.00h (number of mapped objects)
   wkc_write = ecx_SDOwrite(context, slave, TxPDO_INDEX, 0x00, FALSE, 1, &(uint8){number_of_mapped_objects}, EC_TIMEOUTSAFE);

   // Read back TxPDO mapping[0] (1A00.00h) to verify number of mapped objects
   count = 0;
   psize = sizeof(count);
   wkc_read = ecx_SDOread(context, slave, TxPDO_INDEX, 0x00, FALSE, &psize, &count, EC_TIMEOUTSAFE);
   printf("  Read TxPDO (1A00.00h) mapping with %d objects. psize = %d, wkc_read = %d\n", count, psize, wkc_read);

   for (int i = 0; i < count; i++) {
      uint32 entry = 0;
      psize = sizeof(entry);
      wkc_read = ecx_SDOread(context, slave, TxPDO_INDEX, 0x01 + i, FALSE, &psize, &entry, EC_TIMEOUTSAFE);
      printf("  1A00.%02Xh = %08Xh  -> index=%04Xh sub=%02Xh len=%u bits. psize = %d, wkc_read = %d\n",
            0x01 + i, entry, entry >> 16, (entry >> 8) & 0xFF, entry & 0xFF, psize, wkc_read);
   }

   /* Read Maximum Peak Current (20D8.0Ch) for current scaling */
   uint32 kp_raw = 0;
   psize = sizeof(kp_raw);
   wkc_read = ecx_SDOread(context, slave, POWER_BOARD_INFORMATION_INDEX, MAX_PEAK_CURRENT_SUBINDEX, FALSE, &psize, &kp_raw, EC_TIMEOUTSAFE);
   fieldbus->kp_amps = kp_raw / 10.0;
   printf("  Read Maximum Peak Current (KP): raw=0x%08X -> %.1f A. psize = %d, wkc_read = %d\n", kp_raw, fieldbus->kp_amps, psize, wkc_read);

   /* === EXPLORATORY READS: Watchdog & Synchronization Diagnostics === */
   printf("\n  Diagnostic Reads (Synchronization & Watchdog Configuration):\n");

   /* Read Device Type (1000h) */
   uint32_t device_type = 0;
   psize = sizeof(device_type);
   if ((wkc_read = ecx_SDOread(context, slave, DEVICE_TYPE_INDEX, 0x00, FALSE, &psize, &device_type, EC_TIMEOUTSAFE)) > 0)
   {
      printf("    Read Device Type (1000h): 0x%08X. psize = %d, wkc_read = %d", device_type, psize, wkc_read);
      if ((device_type & 0xFFFF) == SERVO_DRIVE_TYPE)
         printf(" [CiA402 Servo Drive]");
      printf("\n");
   }

   /* Read Identity Object (1018h sub-indices) */
   uint32_t vendor_id = 0, product_code = 0, revision = 0, serial = 0;
   psize = sizeof(vendor_id);
   if ((wkc_read = ecx_SDOread(context, slave, IDENTITY_OBJECT_INDEX, VENDOR_ID_SUBINDEX, FALSE, &psize, &vendor_id, EC_TIMEOUTSAFE)) > 0)
      printf("    Read Vendor ID (1018.01h): 0x%02X. psize = %d, wkc_read = %d\n", vendor_id, psize, wkc_read);
   
   psize = sizeof(product_code);
   if ((wkc_read = ecx_SDOread(context, slave, IDENTITY_OBJECT_INDEX, PRODUCT_CODE_SUBINDEX, FALSE, &psize, &product_code, EC_TIMEOUTSAFE)) > 0)
      printf("    Read Product Code (1018.02h): 0x%08X. psize = %d, wkc_read = %d\n", product_code, psize, wkc_read);
   
   psize = sizeof(revision);
   if ((wkc_read = ecx_SDOread(context, slave, IDENTITY_OBJECT_INDEX, REVISION_NUMBER_SUBINDEX, FALSE, &psize, &revision, EC_TIMEOUTSAFE)) > 0)
      printf("    Read Revision (1018.03h): 0x%08X. psize = %d, wkc_read = %d\n", revision, psize, wkc_read);
   
   psize = sizeof(serial);
   if ((wkc_read = ecx_SDOread(context, slave, IDENTITY_OBJECT_INDEX, SERIAL_NUMBER_SUBINDEX, FALSE, &psize, &serial, EC_TIMEOUTSAFE)) > 0)
      printf("    Read Serial Number (1018.04h): 0x%08X. psize = %d, wkc_read = %d\n", serial, psize, wkc_read);

   /* Read Sync Manager Communication Type (1C00h) */
   uint8_t sm_channels = 0;
   psize = sizeof(sm_channels);
   if ((wkc_read = ecx_SDOread(context, slave, SYNC_MANAGER_COMM_TYPE_INDEX, 0x00, FALSE, &psize, &sm_channels, EC_TIMEOUTSAFE)) > 0)
      printf("    Read Sync Manager Channels (1C00.00h): %d. psize = %d, wkc_read = %d\n", sm_channels, psize, wkc_read);

   /* Read Event Action for Comm Channel Error (2065.21h) - watchdog behavior */
   uint16_t comm_error_action = 0;
   psize = sizeof(comm_error_action);
   if ((wkc_read = ecx_SDOread(context, slave, COMM_CHANNEL_ERROR_ACTION_INDEX, COMM_CHANNEL_ERROR_ACTION_SUBINDEX, FALSE, &psize, &comm_error_action, EC_TIMEOUTSAFE)) > 0)
   {
      printf("    Read Comm Channel Error Action (2065.21h): 0x%04X. ", comm_error_action);
      if (comm_error_action == 0)
         printf(" [No Action]");
      else if (comm_error_action == 1)
         printf(" [Fault]");
      else if (comm_error_action == 2)
         printf(" [Shutdown]");
      printf(" psize = %d, wkc_read = %d\n", psize, wkc_read);
   }

   /* Verify Interpolation Time Period (60C2h) is set correctly */
   uint8_t interp_mantissa = 0;
   int8_t interp_exponent = 0;
   int psize_mantissa = sizeof(interp_mantissa);
   if ((wkc_read = ecx_SDOread(context, slave, INTERPOLATION_TIME_INDEX, MANTISSA_SUBINDEX, FALSE, &psize_mantissa, &interp_mantissa, EC_TIMEOUTSAFE)) > 0)
   {
      int psize_exponent = sizeof(interp_exponent);
      if ((wkc_read = ecx_SDOread(context, slave, INTERPOLATION_TIME_INDEX, EXPONENT_SUBINDEX, FALSE, &psize_exponent, &interp_exponent, EC_TIMEOUTSAFE)) > 0)
      {
         double interp_time_s = interp_mantissa * pow(10.0, interp_exponent);
         printf("    Read Interpolation Period (60C2.01h): %u × 10^%d = %.3f ms", 
                interp_mantissa, interp_exponent, interp_time_s * 1000.0);
         if (fabs(interp_time_s - cycle_s) < 0.0001)
            printf(" [✓ Matches Cycle Period]");
         printf(" psize_mantissa = %d, psize_exponent = %d, wkc_read = %d\n", psize_mantissa, psize_exponent, wkc_read);
      }
   }
   printf("\n");
   /* === END EXPLORATORY READS === */

   fieldbus->amc_slave_index = slave;
   return 1;
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
   printf("mapped %dObytes + %dIbytes.\n", grp->Obytes, grp->Ibytes);

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
 *  \param tx Pointer to received TxPDO data
 */
static void
log_sample(Fieldbus *fieldbus, double timestamp_s, const tx_pdo_t *tx)
{
   if (fieldbus->sample_count < MAX_SAMPLES)
   {
      double actual_current_A = (tx->actual_current * fieldbus->kp_amps) / DC1_SCALE;
      double target_current_A = (tx->target_current * fieldbus->kp_amps) / DC2_SCALE;
      double demand_current_A = (tx->demand_current * fieldbus->kp_amps) / DC1_SCALE;
      fieldbus->samples[fieldbus->sample_count].timestamp_s = timestamp_s;
      fieldbus->samples[fieldbus->sample_count].ai1_raw = tx->ai1_raw;
      fieldbus->samples[fieldbus->sample_count].ai2_raw = tx->ai2_raw;
      fieldbus->samples[fieldbus->sample_count].actual_current_A = actual_current_A;
      fieldbus->samples[fieldbus->sample_count].demand_current_A = demand_current_A;
      fieldbus->samples[fieldbus->sample_count].target_current_A = target_current_A;
      fieldbus->samples[fieldbus->sample_count].ai1_value = tx->ai1_value;
      fieldbus->samples[fieldbus->sample_count].ai2_value = tx->ai2_value;
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

static void
lookup_fault_name(uint16 drive_status_flag, uint8 subindex)
{
   /* This function can be expanded to decode specific drive status flags and subindices into human-readable fault names */
   printf("  Subindex: 0x%02X, Drive Status Flag: 0x%04X\n", subindex, drive_status_flag);

   // convert drive status to bitfield and print which bits are set
   for (int bit = 0; bit < 16; bit++)
   {
      if (drive_status_flag & (1 << bit))
      {
         switch (subindex)
         {
            case 0x01:  /* Drive Bridge Status */
               printf("    Drive Bridge Status: Bit %d set: ", bit);
               switch (bit)
               {
                  case 0: printf("Bridge Enabled\n"); break;
                  case 1: printf("Dynamix Brake Enabled\n"); break;
                  case 2: printf("Stop Enabled\n"); break;
                  case 3: printf("Positive Stop Enabled\n"); break;
                  case 4: printf("Negative Stop Enabled\n"); break;
                  case 5: printf("Positive Torque Inhibit Active\n"); break;
                  case 6: printf("Negative Torque Inhibit Active\n"); break;
                  case 7: printf("External Brake Active\n"); break;
                  default: printf("Reserved\n"); break;  
               }
               break;
            case 0x02:  /* Drive Protection Status */
               printf("    Drive Protection Status: Bit %d set: ", bit);
               switch (bit)
               {
                  case 0: printf("Drive Reset\n"); break;
                  case 1: printf("Drive Internal Error\n"); break;
                  case 2: printf("Short Circuit\n"); break;
                  case 3: printf("Over Current\n"); break;
                  case 4: printf("Under Voltage\n"); break;
                  case 5: printf("Over Voltage\n"); break;
                  case 6: printf("Drive Over Temperature\n"); break;
                  default: printf("Reserved\n"); break;  
               }
               break;
            case 0x03:  /* System Protection Status */
               printf("    System Protection Status: Bit %d set: ", bit);
               switch (bit)
               {
                  case 0: printf("Parameter Restore Error\n"); break;
                  case 1: printf("Parameter Store Error\n"); break;
                  case 2: printf("Invalid Hall State\n"); break;
                  case 3: printf("Phase sync Error\n"); break;
                  case 4: printf("Motor over temperature\n"); break;
                  case 5: printf("Phase Detection Fault\n"); break;
                  case 6: printf("Feedback Sensor Error\n"); break;
                  case 7: printf("Motor over Speed\n"); break;
                  case 8: printf("Max measured position\n"); break;
                  case 9: printf("Min measured position\n"); break;
                  case 10: printf("Comm error (node guarding)\n"); break;
                  case 11: printf("PWM input broken wire\n"); break;
                  case 12: printf("Motion Engine Error\n"); break;
                  case 13: printf("Motion Engine Abort\n"); break;
                  default: printf("Reserved\n"); break;  
               }
               break;
            case 0x04:  /* Drive system status 1 */
               printf("   Drive system status 1: Bit %d set: ", bit);
               switch (bit)
               {
                  case 0: printf("Log Entry missed\n"); break;
                  case 1: printf("Software Disable\n"); break;
                  case 2: printf("User Disable\n"); break;
                  case 3: printf("User Positive Inhibit\n"); break;
                  case 4: printf("User Negative Inhibit\n"); break;
                  case 5: printf("Current Limiting\n"); break;
                  case 6: printf("Continuous Current Foldback\n"); break;
                  case 7: printf("Current loop saturated\n"); break;
                  case 8: printf("User under voltage\n"); break;
                  case 9: printf("User over voltage\n"); break;
                  case 10: printf("Non sinusoidal commutation\n"); break;
                  case 11: printf("Phase detect Active\n"); break;
                  case 12: printf("Motion Engine Active\n"); break;
                  case 13: printf("User Auxiliary Disabled\n"); break;
                  case 14: printf("Shunt Regulator\n"); break;
                  case 15: printf("Phase detect done\n"); break;
                  default: printf("Reserved\n"); break;  
               }
               break;
            case 0x05:  /* Drive system status 2 */
               printf("   Drive system status 2: Bit %d set: ", bit);
               switch (bit)
               {
                  case 0: printf("Zero Velocity\n"); break;
                  case 1: printf("At Command\n"); break;
                  case 2: printf("Velocity Following Error\n"); break;
                  case 3: printf("Positive target velocity limit\n"); break;
                  case 4: printf("Negative target velocity limit\n"); break;
                  case 5: printf("Command Limiter Active\n"); break;
                  case 6: printf("In Home Position\n"); break;
                  case 7: printf("Position Following Error\n"); break;
                  case 8: printf("Max Target Position Limit\n"); break;
                  case 9: printf("Min target position limit\n"); break;
                  case 10: printf("Set Position\n"); break;
                  case 11: printf("Reserved\n"); break;
                  case 12: printf("Homing Active\n"); break;
                  case 13: printf("Safe Torque Off Status\n"); break;
                  case 14: printf("Homing Complete\n"); break;
                  case 15: printf("Zero Position Error\n"); break;
                  default: printf("Reserved\n"); break;
               }
               break;
            case 0x06:  /* Drive system status 3 */
               printf("   Drive system status 3: Bit %d set: ", bit);
               switch (bit)
               {
                  case 0: printf("Reserved\n"); break;
                  case 1: printf("Reserved\n"); break;
                  case 2: printf("Reserved\n"); break;
                  case 3: printf("Reserved\n"); break;
                  case 4: printf("Reserved\n"); break;
                  case 5: printf("Reserved\n"); break;
                  case 6: printf("Commanded Stop\n"); break;
                  case 7: printf("User Stop\n"); break;
                  case 8: printf("Capture 1 active\n"); break;
                  case 9: printf("Capture 2 active\n"); break;
                  case 10: printf("Capture 3 active\n"); break;
                  case 11: printf("Commanded Positive Limit\n"); break;
                  case 12: printf("Commanded Negative Limit\n"); break;
                  default: printf("Reserved\n"); break;  
               }
               break;
            case 0x07:  /* Active Configuration Status */
               printf("   Active Configuration Status: Bit %d set: ", bit);
               switch (bit)
               {
                  case 0: printf("Absolute Position valid\n"); break;
                  case 1: printf("Positive Stop Active\n"); break;
                  case 2: printf("Negative Stop Active\n"); break;
                  default: printf("Reserved\n"); break;  
               }
               break;
         }
      }
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
      ecx_SDOread(context, fieldbus->amc_slave_index, 0x2002, sub, FALSE, &psize, &value16, EC_TIMEOUTSAFE);
      // printf("    .%02Xh = 0x%04X\n", sub, value16);
      if (value16 != 0)
         lookup_fault_name(value16, sub);
      // log_fault(fieldbus, timestamp_s, FAULT_DRIVE_STATUS_FLAG, 
      //           (0x2000 | (sub << 8) | value16), RECOVERY_NONE);
   }

   /* Read 2003h.01h–.07h (Drive Status History: events that ever occurred) */
   printf("  2003h (Status History):\n");
   for (sub = 0x01; sub <= 0x07; sub++)
   {
      psize = sizeof(value16);
      ecx_SDOread(context, fieldbus->amc_slave_index, 0x2003, sub, FALSE, &psize, &value16, EC_TIMEOUTSAFE);
      printf("    .%02Xh = 0x%04X (history)\n", sub, value16);
      lookup_fault_name(value16, sub);
      // log_fault(fieldbus, timestamp_s, FAULT_DRIVE_STATUS_FLAG, 
      //           (0x3000 | (sub << 8) | value16), RECOVERY_NONE);
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

   printf("\nStarting %.0f-second cyclic loop... expected WKC: %d\n", RUN_DURATION_S, expected_wkc);
   
   /* Configure DC SYNC0 */
   ecx_dcsync0(context, fieldbus->amc_slave_index, TRUE, (uint32_t)cycle_ns, 0);

   clock_gettime(CLOCK_MONOTONIC, &next_cycle);

   while (elapsed_s < RUN_DURATION_S)
   {
      /* Compute target current (sine wave) */
      sine_phase = 2.0 * M_PI * SINE_FREQ_HZ * elapsed_s;
      target_current_A = SINE_AMPLITUDE_A * sin(sine_phase);
      
      /* Convert to raw Int32 (scale: 2^15 / KP) */
      target_current_raw = (int32_t)round((target_current_A * DC2_SCALE) / fieldbus->kp_amps);
      if (target_current_raw > DC2_SCALE) target_current_raw = DC2_SCALE;  /* Saturate to max DC2 */
      if (target_current_raw < -DC2_SCALE) target_current_raw = -DC2_SCALE;  /* Saturate to min -DC2 */
      
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
      fprintf(fp, "time_s,actual_current_A,target_current_A,demand_current_A,ai1_raw,ai2_raw, ai1_value, ai2_value\n");
      for (i = 0; i < fieldbus->sample_count; i++)
      {
         fprintf(fp, "%.6f,%.6f,%.6f,%.6f,%u,%u,%d,%d\n",
                 fieldbus->samples[i].timestamp_s,
                 fieldbus->samples[i].actual_current_A,
                 fieldbus->samples[i].target_current_A,
                 fieldbus->samples[i].demand_current_A,
                 fieldbus->samples[i].ai1_raw,
                 fieldbus->samples[i].ai2_raw,
                 fieldbus->samples[i].ai1_value,
                 fieldbus->samples[i].ai2_value);
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
         read_drive_status_sdo(&fieldbus, 0.0);
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
