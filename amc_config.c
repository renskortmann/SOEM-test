/** \file amc_config.c
 * \brief AMC servo drive SDO configuration during PreOp→SafeOp transition
 */

#include "main.h"

/** \brief Configure AMC servo drive during PreOp→SafeOp transition (PO2SOconfig hook)
 *  Sets Cyclic Synchronous Torque (CST) mode, interpolation period, PDO mappings, and reads peak current (KP).
 *  Called automatically by ecx_config_map_group() for any slave with PO2SOconfig assigned.
 *  \param context SOEM EtherCAT context
 *  \param slave Slave index (1-based)
 *  \return 1 on success, 0 on failure (not used in this implementation)
 */
int
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
   uint32 map_TxPDO[7];
   map_TxPDO[0] = (STATUS_WORD_INDEX << 16) | (0x00 << 8) | 0x10;
   map_TxPDO[1] = (ACTUAL_CURRENT_INDEX << 16) | (0x00 << 8) | 0x10;
   map_TxPDO[2] = (TARGET_CURRENT_INDEX << 16) | (0x00 << 8) | 0x10;
   map_TxPDO[3] = (AI_VALUE_INDEX << 16) | (AI1_VALUE_SUBINDEX << 8) | 0x10;
   map_TxPDO[4] = (AI_VALUE_INDEX << 16) | (AI2_VALUE_SUBINDEX << 8) | 0x10;
   map_TxPDO[5] = (CURRENT_VALUES_INDEX << 16) | (CURRENT_DEMAND_SUBINDEX << 8) | 0x10;
   map_TxPDO[6] = (POWER_BRIDGE_VALUES_INDEX << 16) | (DC_BUS_VOLTAGE_SUBINDEX << 8) | 0x10;

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

   /* Read DC Bus Over-Voltage limit (20D8.09h) for DC Bus Voltage scaling (K_OV) */
   uint16 kov_raw = 0;
   psize = sizeof(kov_raw);
   wkc_read = ecx_SDOread(context, slave, POWER_BOARD_INFORMATION_INDEX, DC_BUS_OVER_VOLTAGE_SUBINDEX, FALSE, &psize, &kov_raw, EC_TIMEOUTSAFE);
   fieldbus->kov_volts = kov_raw / PBV_SCALE;
   printf("  Read DC Bus Over-Voltage limit (KOV): raw=0x%04X -> %.1f V. psize = %d, wkc_read = %d\n", kov_raw, fieldbus->kov_volts, psize, wkc_read);

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
