/** \file logging.c
 * \brief Sample and fault logging: in-memory buffers and CSV export
 */

#include "voice_coil.h"

/** \brief Record one timestamped sample to the in-memory sample buffer
 *  Converts target and actual currents from raw values to physical Amps using KP scaling.
 *  \param fieldbus Fieldbus context (buffer and count updated)
 *  \param timestamp_s Absolute time in seconds
 *  \param tx Pointer to received TxPDO data
 */
void
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
void
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

/** \brief Write sample and fault buffers to timestamped CSV files in CSV_DIR
 *  Creates two files:
 *    - voice_coil_log_YYYYMMDD_HHMMSS.csv: samples (time_s, ai1_raw, ai2_raw, actual_current_A)
 *    - voice_coil_faults_YYYYMMDD_HHMMSS.csv: fault events (timestamp, type, detail, action)
 *  \param fieldbus Fieldbus context with populated buffers
 */
void
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
