/** \file logging.c
 * \brief Sample and fault logging: in-memory buffers and CSV export
 */

#include "main.h"

/** \brief Record one timestamped sample to the in-memory sample buffer
 *  Converts currents from raw values to physical Amps using KP scaling, and the analog input
 *  values (201Ah) from raw DAI units to physical Volts using DAI_SCALE (2^14/20).
 *  \param fieldbus Fieldbus context (buffer and count updated)
 *  \param timestamp_s Absolute time in seconds
 *  \param tx Pointer to received TxPDO data
 *  \param cycle_jitter_us Signed offset between actual and scheduled cycle time (positive = late)
 *  \param pdo_exchange_us Time spent in send + receive processdata this cycle (frame round-trip)
 */
void
log_sample(Fieldbus *fieldbus, double timestamp_s, const tx_pdo_t *tx,
           double cycle_jitter_us, double pdo_exchange_us)
{
   if (fieldbus->sample_count < MAX_SAMPLES)
   {
      double actual_current_A = (tx->actual_current * fieldbus->kp_amps) / DC1_SCALE;
      double target_current_A = (tx->target_current * fieldbus->kp_amps) / DC2_SCALE;
      double demand_current_A = (tx->demand_current * fieldbus->kp_amps) / DC1_SCALE;
      double ai1_value_V = tx->ai1_value / DAI_SCALE;
      double ai2_value_V = tx->ai2_value / DAI_SCALE;
      fieldbus->samples[fieldbus->sample_count].timestamp_s = timestamp_s;
      fieldbus->samples[fieldbus->sample_count].ai1_raw = tx->ai1_raw;
      fieldbus->samples[fieldbus->sample_count].ai2_raw = tx->ai2_raw;
      fieldbus->samples[fieldbus->sample_count].actual_current_A = actual_current_A;
      fieldbus->samples[fieldbus->sample_count].demand_current_A = demand_current_A;
      fieldbus->samples[fieldbus->sample_count].target_current_A = target_current_A;
      fieldbus->samples[fieldbus->sample_count].ai1_value_V = ai1_value_V;
      fieldbus->samples[fieldbus->sample_count].ai2_value_V = ai2_value_V;
      fieldbus->samples[fieldbus->sample_count].cycle_jitter_us = cycle_jitter_us;
      fieldbus->samples[fieldbus->sample_count].pdo_exchange_us = pdo_exchange_us;
      fieldbus->sample_count++;
   }
}

/** \brief Record one fault event to the in-memory fault buffer
 *  Does not print to console: this is called from inside the real-time cyclic loop, where a
 *  blocking write syscall would add unbounded jitter right when a fault is already occurring.
 *  Fault events are printed to console (and CSV) by export_csv() after the loop has finished.
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
   }
}

/** \brief Write sample and fault buffers to timestamped CSV files in CSV_DIR, and print fault events
 *  Creates two files:
 *    - voice_coil_log_YYYYMMDD_HHMMSS.csv: samples (time_s, ai1_raw, ai2_raw, actual_current_A, cycle_jitter_us)
 *    - voice_coil_faults_YYYYMMDD_HHMMSS.csv: fault events (timestamp, type, detail, action)
 *  Also prints each fault event to console (deferred from log_fault(), which cannot block on I/O
 *  since it runs inside the real-time cyclic loop).
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
      fprintf(fp, "time_s,actual_current_A,target_current_A,demand_current_A,ai1_raw,ai2_raw, ai1_value_V, ai2_value_V, cycle_jitter_us, pdo_exchange_us\n");
      for (i = 0; i < fieldbus->sample_count; i++)
      {
         fprintf(fp, "%.6f,%.6f,%.6f,%.6f,%u,%u,%.6f,%.6f,%.1f,%.1f\n",
                 fieldbus->samples[i].timestamp_s,
                 fieldbus->samples[i].actual_current_A,
                 fieldbus->samples[i].target_current_A,
                 fieldbus->samples[i].demand_current_A,
                 fieldbus->samples[i].ai1_raw,
                 fieldbus->samples[i].ai2_raw,
                 fieldbus->samples[i].ai1_value_V,
                 fieldbus->samples[i].ai2_value_V,
                 fieldbus->samples[i].cycle_jitter_us,
                 fieldbus->samples[i].pdo_exchange_us);
      }
      fclose(fp);
      printf("Wrote %d samples to %s\n", fieldbus->sample_count, sample_file);
   }
   else
   {
      printf("ERROR: Could not open %s for writing\n", sample_file);
   }

   /* Write fault log. Also prints each fault to console here (deferred from log_fault(),
    * which is called from inside the real-time cyclic loop and must not block on I/O). */
   const char *fault_names[] = {"WKC_ERROR", "ALstatuscode_CHANGE", "STATE_DRIFT", "DRIVE_STATUS_FLAG"};
   const char *recovery_names[] = {"NONE", "AUTO_RECOVER", "SHUTDOWN_INITIATED"};

   snprintf(fault_file, sizeof(fault_file), "%s/voice_coil_faults_%s.csv", CSV_DIR, timestamp);
   fp = fopen(fault_file, "w");
   if (fp)
   {
      fprintf(fp, "fault_timestamp_s,fault_type,fault_detail,recovery_action\n");
   }
   else
   {
      printf("ERROR: Could not open %s for writing\n", fault_file);
   }

   for (i = 0; i < fieldbus->fault_count; i++)
   {
      printf("[FAULT] t=%.3fs type=%s detail=0x%04X recovery=%s\n",
             fieldbus->faults[i].timestamp_s,
             fault_names[fieldbus->faults[i].fault_type],
             fieldbus->faults[i].fault_detail,
             recovery_names[fieldbus->faults[i].recovery_action]);
      if (fp)
      {
         fprintf(fp, "%.6f,%s,0x%04X,%s\n",
                 fieldbus->faults[i].timestamp_s,
                 fault_names[fieldbus->faults[i].fault_type],
                 fieldbus->faults[i].fault_detail,
                 recovery_names[fieldbus->faults[i].recovery_action]);
      }
   }

   if (fp)
   {
      fclose(fp);
      printf("Wrote %d fault events to %s\n", fieldbus->fault_count, fault_file);
   }
}
