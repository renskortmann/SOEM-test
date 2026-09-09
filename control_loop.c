/** \file control_loop.c
 * \brief Real-time cyclic loop: sine-wave generation, PDO exchange, fault monitoring, timing
 */

#include "voice_coil.h"

/** \brief Add microseconds to a struct timespec, handling nanosecond overflow
 *  \param ts Pointer to timespec to modify in-place
 *  \param addus Microseconds to add
 */
void
add_timespec(struct timespec *ts, int64_t addus)
{
   ts->tv_nsec += addus * 1000;
   while (ts->tv_nsec >= 1000000000)
   {
      ts->tv_sec++;
      ts->tv_nsec -= 1000000000;
   }
}

/** \brief Run the real-time control loop: generate sine-wave commands, exchange PDO, monitor faults
 *  Operates for RUN_DURATION_S seconds at CYCLE_TIME_MS intervals, synchronized via DC SYNC0.
 *  Detects WKC errors and CiA402 state drift; logs samples and faults to in-memory buffers.
 *  \param fieldbus Fieldbus context (sample and fault buffers populated)
 *  \return TRUE on completion (normal or fault-triggered shutdown), FALSE on internal error
 */
boolean
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
