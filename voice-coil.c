/** \file voice-coil.c
 * \brief CST current-mode cyclic control + logging for AMC servo drive
 *
 * Usage: voice-coil IFNAME1
 * IFNAME1 is the NIC interface name, e.g. 'eth0'
 *
 * Runs a RUN_DURATION_S-second hardware-synchronized control loop commanding
 * a sine-wave current to a voice-coil motor, logging samples and faults to CSV.
 */

#include "voice_coil.h"

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
