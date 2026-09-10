/** \file fieldbus.c
 * \brief Core EtherCAT fieldbus lifecycle: initialization, network discovery, and state management
 */

#include "main.h"

/** \brief Initialize Fieldbus structure with zero values and interface name
 *  \param fieldbus Pointer to Fieldbus structure to initialize
 *  \param iface Network interface name (e.g., "eth0")
 */
void
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
int
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

/** \brief Initialize EtherCAT network: detect slaves, assign config hook, map I/O, configure DC, reach Operational
 *  Detects AMC servo drive by vendor ID (0xBD) and assigns amc_slave_config PO2SOconfig hook.
 *  \param fieldbus Fieldbus context
 *  \return TRUE on success, FALSE if any step fails
 */
boolean
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
void
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
