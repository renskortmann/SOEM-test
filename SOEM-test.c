/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage: SOEM-test IFNAME1
 * IFNAME1 is the NIC interface name, e.g. 'eth0'
 *
 * This is a minimal test.
 */

#include "soem/soem.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/// \brief Structure to hold the state of the fieldbus
// This structure is used to keep track of the EtherCAT context, 
// interface name, group number, roundtrip time, and mapping of I/O bytes.
typedef struct
{
   ecx_contextt context;
   char *iface;
   uint8 group;
   int roundtrip_time;
   uint8 map[4096];
} Fieldbus;

/// \brief Initialize the fieldbus structure
// This function initializes the Fieldbus structure by zero-filling it and 
// setting the interface name, group number, and roundtrip time to default values.
static void
fieldbus_initialize(Fieldbus *fieldbus, char *iface)
{
   /* Let's start by 0-filling `fieldbus` to avoid surprises */
   memset(fieldbus, 0, sizeof(*fieldbus));

   fieldbus->iface = iface;
   fieldbus->group = 0;
   fieldbus->roundtrip_time = 0;
}

/// \brief Perform a roundtrip test
// This function performs a roundtrip test by sending and receiving process data,
// and calculates the roundtrip time.
static int
fieldbus_roundtrip(Fieldbus *fieldbus)
{
   ecx_contextt *context;
   ec_timet start, end, diff;
   int wkc;

   context = &fieldbus->context;

   start = osal_current_time();
   ecx_send_processdata(context);
   wkc = ecx_receive_processdata(context, EC_TIMEOUTRET);
   end = osal_current_time();
   osal_time_diff(&start, &end, &diff);
   fieldbus->roundtrip_time = (int)(diff.tv_sec * 1000000 + diff.tv_nsec / 1000);

   return wkc;
}

/// \brief Start the fieldbus
// This function initializes the fieldbus and performs the necessary steps to start it.
static boolean
fieldbus_start(Fieldbus *fieldbus)
{
   ecx_contextt *context;
   ec_groupt *grp;
   ec_slavet *slave;
   int i;

   context = &fieldbus->context;
   grp = context->grouplist + fieldbus->group;

   printf("Initializing SOEM on '%s'... ", fieldbus->iface);
   // Initialize the EtherCAT context and connect to the specified interface
   if (!ecx_init(context, fieldbus->iface))
   {
      printf("no socket connection\n");
      return FALSE;
   }
   printf("done\n");

   printf("Finding autoconfig slaves... ");
   // Find and configure the slaves on the EtherCAT network
   if (ecx_config_init(context) <= 0)
   {
      printf("no slaves found\n");
      return FALSE;
   }
   printf("%d slaves found\n", context->slavecount);

   printf("Sequential mapping of I/O... ");
   // Map the I/O bytes of the slaves to the process data image
   ecx_config_map_group(context, fieldbus->map, fieldbus->group);
   printf("mapped %dO+%dI bytes from %d segments",
          grp->Obytes, grp->Ibytes, grp->nsegments);
   if (grp->nsegments > 1)
   {
      /* Show how slaves are distributed */
      for (i = 0; i < grp->nsegments; ++i)
      {
         printf("%s%d", i == 0 ? " (" : "+", grp->IOsegment[i]);
      }
      printf(" slaves)");
   }
   printf("\n");

   printf("Configuring distributed clock... ");
   // Configure the distributed clock for synchronization among slaves
   ecx_configdc(context);
   printf("done\n");

   printf("Waiting for all slaves in safe operational... ");
   // Request all slaves to enter the SAFE_OP state and wait for confirmation
   ecx_statecheck(context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
   printf("done\n");

   printf("Send a roundtrip to make outputs in slaves happy... ");
   // Perform a roundtrip to ensure that the outputs in the slaves are updated and stable 
   fieldbus_roundtrip(fieldbus);
   printf("done\n");

   printf("Setting operational state..");
   /* Act on slave 0 (a virtual slave used for broadcasting) */
   slave = context->slavelist;
   slave->state = EC_STATE_OPERATIONAL;
   ecx_writestate(context, 0);
   /* Poll the result ten times before giving up */
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
   printf(" failed,");

   // Read the state of all slaves and print their status if they are not operational
   ecx_readstate(context);
   for (i = 1; i <= context->slavecount; ++i)
   {
      slave = context->slavelist + i;
      if (slave->state != EC_STATE_OPERATIONAL)
      {
         printf(" slave %d is 0x%04X (AL-status=0x%04X %s)",
                i, slave->state, slave->ALstatuscode,
                ec_ALstatuscode2string(slave->ALstatuscode));
      }
   }
   printf("\n");

   // If the function reaches this point, it means that not all slaves could be set to 
   // operational state
   return FALSE;
}

/// \brief Stop the fieldbus
// This function stops the fieldbus by requesting the init state on all slaves and 
// closing the socket.
static void
fieldbus_stop(Fieldbus *fieldbus)
{
   ecx_contextt *context;
   ec_slavet *slave;

   context = &fieldbus->context;
   /* Act on slave 0 (a virtual slave used for broadcasting) */
   slave = context->slavelist;

   printf("Requesting init state on all slaves... ");
   slave->state = EC_STATE_INIT;
   ecx_writestate(context, 0);
   printf("done\n");

   printf("Close socket... ");
   ecx_close(context);
   printf("done\n");
}

/// \brief Dump the current state of the fieldbus
// This function dumps the current state of the fieldbus, including the roundtrip time, 
// working counter (wkc), and the values of the output and input bytes.
static boolean
fieldbus_dump(Fieldbus *fieldbus)
{
   ecx_contextt *context;
   ec_groupt *grp;
   uint32 n;
   int wkc, expected_wkc;

   context = &fieldbus->context;
   grp = context->grouplist + fieldbus->group;

   // Perform a roundtrip and calculate the expected working counter (wkc)
   wkc = fieldbus_roundtrip(fieldbus);
   // The expected working counter is calculated based on the number of output and input bytes
   expected_wkc = grp->outputsWKC * 2 + grp->inputsWKC;
   printf("%6d usec  WKC %d", fieldbus->roundtrip_time, wkc);
   // Check if the actual working counter is less than the expected value
   if (wkc < expected_wkc)
   {
      printf(" wrong (expected %d)\n", expected_wkc);
      return FALSE;
   }

   printf("  O:");
   // Print the values of the output bytes in hexadecimal format
   for (n = 0; n < grp->Obytes; ++n)
   {
      printf(" %02X", grp->outputs[n]);
   }
   printf("  I:");
   // Print the values of the input bytes in hexadecimal format
   for (n = 0; n < grp->Ibytes; ++n)
   {
      printf(" %02X", grp->inputs[n]);
   }
   printf("  T: %lld\r", (long long)context->DCtime);
   return TRUE;
}

/// \brief Check the state of the fieldbus
// This function checks the state of each slave in the fieldbus and attempts to recover 
// any that are in an error state.
static void
fieldbus_check_state(Fieldbus *fieldbus)
{
   ecx_contextt *context;
   ec_groupt *grp;
   ec_slavet *slave;
   int i;

   context = &fieldbus->context;
   grp = context->grouplist + fieldbus->group;
   grp->docheckstate = FALSE;
   ecx_readstate(context);
   for (i = 1; i <= context->slavecount; ++i)
   {
      slave = context->slavelist + i;
      if (slave->group != fieldbus->group)
      {
         /* This slave is part of another group: do nothing */
      }
      else if (slave->state != EC_STATE_OPERATIONAL)
      {
         grp->docheckstate = TRUE;
         if (slave->state == EC_STATE_SAFE_OP + EC_STATE_ERROR)
         {
            printf("* Slave %d is in SAFE_OP+ERROR, attempting ACK\n", i);
            slave->state = EC_STATE_SAFE_OP + EC_STATE_ACK;
            ecx_writestate(context, i);
         }
         else if (slave->state == EC_STATE_SAFE_OP)
         {
            printf("* Slave %d is in SAFE_OP, change to OPERATIONAL\n", i);
            slave->state = EC_STATE_OPERATIONAL;
            ecx_writestate(context, i);
         }
         else if (slave->state > EC_STATE_NONE)
         {
            if (ecx_reconfig_slave(context, i, EC_TIMEOUTRET))
            {
               slave->islost = FALSE;
               printf("* Slave %d reconfigured\n", i);
            }
         }
         else if (!slave->islost)
         {
            ecx_statecheck(context, i, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (slave->state == EC_STATE_NONE)
            {
               slave->islost = TRUE;
               printf("* Slave %d lost\n", i);
            }
         }
      }
      else if (slave->islost)
      {
         if (slave->state != EC_STATE_NONE)
         {
            slave->islost = FALSE;
            printf("* Slave %d found\n", i);
         }
         else if (ecx_recover_slave(context, i, EC_TIMEOUTRET))
         {
            slave->islost = FALSE;
            printf("* Slave %d recovered\n", i);
         }
      }
   }

   if (!grp->docheckstate)
   {
      printf("All slaves resumed OPERATIONAL\n");
   }
}

/// \brief Main function
// This is the main function of the program, which initializes the fieldbus, 
// starts it, and performs a series of tests.
int main(int argc, char *argv[])
{
   Fieldbus fieldbus;

   if (argc != 2)
   {
      ec_adaptert *adapter = NULL;
      ec_adaptert *head = NULL;

      // Print usage information and list available network adapters
      printf("Usage: SOEM-test IFNAME1\n"
             "IFNAME1 is the NIC interface name, e.g. 'eth0'\n");

      printf("\nAvailable adapters:\n");
      head = adapter = ec_find_adapters();
      while (adapter != NULL)
      {
         printf("    - %s  (%s)\n", adapter->name, adapter->desc);
         adapter = adapter->next;
      }
      ec_free_adapters(head);
      return 1;
   }

   // Initialize the fieldbus with the specified interface name
   fieldbus_initialize(&fieldbus, argv[1]);
   // Start the fieldbus and perform tests if successful
   if (fieldbus_start(&fieldbus))
   {
      int i, min_time, max_time;
      min_time = max_time = 0;

      // for (i = 1; i <= 10000; ++i)
      // {
      //    printf("Iteration %4d:", i);
      //    // Dump the current state of the fieldbus and check for errors
      //    if (!fieldbus_dump(&fieldbus))
      //    {
      //       // If there is an error, check the state of the fieldbus and attempt recovery
      //       fieldbus_check_state(&fieldbus);
      //    }
      //    else if (i == 1)
      //    {
      //       min_time = max_time = fieldbus.roundtrip_time;
      //    }
      //    else if (fieldbus.roundtrip_time < min_time)
      //    {
      //       min_time = fieldbus.roundtrip_time;
      //    }
      //    else if (fieldbus.roundtrip_time > max_time)
      //    {
      //       max_time = fieldbus.roundtrip_time;
      //    }
      //    // Sleep for 5 milliseconds before the next iteration
      //    osal_usleep(5000);
      // }
      // printf("\nRoundtrip time (usec): min %d max %d\n", min_time, max_time);

      fieldbus_stop(&fieldbus);
   }

   return 0;
}
