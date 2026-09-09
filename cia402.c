/** \file cia402.c
 * \brief CiA402 state machine bring-up: progress drive through enable sequence
 */

#include "voice_coil.h"

/** \brief Progress drive through CiA402 state machine to Operation Enabled
 *  Executes state transitions: Shutdown→Ready to Switch On→Switched On→Operation Enabled
 *  Each step polls StatusWord via PDO exchange until target state reached or timeout.
 *  \param fieldbus Fieldbus context
 *  \return TRUE when Operation Enabled reached, FALSE on timeout
 */
boolean
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
