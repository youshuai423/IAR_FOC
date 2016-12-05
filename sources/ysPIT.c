/******************************************************************************
| includes                          
|----------------------------------------------------------------------------*/
#include "ysPIT.h"

/******************************************************************************
@brief   Initialization of PIT

@param   N/A

@return  N/A
******************************************************************************/
void InitPIT(void)
{
  /* Init PIT */
  //PIT_WR_MCR_MDIS(PIT, 0);
  PIT_WR_MCR_FRZ(PIT, 1);
  PIT_WR_LDVAL(PIT, 0, PIT_MODULO_1ms);  // ¶¨Ê±100ms
  PIT_WR_TCTRL_TIE(PIT, 0, 1);
  PIT_WR_TCTRL_TEN(PIT, 0, 1);
  
  /* enable & setup interrupts */
  NVIC_EnableIRQ(PIT0_IRQn);                                                  /* enable Interrupt */
  NVIC_SetPriority(PIT0_IRQn, 3);                                             /* set priority to interrupt */
}