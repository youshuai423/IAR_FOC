/******************************************************************************
| includes                          
|----------------------------------------------------------------------------*/
#include "ysPORT.h"

/******************************************************************************
| local variable definitions                          
|----------------------------------------------------------------------------*/

/******************************************************************************
| global variable definitions                          
|----------------------------------------------------------------------------*/

/******************************************************************************
@brief   Initialization of ports

@param   N/A

@return  N/A
******************************************************************************/
void InitPORT(void)
{
    /* enable clock to all PORTs */
    SIM_WR_SCGC5_PORTA(SIM, 1);                                              /* PTA clock enabled */
    SIM_WR_SCGC5_PORTB(SIM, 1);                                              /* PTB clock enabled */
    SIM_WR_SCGC5_PORTC(SIM, 1);                                              /* PTC clock enabled */
    SIM_WR_SCGC5_PORTD(SIM, 1);                                              /* PTD clock enabled */
    SIM_WR_SCGC5_PORTE(SIM, 1);                                              /* PTE clock enabled */
}