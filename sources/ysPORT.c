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
    SIM_WR_SCGC5_PORTA(SIM, 1);  // enable clock
    SIM_WR_SCGC5_PORTB(SIM, 1); 
    SIM_WR_SCGC5_PORTC(SIM, 1);
    SIM_WR_SCGC5_PORTD(SIM, 1); 
    SIM_WR_SCGC5_PORTE(SIM, 1); 
    
    /* ADC port configuration */
    SIM_WR_SCGC5_ADC(SIM, 1);  // enable clock
  
    /* PWMA port configuration */
    SIM_WR_SCGC4_eFlexPWM0(SIM, 1);  // enable clock
    SIM_WR_SCGC4_eFlexPWM1(SIM, 1);
    SIM_WR_SCGC4_eFlexPWM2(SIM, 1);

    PORT_WR_PCR_MUX(PORTD, 0, 6);  // set port
    PORT_WR_PCR_MUX(PORTD, 1, 6); 
    PORT_WR_PCR_MUX(PORTD, 2, 6);
    PORT_WR_PCR_MUX(PORTD, 3, 6);
    PORT_WR_PCR_MUX(PORTD, 4, 5);
    PORT_WR_PCR_MUX(PORTD, 5, 5);  

    /* FTM1 port configuration */
    SIM_WR_SCGC6_FTM1(SIM, 1);  // enable clock
    
    PORT_WR_PCR_MUX(PORTA, 12, 7);  // set port
    PORT_WR_PCR_MUX(PORTA, 13, 7);  
    
    /* FTM3 port configuration */
    SIM_WR_SCGC6_FTM3(SIM, 1);
    
    PORT_WR_PCR_MUX(PORTC, 11, 3);  // set port
    PORT_WR_PCR_MUX(PORTC, 10, 3);  // set port
    
    /* PIT port configuration */
    SIM_WR_SCGC6_PIT(SIM, 1);  // enable clock
    
    /* GPIO configuration */
    PORT_WR_PCR_MUX(PORTB, 22, 1);  
    GPIO_SET_PDDR(PTB, 1<<22);
    GPIO_WR_PCOR(PTB, 1<<22);
    
    /* RUN/STOP command configuration */
    PORT_WR_PCR_MUX(PORTE, 19, 1); 
    PORT_WR_PCR_IRQC(PORTE, 19, 11);
    GPIO_SET_PDDR(PTE, 0<<19);    
      /* enable & setup interrupts */
    NVIC_EnableIRQ(PORTE_IRQn);                                          /* enable Interrupt */
    NVIC_SetPriority(PORTE_IRQn, 4);                                     /* set priority to interrupt */
}