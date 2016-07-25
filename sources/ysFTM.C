/******************************************************************************
| includes                          
|----------------------------------------------------------------------------*/
#include "ysFTM.h"

/******************************************************************************
| local variable definitions                          
|----------------------------------------------------------------------------*/

/******************************************************************************
| global variable definitions                          
|----------------------------------------------------------------------------*/

/******************************************************************************
@brief   Initialization of FTM0

@param   N/A

@return  N/A
******************************************************************************/
void InitFTM0(void)
{
  /* Disable all channel 0-5 outputs using the OUTPUT MASK feature.
  (please note that the output pins are still driven as GPIO since the
  channel mode is set to FTM channel disabled after RESET) */
  FTM_WR_OUTMASK(FTM0, 0x3F);                     
    
  /* disable write protection for certain registers */
  FTM_WR_MODE_WPDIS(FTM0, TRUE); 
    
  /* enable the counter */
  FTM_WR_MODE_FTMEN(FTM0, TRUE);
    
  /* fault cotrol mode - fault control is enabled for all channels, and
  the selected mode is the manual fault clearing */
  //FTM_WR_MODE_FAULTM(FTM0, 0x03);
    
  /* counter running in BDM mode */
  FTM_WR_CONF_BDMMODE(FTM0, 0x03);
       
  /* set modulo register */
  FTM_WR_MOD(FTM0, (uint32_t)((FTM0_MODULO / 2) - 1));
  
  /* set initial counting value */
  FTM_WR_CNTIN(FTM0, (uint32_t) (-FTM0_MODULO / 2));
 
  /* PWM update at counter in maximal value */
  FTM_WR_SYNC_CNTMAX(FTM0, TRUE);
    
  /* set combine mode */
  FTM_WR_COMBINE_COMBINE0(FTM0, TRUE);
  FTM_WR_COMBINE_COMBINE1(FTM0, TRUE);
  FTM_WR_COMBINE_COMBINE2(FTM0, TRUE);
    
  /* set complementary PWM */
  FTM_WR_COMBINE_COMP0(FTM0, TRUE); 
  FTM_WR_COMBINE_COMP1(FTM0, TRUE); 
  FTM_WR_COMBINE_COMP2(FTM0, TRUE); 
    
  /* enable dead time */
  FTM_WR_COMBINE_DTEN0(FTM0, TRUE);
  FTM_WR_COMBINE_DTEN1(FTM0, TRUE);   
  FTM_WR_COMBINE_DTEN2(FTM0, TRUE);   
                  
  /* enable PWM update synchronization */
  FTM_WR_COMBINE_SYNCEN0(FTM0, TRUE); 
  FTM_WR_COMBINE_SYNCEN1(FTM0, TRUE);  
  FTM_WR_COMBINE_SYNCEN2(FTM0, TRUE);  
             
  /* enable fault control */
  //FTM_WR_COMBINE_FAULTEN0(FTM0, TRUE);       
  //FTM_WR_COMBINE_FAULTEN1(FTM0, TRUE);       
  //FTM_WR_COMBINE_FAULTEN2(FTM0, TRUE);                                       
    
  /* recomended value of deadtime for FNB41560 on HVP-MC3PH is 1.5us
  DTPS x DTVAL = T_dead * f_fpc = 1.5us * 74MHz = 111 ~ 112 = 4 x 28 */
  FTM_WR_DEADTIME_DTPS(FTM0, 0x2); 
  FTM_WR_DEADTIME_DTVAL(FTM0, 28);
    
  /* initial setting of value registers to 50 % of duty cycle  */
  FTM_WR_CnV_VAL(FTM0, 0, (uint32_t)(-FTM0_MODULO / 4));
  FTM_WR_CnV_VAL(FTM0, 1, (uint32_t)(FTM0_MODULO / 4));
  FTM_WR_CnV_VAL(FTM0, 2, (uint32_t)(-FTM0_MODULO / 4));
  FTM_WR_CnV_VAL(FTM0, 3, (uint32_t)(FTM0_MODULO / 4));    
  FTM_WR_CnV_VAL(FTM0, 4, (uint32_t)(-FTM0_MODULO / 4));
  FTM_WR_CnV_VAL(FTM0, 5, (uint32_t)(FTM0_MODULO / 4));    

  /* note:
  1. From this moment the output pins are under FTM control. Since the PWM 
  output is disabled by the FTM0OUTMASK register, there is no change on 
  PWM outputs. Before the channel mode is set, the correct output pin 
  polarity has to be defined.
  2. Even if the odd channels are generated automatically by complementary 
  logic, these channels have to be set to be in the same channel mode. */
  FTM_WR_CnSC_ELSB(FTM0, 0, TRUE); 
  FTM_WR_CnSC_ELSB(FTM0, 1, TRUE); 
  FTM_WR_CnSC_ELSB(FTM0, 2, TRUE); 
  FTM_WR_CnSC_ELSB(FTM0, 3, TRUE); 
  FTM_WR_CnSC_ELSB(FTM0, 4, TRUE); 
  FTM_WR_CnSC_ELSB(FTM0, 5, TRUE);
 
  /* set LOAD OK register */
  FTM_WR_PWMLOAD_LDOK(FTM0, TRUE);
  
  /* initialization trigger enable */
  //FTM_WR_EXTTRIG_INITTRIGEN(FTM0, TRUE);  // ???????????
    
  /* initialize the channels output */
  FTM_WR_MODE_INIT(FTM0, TRUE);                                               
    
  /* set system clock as source for FTM0 (CLKS[1:0] = 01) */
  FTM_WR_SC_CLKS(FTM0, 0x01);
  //FTM_WR_SC_CPWMS(FTM0, 0x01);
    
  /* set fault fitler to 15 agreeing consecutive samples */
  //FTM_WR_FLTCTRL_FFVAL(FTM0, 0xF); 
    
  /* enable fault 3 - FNB41560 10.5A over-current flag */  
  //FTM_WR_FLTCTRL_FAULT3EN(FTM0, TRUE);
    
  /* fault 3 polarity setting (active low) */
  //FTM_WR_FLTPOL_FLT3POL(FTM0, TRUE);
    
  /* fault 3 is connected to XBARA_49 */
  //SIM_WR_SOPT4_FTM0FLT3(SIM, TRUE);
    
  /* enable fault 1 - adjustable over-current flag from comparator 1 */    
  //FTM_WR_FLTCTRL_FAULT1EN(FTM0, TRUE); 
    
  /* fault 1 is connected to CMP1_OUT */
  //SIM_WR_SOPT4_FTM0FLT1(SIM, TRUE);
    
  /* fault 1 polarity setting (active high) */
  //FTM_WR_FLTPOL_FLT1POL(FTM0, FALSE);
        
  /* set ports */
  PORT_WR_PCR_MUX(PORTD, 0, 5);                                               /* HVP-MC3PH phase A top */
  PORT_WR_PCR_MUX(PORTD, 1, 5);                                               /* HVP-MC3PH phase A bottom */
  PORT_WR_PCR_MUX(PORTD, 2, 5);                                               /* HVP-MC3PH phase B top */
  PORT_WR_PCR_MUX(PORTD, 3, 5);                                               /* HVP-MC3PH phase B bottom */
  PORT_WR_PCR_MUX(PORTD, 4, 4);                                               /* HVP-MC3PH phase C top */
  PORT_WR_PCR_MUX(PORTD, 5, 4);                                               /* HVP-MC3PH phase C bottom */
  
  FTM_WR_OUTMASK(FTM0, 0x00);     
  
  FTM_WR_SC_TOIE(FTM0, 0x01);
  /* enable & setup interrupts */
  NVIC_EnableIRQ(FTM0_IRQn);                                                  /* enable Interrupt */
  NVIC_SetPriority(FTM0_IRQn, 3);                                             /* set priority to interrupt */
}

/******************************************************************************
@brief   Initialization of FTM1¡ª¡ª method M

@param   N/A

@return  N/A
******************************************************************************/
void InitFTM1(void)
{
  /* Disable all channel 0-1 outputs using the OUTPUT MASK feature. */
  FTM_WR_OUTMASK(FTM1, 0x03);                     
    
  /* disable write protection for certain registers */
  FTM_WR_MODE_WPDIS(FTM1, TRUE); 
    
  /* enable the counter */
  FTM_WR_MODE_FTMEN(FTM1, TRUE);
    
  /* counter running in BDM mode */
  FTM_WR_CONF_BDMMODE(FTM1, 0x03);
       
  /* set modulo register */
  FTM_WR_MOD(FTM1, (uint32_t)FTM1_MODULO);
  
  /* set initial counting value */
  FTM_WR_CNTIN(FTM1, 0);                                                 
        
  FTM_WR_FILTER_CH0FVAL(FTM1, 0x03);
  FTM_WR_FILTER_CH1FVAL(FTM1, 0x03);
  
  /* initial setting of value registers to 0 % of duty cycle  */
  FTM_WR_CnV_VAL(FTM1, 0, 0);
  FTM_WR_CnV_VAL(FTM1, 1, 0);

  FTM_WR_CnSC_ELSA(FTM1, 0, TRUE); 
  FTM_WR_CnSC_ELSA(FTM1, 1, TRUE); 
  
  FTM_WR_QDCTRL_PHAFLTREN(FTM1, TRUE); 
  FTM_WR_QDCTRL_PHBFLTREN(FTM1, TRUE); 
  FTM_WR_QDCTRL_QUADEN(FTM1, TRUE); 
    
  /* initialize the channels output */
  FTM_WR_MODE_INIT(FTM1, TRUE);                                               
    
  /* set system clock as source for FTM1 (CLKS[1:0] = 01) */
  FTM_WR_SC_CLKS(FTM1, 0x01);
}