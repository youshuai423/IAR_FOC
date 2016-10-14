/******************************************************************************
| includes                          
|----------------------------------------------------------------------------*/
#include "ysPWM.h"

/******************************************************************************
| local variable definitions                          
|----------------------------------------------------------------------------*/

/******************************************************************************
| global variable definitions                          
|----------------------------------------------------------------------------*/

/******************************************************************************
@brief   Initialization of PWMs

@param   N/A

@return  N/A
******************************************************************************/
void InitPWM(void)
{   
  /* full cycle reload */
  PWM_WR_CTRL_FULL(PWMA, 0, 1);
  PWM_WR_CTRL_FULL(PWMA, 1, 1);
  PWM_WR_CTRL_FULL(PWMA, 2, 1);
    
  /* value register initial values, duty cycle 50% */
  PWM_WR_INIT(PWMA, 0, (uint16_t)(-(M1_PWM_MODULO/2)));
  PWM_WR_INIT(PWMA, 1, (uint16_t)(-(M1_PWM_MODULO/2)));
  PWM_WR_INIT(PWMA, 2, (uint16_t)(-(M1_PWM_MODULO/2)));
    
  PWM_WR_VAL0(PWMA, 0, (uint16_t)(0));
  PWM_WR_VAL0(PWMA, 1, (uint16_t)(0));
  PWM_WR_VAL0(PWMA, 2, (uint16_t)(0));
    
  PWM_WR_VAL1(PWMA, 0, (uint16_t)((M1_PWM_MODULO/2)-1));
  PWM_WR_VAL1(PWMA, 1, (uint16_t)((M1_PWM_MODULO/2)-1));
  PWM_WR_VAL1(PWMA, 2, (uint16_t)((M1_PWM_MODULO/2)-1));
  
  PWM_WR_VAL2(PWMA, 0, (uint16_t)(-M1_PWM_MODULO/2));
  PWM_WR_VAL2(PWMA, 1, (uint16_t)(-M1_PWM_MODULO)/2);
  PWM_WR_VAL2(PWMA, 2, (uint16_t)(-M1_PWM_MODULO/2));
    
  PWM_WR_VAL3(PWMA, 0, (uint16_t)(M1_PWM_MODULO));
  PWM_WR_VAL3(PWMA, 1, (uint16_t)(M1_PWM_MODULO));
  PWM_WR_VAL3(PWMA, 2, (uint16_t)(M1_PWM_MODULO));
    
  PWM_WR_VAL4(PWMA, 0, (uint16_t)(0));
  PWM_WR_VAL4(PWMA, 1, (uint16_t)(0));
  PWM_WR_VAL4(PWMA, 2, (uint16_t)(0));
    
  PWM_WR_VAL5(PWMA, 0, (uint16_t)(0)); 
  PWM_WR_VAL5(PWMA, 1, (uint16_t)(0));
  PWM_WR_VAL5(PWMA, 2, (uint16_t)(0));

  /* recomended value of deadtime for FNB41560 on HVP-MC3PH is 1.5us
     DTCNT0,1 = T_dead * f_fpc = 1.5us * 74MHz = 111 */
  PWM_WR_DTCNT0(PWMA, 0, M1_PWM_DT);
  PWM_WR_DTCNT0(PWMA, 1, M1_PWM_DT);
  PWM_WR_DTCNT0(PWMA, 2, M1_PWM_DT);
  PWM_WR_DTCNT1(PWMA, 0, M1_PWM_DT);
  PWM_WR_DTCNT1(PWMA, 1, M1_PWM_DT);
  PWM_WR_DTCNT1(PWMA, 2, M1_PWM_DT);
      
  /* channels A and B disabled when fault 0 occurs */
  PWM_WR_DISMAP_DIS0A(PWMA, 0, 0, 0x0);
  PWM_WR_DISMAP_DIS0A(PWMA, 1, 0, 0x0);
  PWM_WR_DISMAP_DIS0A(PWMA, 2, 0, 0x0); 
  PWM_WR_DISMAP_DIS0B(PWMA, 0, 0, 0x0);
  PWM_WR_DISMAP_DIS0B(PWMA, 1, 0, 0x0);
  PWM_WR_DISMAP_DIS0B(PWMA, 2, 0, 0x0);

  /* modules one and two gets clock from module zero */
  PWM_WR_CTRL2_CLK_SEL(PWMA, 1, 0x2);
  PWM_WR_CTRL2_CLK_SEL(PWMA, 2, 0x2);
  
  /* master reload active for modules one and two*/
  PWM_WR_CTRL2_RELOAD_SEL(PWMA, 1, 1);
  PWM_WR_CTRL2_RELOAD_SEL(PWMA, 2, 1);
    
  /* master sync active for modules one and two*/
  PWM_WR_CTRL2_INIT_SEL(PWMA, 1, 0x2);
  PWM_WR_CTRL2_INIT_SEL(PWMA, 2, 0x2);
    
  /* fault 0 active in high, fault 1 active in low, manual clearing */
  PWM_WR_FCTRL_FLVL(PWMA, 0x1);
  PWM_WR_FCTRL_FAUTO(PWMA, 0x0);

  /* PWMs are re-enabled at PWM full cycle */
  PWM_WR_FSTS_FFULL(PWMA, 0x1); 
     
  /* PWM fault filter - 5 Fast periph. clocks sample rate, 5 agreeing 
     samples to activate */
  PWM_WR_FFILT_FILT_PER(PWMA, 5);
  PWM_WR_FFILT_FILT_CNT(PWMA, 5);
        
  /* enable A&B PWM outputs for submodules one, two and three */
  PWM_WR_OUTEN_PWMA_EN(PWMA, 0x7);
  PWM_WR_OUTEN_PWMB_EN(PWMA, 0x7);
    
  PWM_WR_CTRL_PRSC(PWMA, 0, 0);
  //PWM_WR_CTRL_PRSC(PWMA, 1, 3);
  //PWM_WR_CTRL_PRSC(PWMA, 2, 3);
       
  /* start PWMs (set load OK flags and run) */
  PWM_WR_MCTRL_CLDOK(PWMA, 0x7);
  PWM_WR_MCTRL_LDOK(PWMA, 0x7);
  PWM_WR_MCTRL_RUN(PWMA, 0x7);
    
  PWM_WR_INTEN_RIE(PWMA, 0 , 1);
  /* enable & setup interrupts */
  NVIC_EnableIRQ(PWMA_RELOAD0_IRQn);                                                  /* enable Interrupt */
  NVIC_SetPriority(PWMA_RELOAD0_IRQn, 3);                                             /* set priority to interrupt */
}