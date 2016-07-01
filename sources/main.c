/******************************************************************************
| includes                          
|----------------------------------------------------------------------------*/
#include "main.h"
#include "math.h"

/******************************************************************************
| local variable definitions                          
|----------------------------------------------------------------------------*/
unsigned int Tinv[3] = {0, 0, 0};  // 
unsigned int last[3];  // 

/******************************************************************************
| global variable definitions                          
|----------------------------------------------------------------------------*/

/******************************************************************************
@brief  Main 
******************************************************************************/
void main(void)
{
  /* disable all interrupts before peripherals are initialized */
  __disable_irq();
  
  /* init application ports */  
  InitPORT();  
  InitPWM();   
  //InitFTM0();
  InitADC();
  //Init_PIT();
  
  int i = 0;
  for(i = 0; i < 1000; i++){};
  ADC_WR_CTRL1_START0(ADC, 1);
    
  /* LED for test */ 
  PORT_WR_PCR_MUX(PORTB, 22, 1); 
  GPIO_SET_PDDR(PTB, 1<<22);
  GPIO_WR_PSOR(PTB, 1<<22);

  /* enable interrupts  */
  __enable_irq();

  /* infinite loop */
  while(1){}
}

/******************************************************************************
@brief   FOC 
******************************************************************************/
void PWMA_RELOAD0_IRQHandler(void)
{
  Ud = 60;
  ualbe_cmd.al = 40 * cos(2*pi*40 * (period_count/10000.0));
  ualbe_cmd.be = 40 * cos(2*pi*40 * (period_count/10000.0) - 0.5*pi);
  
  period_count++;
  if (period_count > 10000) 
  {
    period_count = 0;
  }

  /* current sampling and voltage calculation *

  /* speed calculation *
  wrCal(&lamdaralbe, &anglek, ualbe, ialbe, Ts);
  
  /* 3s/2r coordinate transform *
  S3toR2(&iabc, &idq, theta);

  /* rotor flux calculation *
  lamdar = lamdarCal(lamdar, idq.d);

  /* theta calculation *
  theta = positonCal(wr, lamdar, idq.q, theta);

  /* ud* calculation *
  udq_cmd.d = PImodule(ud_Kp, ud_Ki, idset - idq.d, &ud_Isum, ud_Uplimit, ud_Downlimit);

  /* uq* calculation *
  if (n < 370)
    iqset = PImodule(iqset_Kp1, iqset_Ki1, nset - n, &iqset_Isum, iqset_Uplimit, iqset_Downlimit);
  else
    iqset = PImodule(iqset_Kp2, iqset_Ki2, nset - n, &iqset_Isum, iqset_Uplimit, iqset_Downlimit);
  
  udq_cmd.q = PImodule(uq_Kp, uq_Ki, iqset - idq.q, &uq_Isum, uq_Uplimit, uq_Downlimit);

  /* 2r/2s coordinate transform *
  R2toS2(&udq_cmd, &ualbe_cmd, theta); */

  /* SVM modulation */
  ualbeSVM(ualbe_cmd.al, ualbe_cmd.be, Ud, Tinv);

  /* register setting */
  PWM_WR_VAL2(PWMA, 0, -Tinv[0]);
  PWM_WR_VAL2(PWMA, 1, -Tinv[1]);
  PWM_WR_VAL2(PWMA, 2, -Tinv[2]);
  
  PWM_WR_VAL3(PWMA, 0, Tinv[0]);
  PWM_WR_VAL3(PWMA, 1, Tinv[1]);
  PWM_WR_VAL3(PWMA, 2, Tinv[2]);

  PWM_WR_STS_RF(PWMA, 0, 1);
  
  PWM_WR_MCTRL_LDOK(PWMA, 1);  // start PWMs (set load OK flags and run)
  
  //GPIO_WR_PCOR(PTB, 1<<22); */
}

void PWMA_RERR_IRQHandler(void)
{
  GPIO_WR_PSOR(PTB, 1<<22);
}

void FTM0_IRQHandler(void)
{
  //FTM_RD_SC(FTM0);
  //FTM_WR_SC_TOF(FTM0, 0x00);
  Ud = 60;
  ualbe_cmd.al = 40 * cos(2*pi*40 * (period_count/10000.0));
  ualbe_cmd.be = 40 * cos(2*pi*40 * (period_count/10000.0) - 0.5*pi);
  
  period_count++;
  if (period_count > 10000) 
  {
    period_count = 0;
  }
  
  /* SVM modulation */
  ualbeSVM(ualbe_cmd.al, ualbe_cmd.be, Ud, Tinv);

  /* register setting */
  FTM_WR_CnV_VAL(FTM0, 0, (uint32_t)(-Tinv[0]));
  FTM_WR_CnV_VAL(FTM0, 2, (uint32_t)(-Tinv[1]));
  FTM_WR_CnV_VAL(FTM0, 4, (uint32_t)(-Tinv[2]));
  FTM_WR_CnV_VAL(FTM0, 1, (uint32_t)(Tinv[0]));    
  FTM_WR_CnV_VAL(FTM0, 3, (uint32_t)(Tinv[1]));
  FTM_WR_CnV_VAL(FTM0, 5, (uint32_t)(Tinv[2]));  

  FTM_WR_SC_TOF(FTM0, 0x00);
  FTM_WR_PWMLOAD_LDOK(FTM0, TRUE);
  
  //GPIO_WR_PCOR(PTB, 1<<22); */
}