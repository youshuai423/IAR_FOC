/******************************************************************************
| includes                          
|----------------------------------------------------------------------------*/
#include "main.h"
#include "math.h"

/******************************************************************************
| local variable definitions                          
|----------------------------------------------------------------------------*/
PHASE_ABC iabc;
PHASE_DQ idq;
PHASE_ALBE ualbe;
PHASE_DQ udq;

double theta = 0;

unsigned int Tinv[3] = {0, 0, 0};  // ������Ӧ�Ƚ�ֵ
unsigned int last[3];  // ������Tinvֵ(for test)

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
@brief  Openloop 
******************************************************************************/
/*void PWMA_RELOAD0_IRQHandler(void)
{
  positionSVM(Tinv);
  PWM_WR_VAL2(PWMA, 0, -Tinv[0]);
  PWM_WR_VAL2(PWMA, 1, -Tinv[1]);
  PWM_WR_VAL2(PWMA, 2, -Tinv[2]);
  
  PWM_WR_VAL3(PWMA, 0, Tinv[0]);
  PWM_WR_VAL3(PWMA, 1, Tinv[1]);
  PWM_WR_VAL3(PWMA, 2, Tinv[2]);
  
  period_count++;
  if (period_count > 1000) 
  {
    period_count = 0;
  }

  last[0] = Tinv[0];
  last[1] = Tinv[1];
  last[2] = Tinv[2];
    
  PWM_WR_STS_RF(PWMA, 0, 1);
  // start PWMs (set load OK flags and run)
  PWM_WR_MCTRL_LDOK(PWMA, 1);
  
  //GPIO_WR_PCOR(PTB, 1<<22);
}*/

/******************************************************************************
@brief   FOC 
******************************************************************************/
void PWMA_RELOAD0_IRQHandler(void)
{
  /* current sampling */

  /* speed calculation */

  /* 3s/2r coordinate transform */
  S3toR2(&iabc, &idq, theta);

  /* rotor flux calculation */
  lamdar = lamdarCal(lamdar, idq.d);

  /* theta calculation */

  /* ud* calculation */
  udq.d = PImodule(ud_Kp, ud_Ki, idset - idq.d, &ud_Isum, ud_Uplim, ud_Downlim);

  /* uq* calculation */
  iqset = PImodule(iqset_Kp, iqset_Ki, nset - n, &iqset_Isum, iqset_Uplim, iqset_Downlim);
  udq.q = PImodule(uq_Kp, uq_Ki, iqset - idq.q, &uq_Isum, uq_Uplim, uq_Downlim);

  /* 2r/2s coordinate transform */
  R2toS2(&udq, &ualbe, theta);

  /* SVM modulation */
  ualbeSVM(ualbe.al, ualbe.be, Tinv);

  /* register setting */
  PWM_WR_VAL2(PWMA, 0, -Tinv[0]);
  PWM_WR_VAL2(PWMA, 1, -Tinv[1]);
  PWM_WR_VAL2(PWMA, 2, -Tinv[2]);
  
  PWM_WR_VAL3(PWMA, 0, Tinv[0]);
  PWM_WR_VAL3(PWMA, 1, Tinv[1]);
  PWM_WR_VAL3(PWMA, 2, Tinv[2]);

  PWM_WR_STS_RF(PWMA, 0, 1);
  
  PWM_WR_MCTRL_LDOK(PWMA, 1);  // start PWMs (set load OK flags and run)
  
  //GPIO_WR_PCOR(PTB, 1<<22);
}

void PWMA_RERR_IRQHandler(void)
{
  GPIO_WR_PSOR(PTB, 1<<22);
}
