/******************************************************************************
| includes                          
|----------------------------------------------------------------------------*/
#include "main.h"
#include "math.h"

/******************************************************************************
| local variable definitions                          
|----------------------------------------------------------------------------*/
uint16_t Tinv[3] = {0, 0, 0};  // �����ӦPWM�Ĵ����Ƚ�ֵ
int temp = 0;
int count = 0;

/******************************************************************************
@brief   Main

@param   N/A

@return  N/A
******************************************************************************/
void main(void)
{
  /* disable all interrupts before peripherals are initialized */
  __disable_irq();
  
  /* init appl1ication ports */  
  InitPORT(); 
  InitPWM();   
  InitFTM1();  // ����������
  InitFTM3();  // PWM DA
  InitADC();
  InitPIT();  // ����ת�ٺ�ת�ٸ���ֵ
 
  int i = 0;
  for(i = 0; i < 1000; i++){};  // �ȴ�ADCģ���ȶ�
  ADC_WR_CTRL1_START0(ADC, 1); 
  
  idq_cmd.d = 5;
  idq_cmd.q = 10;
  
  /* enable interrupts  */
  __enable_irq();

  /* infinite loop */
  while(1){}
}

/******************************************************************************
@brief   PWMA �ж� -- FOC����

@param   N/A

@return  N/A
******************************************************************************/
/*void PWMA_RELOAD0_IRQHandler_TEMP(void)
{  
  /* current sampling and voltage calculation *

  /* speed calculation *
  spdCal_M();
  
  /* 3s/2r coordinate transform *
  S3toR2(iabc, &idq, theta);

  /* rotor flux calculation *
  lamdar = lamdarCal(lamdar, idq.d);

  /* theta calculation *
  theta = positonCal(speed / 60.0 * np, lamdar, idq.q, theta);

  /* ud* calculation *
  udq_cmd.d = PImodule(ud_Kp, ud_Ki, udq_cmd.d, idq_cmd.d - idq.d, &idlasterr, udlimit_H, udlimit_L);

  /* uq* calculation *
  if (speed < spdthd)
    idq_cmd.q = PImodule(iq_Kp1, iq_Ki1, idq_cmd.q, spd_cmd - speed, &spdlasterr, iqlimit_H, iqlimit_L);
  else
    idq_cmd.q = PImodule(iq_Kp2, iq_Ki2, idq_cmd.q, spd_cmd - speed, &spdlasterr, iqlimit_H, iqlimit_L);
 
  udq_cmd.q = PImodule(uq_Kp, uq_Ki, udq_cmd.q, idq_cmd.q - idq.q, &iqlasterr, uqlimit_H, uqlimit_L);

  /* 2r/2s coordinate transform *
  R2toS2(udq_cmd, &ualbe_cmd, theta); 

  /* SVM modulation *
  ualbeSVM(ualbe_cmd.al, ualbe_cmd.be, Ud, Tinv);

  /* register setting *
  PWM_WR_VAL2(PWMA, 0, -Tinv[0]);
  PWM_WR_VAL2(PWMA, 1, -Tinv[1]);
  PWM_WR_VAL2(PWMA, 2, -Tinv[2]);
  
  PWM_WR_VAL3(PWMA, 0, Tinv[0]);
  PWM_WR_VAL3(PWMA, 1, Tinv[1]);
  PWM_WR_VAL3(PWMA, 2, Tinv[2]);

  PWM_WR_STS_RF(PWMA, 0, 1);
  
  PWM_WR_MCTRL_LDOK(PWMA, 1);  // start PWMs (set load OK flags and run)

} */

/******************************************************************************
@brief   PORTE �ж� -- RUN/STOP����

@param   N/A

@return  N/A
******************************************************************************/
void PORTE_IRQHandler(void)
{  
  while(PORT_RD_PCR_ISF(PORTE, 19) == 1) 
    PORT_WR_PCR_ISF(PORTE, 19, 1);

  if ((GPIO_RD_PDIR(PTE) & 0x00080000) == 0x00080000) // PTE19 = 1,RUN
  {
    /* start ADCA */
    ADC_WR_CTRL1_STOP0(ADC, 0);
    ADC_WR_CTRL2_STOP1(ADC, 1);
    
    /* start PWMs (set load OK flags and run) */
    PWM_WR_MASK_MASKA(PWMA, 0);
    PWM_WR_MASK_MASKB(PWMA, 0);
    PWM_WR_MCTRL_LDOK(PWMA, 0x7);
    PWM_WR_MCTRL_RUN(PWMA, 0x7);
    
    /* start PIT */
    PIT_WR_MCR_MDIS(PIT, 0);    
  }
  else
  {
    spd_req = 0;
    
    while(spd_cmd > 100){};  // �ж����ȼ����ó���
    
    /* clear interrupt flags */
    PWM_WR_STS_RF(PWMA, 0, TRUE);
    PIT_WR_TFLG_TIF(PIT, 0, 1);
    
    /* stop ADCA */
    ADC_WR_CTRL1_STOP0(ADC, 1);
    ADC_WR_CTRL2_STOP1(ADC, 1);
    
    /* stop PWMs */
    PWM_WR_MASK_MASKA(PWMA, 1);
    PWM_WR_MASK_MASKB(PWMA, 1);
    PWM_WR_MCTRL_CLDOK(PWMA, 0x7);
    PWM_WR_MCTRL_RUN(PWMA, 0x0);
    
    /* stop PIT */
    PIT_WR_MCR_MDIS(PIT, 1); 
    
    /* initial variables */
    theta = 0;
    speed = 0;
    u_cmd = 0;
    spd_req= 450;
  }
      
}

/******************************************************************************
@brief   PWMA �ж� -- V/f��������

@param   N/A

@return  N/A
******************************************************************************/
void PWMA_RELOAD0_IRQHandler(void)
{   
    double cosIn = cos(theta);
    double sinIn = sin(theta);
    
      /* ��ȡ�������� */
    iabc.c = ADC_RD_RSLT_RSLT(ADC, 1) * 0.2111 - 400;
    iabc.a = ADC_RD_RSLT_RSLT(ADC, 2) * 0.2084 - 400;
    iabc.b = -iabc.a - iabc.c;
    
    //S3toR2(iabc, &idq, theta+1);
    S3toS2(iabc, &ialbe);
    S2toR2(ialbe, &idq, cosIn, sinIn);
    
    /*udq_cmd.d = PImodule(50, 0, udq_cmd.d, idq_cmd.d - idq.d, &idlasterr, 15, 0);
    udq_cmd.q = PImodule(50, 0, udq_cmd.q, idq_cmd.q - idq.q, &iqlasterr, 15, 0);
    
    R2toS2(udq_cmd, &ualbe_cmd, cosIn, sinIn);
    ualbeSVM(ualbe_cmd.al, ualbe_cmd.be, Ud, Tinv);*/
    
    /* SVM�������� */
    u_cmd = RAMP(VSpdramp, 0, spd_cmd, Voltlimit_H, Voltlimit_L);
    theta += 0.0000418879 * spd_cmd; // theta += 2 * pi * (spd_cmd / 30.0) * 0.0002; 
    if (theta > 6.2831852)  // 2 * pi = 6.2831852
      theta -= 6.2831852;
    ualbe_cmd.al = u_cmd * cosIn;
    ualbe_cmd.be = u_cmd * sinIn;
    ualbeSVM(ualbe_cmd.al, ualbe_cmd.be, Ud, Tinv); 
    //positionSVM(Tinv);
    
    /* �ȽϼĴ������� */
    PWM_WR_VAL2(PWMA, 0, -Tinv[0]);
    PWM_WR_VAL2(PWMA, 1, -Tinv[1]);
    PWM_WR_VAL2(PWMA, 2, -Tinv[2]);
    
    PWM_WR_VAL3(PWMA, 0, Tinv[0]);
    PWM_WR_VAL3(PWMA, 1, Tinv[1]);
    PWM_WR_VAL3(PWMA, 2, Tinv[2]);
    
    PWM_WR_STS_RF(PWMA, 0, TRUE);  // clear reload flag
    PWM_WR_MCTRL_LDOK(PWMA, TRUE);  // start PWMs (set load OK flags and run)
    
        /* PWM DA */
    FTM_WR_CnV_VAL(FTM3, 6, (uint16_t)(FTM3_MODULO * ((Tinv[0] - 2000) / 4000.0)));
    FTM_WR_CnV_VAL(FTM3, 7, (uint16_t)(FTM3_MODULO * ((Tinv[1] - 2000) / 4000.0)));
    //FTM_WR_CnV_VAL(FTM3, 7, (uint16_t)(FTM3_MODULO * ((idq.d + 300)/600.0)));
    //FTM_WR_CnV_VAL(FTM3, 6, (uint16_t)(FTM3_MODULO * ((idq.q + 300)/600.0)));
    //FTM_WR_CnV_VAL(FTM3, 7, (uint16_t)(FTM3_MODULO * ((iabc.a + 400)/800.0)));
    //FTM_WR_CnV_VAL(FTM3, 7, (uint16_t)(FTM3_MODULO * (sin(theta) + 1) / 2.0));
    FTM_WR_SYNC_SWSYNC(FTM3, TRUE);
 }

/******************************************************************************
@brief   PWMA �����ж�

@param   N/A

@return  N/A
******************************************************************************/
void PWMA_RERR_IRQHandler(void)
{
  GPIO_WR_PSOR(PTB, 1<<22);
}

/******************************************************************************
@brief   PIT �ж� -- ����ת�ټ�ת�ٸ���ֵ

@param   N/A

@return  N/A
******************************************************************************/
void PIT0_IRQHandler(void)
{
  PIT_WR_TFLG_TIF(PIT, 0, 1);
  
  speed = spdCal_M();  // ת�ټ���
  
  if (spd_cmd < spd_req)
  {
    spd_cmd = RAMP(spdramp, spd_cmd, 0.1, spdlimit_H, spdlimit_L);  // ת�ٸ���ֵ����
  }
  else if (spd_cmd > spd_req)
  {
    spd_cmd = RAMP(spdramp, spd_cmd, -0.1, spdlimit_H, spdlimit_L);  // ת�ٸ���ֵ����
  }
}