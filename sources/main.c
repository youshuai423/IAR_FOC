/******************************************************************************
| includes                          
|----------------------------------------------------------------------------*/
#include "main.h"
#include "math.h"

/******************************************************************************
| local variable definitions                          
|----------------------------------------------------------------------------*/
uint16_t Tinv[3] = {0, 0, 0};  // 三相对应PWM寄存器比较值

int count = 0;
#define spdramp 50

/******************************************************************************
@brief   Main

@param   N/A

@return  N/A
******************************************************************************/
void main(void)
{
  /* disable all interrupts before peripherals are initialized */
  __disable_irq();
  
  /* init application ports */  
  InitPORT(); 
  InitPWM();   
  //InitFTM0();
  InitFTM1();  // 编码器控制
  InitFTM3();  // PWM DA
  InitADC();
  InitPIT();  // 计算转速和转速给定值
 
  int i = 0;
  for(i = 0; i < 1000; i++){};  // 等待ADC模块稳定
  ADC_WR_CTRL1_START0(ADC, 1); 
  
  /* enable interrupts  */
  __enable_irq();

  /* infinite loop */
  while(1){}
}

/******************************************************************************
@brief   PWMA 中断 -- FOC控制

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
@brief   PWMA 中断 -- V/f开环控制

@param   N/A

@return  N/A
******************************************************************************/
void PWMA_RELOAD0_IRQHandler(void)
{   
      /* 读取电流采样 */
    iabc.a = ADC_RD_RSLT_RSLT(ADC, 1);
    iabc.c = ADC_RD_RSLT_RSLT(ADC, 2);
    
    /* SVM开环计算 */
    positionSVM(Tinv);
    
    /* 比较寄存器配置 */
    PWM_WR_VAL2(PWMA, 0, -Tinv[0]);
    PWM_WR_VAL2(PWMA, 1, -Tinv[1]);
    PWM_WR_VAL2(PWMA, 2, -Tinv[2]);
    
    PWM_WR_VAL3(PWMA, 0, Tinv[0]);
    PWM_WR_VAL3(PWMA, 1, Tinv[1]);
    PWM_WR_VAL3(PWMA, 2, Tinv[2]);
    
    PWM_WR_STS_RF(PWMA, 0, TRUE);  // clear reload flag
    PWM_WR_MCTRL_LDOK(PWMA, TRUE);  // start PWMs (set load OK flags and run)
    
        /* PWM DA */
    //FTM_WR_CnV_VAL(FTM3, 7, (uint16_t)(FTM3_MODULO * ((IU - 1500)/1000.0)));
    //FTM_WR_CnV_VAL(FTM3, 7, (uint16_t)(FTM3_MODULO * (sin(theta) + 1) / 2.0));
    FTM_WR_CnV_VAL(FTM3, 7, (uint16_t)(FTM3_MODULO * count / 1000));
    count++;
    if (count >= 1000)
    {
       count -= 1000;
       
    }
    FTM_WR_SYNC_SWSYNC(FTM3, TRUE);  
    GPIO_WR_PTOR(PTB, 1<<22);
 }

/******************************************************************************
@brief   PWMA 错误中断

@param   N/A

@return  N/A
******************************************************************************/
void PWMA_RERR_IRQHandler(void)
{
  GPIO_WR_PSOR(PTB, 1<<22);
}

/******************************************************************************
@brief   PIT 中断 -- 计算转速及转速给定值

@param   N/A

@return  N/A
******************************************************************************/
void PIT0_IRQHandler(void)
{
  PIT_WR_TFLG_TIF(PIT, 0, 1);
  
  speed = spdCal_M();  // 转速计算
  
  if (spd_cmd < spd_req)
  {
    spd_cmd = RAMP(spdramp, spd_cmd, 0.1, spdlimit_H, spdlimit_L);  // 转速给定值计算
  }
}