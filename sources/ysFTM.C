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
  /* 屏蔽输出 */
  FTM_WR_OUTMASK(FTM0, 0x3F);                     
    
  /* disable write protection */
  FTM_WR_MODE_WPDIS(FTM0, TRUE); 
    
  /* enable the counter */
  FTM_WR_MODE_FTMEN(FTM0, TRUE);
    
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
    
  /* initialize the channels output */
  FTM_WR_MODE_INIT(FTM0, TRUE);                                               
    
  /* set system clock as source for FTM0 (CLKS[1:0] = 01) */
  FTM_WR_SC_CLKS(FTM0, 0x01);
  
  // 使能输出
  FTM_WR_OUTMASK(FTM0, 0x00);     
  
  // 使能中断
  FTM_WR_SC_TOIE(FTM0, 0x01);
  
  /* enable & setup interrupts */
  NVIC_EnableIRQ(FTM0_IRQn);                                                  /* enable Interrupt */
  NVIC_SetPriority(FTM0_IRQn, 3);                                             /* set priority to interrupt */
}

/******************************************************************************
@brief   Initialization of FTM1—— method M

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
  
  /* 设置输入采样数；
     连续3个时钟认定为有效；
     在不为输入模式时更改 */
  FTM_WR_FILTER_CH0FVAL(FTM1, 0x03);
  FTM_WR_FILTER_CH1FVAL(FTM1, 0x03);
  
  /* initial setting of value registers to 0 % of duty cycle  */
  FTM_WR_CnV_VAL(FTM1, 0, 0);
  FTM_WR_CnV_VAL(FTM1, 1, 0);

  /* 设置成输入捕捉 */
  FTM_WR_CnSC_ELSA(FTM1, 0, TRUE); 
  FTM_WR_CnSC_ELSA(FTM1, 1, TRUE); 
  
  /* 光电解码器设置 */
  FTM_WR_QDCTRL_PHAFLTREN(FTM1, TRUE);  // 使能输入滤波
  FTM_WR_QDCTRL_PHBFLTREN(FTM1, TRUE); 
  FTM_WR_QDCTRL_QUADEN(FTM1, TRUE);  // 使能解码器
    
  /* initialize the channels output */
  FTM_WR_MODE_INIT(FTM1, TRUE);                                               
    
  /* set system clock as source for FTM1 (CLKS[1:0] = 01) */
  FTM_WR_SC_CLKS(FTM1, 0x01);
}

/******************************************************************************
@brief   FTM3 初始化 -- PWM DA输出

@param   N/A

@return  N/A
******************************************************************************/
void InitFTM3(void)
{
  /* Disable all channel 0-1 outputs using the OUTPUT MASK feature. */
  FTM_WR_OUTMASK(FTM3, 0xff);                     
    
  /* disable write protection for certain registers */
  FTM_WR_MODE_WPDIS(FTM3, TRUE); 
    
  /* enable the counter */
  FTM_WR_MODE_FTMEN(FTM3, TRUE);
    
  /* counter running in BDM mode */
  FTM_WR_CONF_BDMMODE(FTM3, 0x03);
  
  /* 使能PWM同步 */
  FTM_WR_COMBINE_SYNCEN3(FTM3, TRUE);
  
  /* set modulo register */
  FTM_WR_MOD(FTM3, (uint16_t)(FTM3_MODULO));
  
  /* set initial counting value */
  FTM_WR_CNTIN(FTM3, (uint16_t)(0));  //
        
  /* PWM update at counter in maximal value */
  FTM_WR_SYNC_CNTMAX(FTM3, TRUE);

  /* Edge-Aligned PWM 模式 */
  FTM_WR_CnSC_ELSB(FTM3, 6, TRUE); 
  FTM_WR_CnSC_ELSA(FTM3, 6, FALSE); 
  FTM_WR_CnSC_MSB(FTM3, 6, TRUE); 
  FTM_WR_CnSC_ELSB(FTM3, 7, TRUE); 
  FTM_WR_CnSC_ELSA(FTM3, 7, FALSE); 
  FTM_WR_CnSC_MSB(FTM3, 7, TRUE); 
  
  /* initial setting of value registers to 0 % of duty cycle  */
  FTM_WR_CnV_VAL(FTM3, 6, (uint16_t)(0));
  FTM_WR_CnV_VAL(FTM3, 7, (uint16_t)(0));
  
    /* set LOAD OK register */
  FTM_WR_PWMLOAD_LDOK(FTM3, TRUE);
    
  /* initialize the channels output */
  FTM_WR_MODE_INIT(FTM3, TRUE);                                               
    
  /* set system clock as source for FTM3 (CLKS[1:0] = 01) */
  FTM_WR_SC_CLKS(FTM3, 0x01);
  
  /* 输出使能 */
  FTM_WR_OUTMASK(FTM3, 0x00);  
}
