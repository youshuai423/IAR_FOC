/******************************************************************************
| includes                          
|----------------------------------------------------------------------------*/
#include "ysADC.h"

/******************************************************************************
| local variable definitions                          
|----------------------------------------------------------------------------*/

/******************************************************************************
| global variable definitions                          
|----------------------------------------------------------------------------*/

/******************************************************************************
@brief   Initialization of ADC

@param   N/A

@return  N/A
******************************************************************************/
void InitADC(void)
{
  /* loop parallel mode */
  ADC_WR_CTRL1_SMODE(ADC, 0x3);
  
  /* enable end-of-scan interrupt */
  //ADC_WR_CTRL1_EOSIE0(ADC, 1);
  
  /* enable hwardware triggering */
  //ADC_WR_CTRL1_SYNC0(ADC, 1);
                      
  /* start ADCA */
  //ADC_WR_CTRL1_STOP0(ADC, 0);
  //ADC_WR_CTRL2_STOP1(ADC, 1);
    
  /* input clock is 24.66MHz (148MHz fast peripheral clock divided by 6), 
     single ended */
  ADC_WR_CTRL2_DIV0(ADC, 0x005);
  
  /* parallel scans done independently */
  ADC_WR_CTRL2_SIMULT(ADC, 0);
  
  ADC_WR_CLIST1_SAMPLE0(ADC, 0);  // Udc: 0通道 -- ADC_CH0
  ADC_WR_CLIST1_SAMPLE1(ADC, 6);  // Ia:  1              6
  ADC_WR_CLIST1_SAMPLE2(ADC, 7);  // Ic:  2              7

  /* 使能采样通道0 1 2 */
  ADC_WR_SDIS(ADC, 0xFFF8);
        
  /* power-up ADCA and ADCB */
  ADC_WR_PWR_PD0(ADC, 0);
  //ADC_WR_PWR_PD1(ADC, 0);
 
  //ADC_WR_GC1_GAIN0(ADC, 2);
  //ADC_WR_GC1_GAIN1(ADC, 2);
  //ADC_WR_GC1_GAIN2(ADC, 2);
  
  /* 设置转换时钟频率 */
  ADC_WR_PWR2_SPEEDA(ADC, 3);
  
  /* enable & setup interrupt from ADC */
  // NVIC_EnableIRQ(ADCA_IRQn);                                                  /* enable Interrupt */
  // NVIC_SetPriority(ADCA_IRQn, 4);                                             /* set priority to interrupt */
}