/******************************************************************************
| includes                          
|----------------------------------------------------------------------------*/
#include "ysUART.h"
#define hhh 2
/******************************************************************************
@brief   Initialization of PIT

@param   N/A

@return  N/A
******************************************************************************/
void InitUART1(void)
{
  /* Baud rate settings */
  UART_WR_BDH_LBKDIE(UART1, 0);
  UART_WR_BDH_RXEDGIE(UART1, 0);
  UART_WR_BDH_SBNS(UART1, 1);
  UART_WR_BDH_SBR(UART1, 0x0);
  UART_WR_BDL(UART1, 0x28);
  UART_BWR_C4_BRFA(UART1, 0x0);
  
  //UART_BWR_BDH_SBR(base, (uint8_t)(sbr >> 8));
  //UART_WR_BDL(base, (uint8_t)sbr);  
  //UART_BWR_C4_BRFA(base, brfa);
  
  UART_WR_C1_LOOPS(UART1, 0);
  UART_WR_C1_UARTSWAI(UART1, 0);
  //UART_WR_C1_RSRC(UART1, 0);
  UART_WR_C1_M(UART1, 0);
  UART_WR_C1_WAKE(UART1, 0);
  UART_WR_C1_ILT(UART1, 0);  // ????????????????
  UART_WR_C1_PE(UART1, 0);
  UART_WR_C1_PT(UART1, 0);  
  //UART_BWR_C1_M(base, bitCountPerChar);
  //UART_BWR_C1_PE(base, parityMode >> 1U);
  //UART_BWR_C1_PT(base, parityMode & 1U);
  
  UART_WR_C2_TIE(UART1, 0);
  UART_WR_C2_TCIE(UART1, 0);
  UART_WR_C2_RIE(UART1, 0);
  UART_WR_C2_ILIE(UART1, 0);
  UART_WR_C2_RWU(UART1, 0);  // ???????????
  UART_WR_C2_SBK(UART1, 0);
  //UART_BWR_C2_TE(base, 1U);
  //UART_BWR_C2_TE(base, 0U);
  
  UART_WR_S2_MSBF(UART1, 0);  
  UART_WR_S2_RXINV(UART1, 0); 
  UART_WR_S2_RWUID(UART1, 0);
  UART_WR_S2_BRK13(UART1, 0); 
  UART_WR_S2_LBKDE(UART1, 0); 
  
  //UART_WR_C3_TXDIR(UART1, 0); 
  UART_WR_C3_TXINV(UART1, 0); 
  UART_WR_C3_ORIE(UART1, 0); 
  UART_WR_C3_NEIE(UART1, 0); 
  UART_WR_C3_FEIE(UART1, 0); 
  UART_WR_C3_PEIE(UART1, 0); 
  
  UART_WR_MA1(UART1, 0);
  UART_WR_MA2(UART1, 0);
  
  UART_WR_C4_MAEN1(UART1, 0);
  UART_WR_C4_MAEN2(UART1, 0);
  UART_WR_C4_M10(UART1, 0);
  
  UART_WR_C5_TDMAS(UART1, 0);
  UART_WR_C5_RDMAS(UART1, 0);
  UART_WR_C5_LBKDDMAS(UART1, 0);
  
  UART_WR_MODEM_RXRTSE(UART1, 0);
  UART_WR_MODEM_TXRTSPOL(UART1, 0);
  UART_WR_MODEM_TXRTSE(UART1, 0);
  UART_WR_MODEM_TXCTSE(UART1, 0);
  
  UART_WR_PFIFO_TXFE(UART1, 0);
  UART_WR_PFIFO_RXFE(UART1, 0);
  
  //UART_WR_CFIFO_TXFLUSH(UART1, 0);
  //UART_WR_CFIFO_RXFLUSH(UART1, 0);
  UART_WR_CFIFO_RXOFE(UART1, 0);
  UART_WR_CFIFO_TXOFE(UART1, 0);
  UART_WR_CFIFO_RXUFE(UART1, 0);
  
  UART_WR_TWFIFO(UART1, 0x0);
  UART_WR_RWFIFO(UART1, 0x0);
  
  UART_WR_C2_TE(UART1, 1);
  UART_WR_C2_RE(UART1, 0);
}

void uartSend(int8_t a)
{
  while(UART_RD_S1_TC(UART1) != 1) {}
  UART_WR_D(UART1, a);
}

void uartNumber(int8_t* msg, int maxIndex)
{
  int i = 0;

  for (i = 0; i <= maxIndex; i++)
  {
    uartSend(msg[i]);
  }
}

int8_t FtoInt8(float fin, int fractDigit)
{
  switch(fractDigit)
  {
  case 1: return (fin * 10); break;
  case 2: return (fin * 100); break;
  case 3: return (fin * 1000); break;
  default: return (fin); break;
  }  
}

int16_t FtoInt16(float fin, int fractDigit)
{
  switch(fractDigit)
  {
  case 1: return (fin * 10); break;
  case 2: return (fin * 100); break;
  case 3: return (fin * 1000); break;
  case 4: return (fin * 10000); break;
  case 5: return (fin * 100000); break;
  default: return (fin); break;
  }
}
