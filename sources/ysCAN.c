/******************************************************************************
| includes                          
|----------------------------------------------------------------------------*/
#include "ysCAN.h"

/******************************************************************************
@brief   Initialization of PIT

@param   N/A

@return  N/A
******************************************************************************/
void InitCAN0(void)
{
  int i = 0;
  
  /* Init CAN0 */
  CAN_BWR_CTRL1_CLKSRC(CAN0, 1);  // CAN engine clock (PE) source is oscillator clock = 74MHz??????
  
  /* Check for low power mode*/
  if(CAN_BRD_MCR_LPMACK(CAN0))
  {
      /* Enable clock*/
      CAN_CLR_MCR(CAN0, CAN_MCR_MDIS_MASK);
      /* Wait until enabled*/
      while (CAN_BRD_MCR_LPMACK(CAN0)){}
  }
  
  /* Reset the FLEXCAN*/
  CAN_BWR_MCR_SOFTRST(CAN0, 0x1);
  /* Wait for reset cycle to complete*/
  while (CAN_BRD_MCR_SOFTRST(CAN0)){}
  /* Set Freeze, Halt*/
  CAN_BWR_MCR_FRZ(CAN0, 0x1);
  CAN_BWR_MCR_HALT(CAN0, 0x1);
  /* Wait for entering the freeze mode*/
  while (!(CAN_BRD_MCR_FRZACK(CAN0))){}
  
  /* Rx global mask*/
  CAN_WR_RXMGMASK(CAN0, CAN_ID_EXT(CAN_RXMGMASK_MG_MASK));
  /* Rx reg 14 mask*/
  CAN_WR_RX14MASK(CAN0, CAN_ID_EXT(CAN_RX14MASK_RX14M_MASK));
  /* Rx reg 15 mask*/
  CAN_WR_RX15MASK(CAN0, CAN_ID_EXT(CAN_RX15MASK_RX15M_MASK));
  /* Disable all MB interrupts*/
  CAN_WR_IMASK1(CAN0, 0x0);

  /* Set the maximum number of MBs*/
  CAN_BWR_MCR_MAXMB(CAN0, 1);
  /* Initialize all message buffers as inactive*/
  for (i = 0; i < 16; i++)
  {
      CAN0->MB[i].CS = 0x0;
      CAN0->MB[i].ID = 0x0;
      CAN0->MB[i].WORD0 = 0x0;
      CAN0->MB[i].WORD1 = 0x0;
  }
  
  //CAN_BWR_MCR_SUPV(CAN0, 0x0);
  CAN_BWR_CTRL1_LPB(CAN0, 0x1);
  
  /* Set FlexCAN time segments*/
  CAN_CLR_CTRL1(CAN0, (CAN_CTRL1_PROPSEG_MASK | CAN_CTRL1_PSEG2_MASK |
                              CAN_CTRL1_PSEG1_MASK | CAN_CTRL1_PRESDIV_MASK) |
                              CAN_CTRL1_RJW_MASK);
  CAN_SET_CTRL1(CAN0, (CAN_CTRL1_PROPSEG(4) |
                              CAN_CTRL1_PSEG2(3) |
                              CAN_CTRL1_PSEG1(5) |
                              CAN_CTRL1_PRESDIV(36) |
                              CAN_CTRL1_RJW(2)));
  
  /* Enable Global RX masking*/
  CAN_BWR_MCR_IRMQ(CAN0, 0x0);
  
  /////////////////////////////////////////////////////////
  /*CAN_WR_MCR_HALT(CAN0, 1);
  CAN_WR_MCR_IRMQ(CAN0, 0);  // disable individual RX mask
  CAN_WR_MCR_WRNEN(CAN0, 0);  // disable warning interrupts
  CAN_WR_MCR_SRXDIS(CAN0, 1);  // disable self reception
  CAN_WR_MCR_RFEN(CAN0, 0);  // RX FIFO disabled
  CAN_WR_MCR_AEN(CAN0, 1);  // abort enabled
  CAN_WR_MCR_LPRIOEN(CAN0, 0);  // local priority disabled
  CAN_WR_MCR_FRZ(CAN0, 1);  // enabled to enter freeze mode
  CAN_WR_MCR_WAKMSK(CAN0, 0);  // wake up interrupt is disabled
  CAN_WR_MCR_SUPV(CAN0, 1);  // supervisor mode, restricted access to some registers
  CAN_WR_MCR_SLFWAK(CAN0, 0);  // self wake up disabled
  CAN_WR_MCR_WAKSRC(CAN0, 1);  // filtered RX input
  CAN_WR_MCR_DOZE(CAN0, 0);  // not enabled to enter low-power mode when doze mode is requested
  CAN_WR_MCR_DMA(CAN0, 0);  // DMA for RX FIFO disabled
  CAN_WR_MCR_IDAM(CAN0, 0);  // ???????
  CAN_WR_MCR_MAXMB(CAN0, 0x1);  // 2MB will take part in the matching and arbitration processes.
  CAN_WR_MCR_SOFTRST(CAN0, 1);  // soft reset 
  
  while(CAN_RD_MCR_SOFTRST(CAN0) == 1) {}  // wait until soft reset complete
  while(CAN_RD_MCR_NOTRDY(CAN0) == 0) {} // check if in freeze mode
  
  CAN_WR_CTRL1_PROPSEG(CAN0, 2);  // propagation segment time = 2TQ
  CAN_WR_CTRL1_PSEG1(CAN0, 7);  // phase buffer segment 1 = 2TQ
  CAN_WR_CTRL1_PSEG2(CAN0, 3);  // phase buffer segment 2 = 2TQ
  CAN_WR_CTRL1_RJW(CAN0, 2);  // resync jump width = 2TQ
  CAN_WR_CTRL1_PRESDIV(CAN0, 36);  // Sclock frequency = PE clock frequency / (PROPSEG + 1) = 2m
  CAN_WR_CTRL1_LBUF(CAN0, 0);  // buffer with highest priority is transmitted first
  CAN_WR_CTRL1_BOFFMSK(CAN0, 0); // bus off interrupt disabled
  CAN_WR_CTRL1_ERRMSK(CAN0, 0);  // error interrupt disabled
  CAN_WR_CTRL1_CLKSRC(CAN0, 1);  // CAN engine clock (PE) source is peripheral clock = 74MHz
  CAN_WR_CTRL1_LPB(CAN0, 0);  // loop back disabled
  CAN_WR_CTRL1_TWRNMSK(CAN0, 0);  // TX warning interrupt disabled
  CAN_WR_CTRL1_RWRNMSK(CAN0, 0);  // RX warning interrupt disabled
  CAN_WR_CTRL1_SMP(CAN0, 1);  // three samples is used to determine the bit value
  CAN_WR_CTRL1_BOFFREC(CAN0, 0);  // auto recovering from bus off state
  CAN_WR_CTRL1_TSYN(CAN0, 0);  // timer sync feature disabled
  CAN_WR_CTRL1_LOM(CAN0, 0);  // listen only mode is diactivated
  
  CAN_WR_RXMGMASK(CAN0, 0xFFFFFFFF);  // the corresponding bit is checked
  CAN_WR_RX14MASK(CAN0, 0xFFFFFFFF);
  CAN_WR_RX15MASK(CAN0, 0xFFFFFFFF);
  
  CAN_WR_ECR_RXERRCNT(CAN0, 0);  // clear receive error counter
  CAN_WR_ECR_TXERRCNT(CAN0, 0);  // clear transmit error counter
  
  CAN_WR_ESR1_ERROVR(CAN0, 1);  // clear error overrun flag
  CAN_WR_ESR1_BOFFDONEINT(CAN0, 1);  // clear bus off done flag
  CAN_WR_ESR1_TWRNINT(CAN0, 1);  // clear TX warning interrupt flag
  CAN_WR_ESR1_RWRNINT(CAN0, 1);  // clear RX warning interrupt flag
  CAN_WR_ESR1_BOFFINT(CAN0, 1);  // clear bus off interrupt flag
  CAN_WR_ESR1_ERRINT(CAN0, 1);  // clear error interrupt flag
  CAN_WR_ESR1_WAKINT(CAN0, 1);  // clar wake up interrupt flag
  
  CAN_WR_IMASK1(CAN0, 0);  // disable all MB buffer interrupt
  
  CAN_WR_IFLAG1_BUF31TO8I(CAN0, 0xffffff);  // clear buffer 31-8 interrupt flag
  CAN_WR_IFLAG1_BUF7I(CAN0, 1);
  CAN_WR_IFLAG1_BUF6I(CAN0, 1);
  CAN_WR_IFLAG1_BUF5I(CAN0, 1);
  CAN_WR_IFLAG1_BUF4TO1I(CAN0, 0xf);
  CAN_WR_IFLAG1_BUF0I(CAN0, 1);
  
  CAN_WR_CTRL2_BOFFDONEMSK(CAN0, 0);  // bus off done interrupt disabled
  CAN_WR_CTRL2_RFFN(CAN0, 0);  // number of Rx FIFO filters
  CAN_WR_CTRL2_TASD(CAN0, 0);  // Tx arbitration start delay = 0
  CAN_WR_CTRL2_MRP(CAN0, 0);  // Matching starts from Rx FIFO and continues on mailboxes
  CAN_WR_CTRL2_RRS(CAN0, 0);  // remote response frame is generated
  CAN_WR_CTRL2_EACEN(CAN0, 1);  // compare both Rx mailbox filter's IDE and RTR bit
  
  CAN_WR_RXFGMASK(CAN0, 0xffffffff);  // the corresponding bit in the filter is checked
  
  CAN_WR_CBT_BTF(CAN0, 0);  // extended bit time definitions disabled
  
  CAN_WR_RXIMR(CAN0, 0, 0xffffffff);  // the corresponding bit in the filter is checked
  CAN_WR_RXIMR(CAN0, 1, 0xffffffff);  // the corresponding bit in the filter is checked
  CAN_WR_RXIMR(CAN0, 2, 0xffffffff);  // the corresponding bit in the filter is checked
  CAN_WR_RXIMR(CAN0, 3, 0xffffffff);  // the corresponding bit in the filter is checked
  CAN_WR_RXIMR(CAN0, 4, 0xffffffff);  // the corresponding bit in the filter is checked
  CAN_WR_RXIMR(CAN0, 5, 0xffffffff);  // the corresponding bit in the filter is checked
  CAN_WR_RXIMR(CAN0, 6, 0xffffffff);  // the corresponding bit in the filter is checked
  CAN_WR_RXIMR(CAN0, 7, 0xffffffff);  // the corresponding bit in the filter is checked
  CAN_WR_RXIMR(CAN0, 8, 0xffffffff);  // the corresponding bit in the filter is checked
  CAN_WR_RXIMR(CAN0, 9, 0xffffffff);  // the corresponding bit in the filter is checked
  CAN_WR_RXIMR(CAN0, 10, 0xffffffff);  // the corresponding bit in the filter is checked
  CAN_WR_RXIMR(CAN0, 11, 0xffffffff);  // the corresponding bit in the filter is checked
  CAN_WR_RXIMR(CAN0, 12, 0xffffffff);  // the corresponding bit in the filter is checked
  CAN_WR_RXIMR(CAN0, 13, 0xffffffff);  // the corresponding bit in the filter is checked
  CAN_WR_RXIMR(CAN0, 14, 0xffffffff);  // the corresponding bit in the filter is checked
  CAN_WR_RXIMR(CAN0, 15, 0xffffffff);  // the corresponding bit in the filter is checked  
  
  CAN_WR_CS(CAN0, 0, 0x00000000);
  CAN_WR_CS(CAN0, 1, 0x00000000);
  CAN_WR_CS(CAN0, 2, 0x00000000);
  CAN_WR_CS(CAN0, 3, 0x00000000);
  CAN_WR_CS(CAN0, 4, 0x00000000);
  CAN_WR_CS(CAN0, 5, 0x00000000);
  CAN_WR_CS(CAN0, 6, 0x00000000);
  CAN_WR_CS(CAN0, 7, 0x00000000);
  CAN_WR_CS(CAN0, 8, 0x00000000);
  CAN_WR_CS(CAN0, 9, 0x00000000);
  CAN_WR_CS(CAN0, 10, 0x00000000);
  CAN_WR_CS(CAN0, 11, 0x00000000);
  CAN_WR_CS(CAN0, 12, 0x00000000);
  CAN_WR_CS(CAN0, 13, 0x00000000);
  CAN_WR_CS(CAN0, 14, 0x00000000);
  CAN_WR_CS(CAN0, 15, 0x00000000);
  
  CAN_WR_ID(CAN0, 0, 0x00000000);
  CAN_WR_ID(CAN0, 1, 0x00000000);
  CAN_WR_ID(CAN0, 2, 0x00000000);
  CAN_WR_ID(CAN0, 3, 0x00000000);
  CAN_WR_ID(CAN0, 4, 0x00000000);
  CAN_WR_ID(CAN0, 5, 0x00000000);
  CAN_WR_ID(CAN0, 6, 0x00000000);
  CAN_WR_ID(CAN0, 7, 0x00000000);
  CAN_WR_ID(CAN0, 8, 0x00000000);
  CAN_WR_ID(CAN0, 9, 0x00000000);
  CAN_WR_ID(CAN0, 10, 0x00000000);
  CAN_WR_ID(CAN0, 11, 0x00000000);
  CAN_WR_ID(CAN0, 12, 0x00000000);
  CAN_WR_ID(CAN0, 13, 0x00000000);
  CAN_WR_ID(CAN0, 14, 0x00000000);
  CAN_WR_ID(CAN0, 15, 0x00000000);
  
  CAN_WR_WORD0(CAN0, 0, 0x00000000);
  CAN_WR_WORD0(CAN0, 1, 0x00000000);
  CAN_WR_WORD0(CAN0, 2, 0x00000000);
  CAN_WR_WORD0(CAN0, 3, 0x00000000);
  CAN_WR_WORD0(CAN0, 4, 0x00000000);
  CAN_WR_WORD0(CAN0, 5, 0x00000000);
  CAN_WR_WORD0(CAN0, 6, 0x00000000);
  CAN_WR_WORD0(CAN0, 7, 0x00000000);
  CAN_WR_WORD0(CAN0, 8, 0x00000000);
  CAN_WR_WORD0(CAN0, 9, 0x00000000);
  CAN_WR_WORD0(CAN0, 10, 0x00000000);
  CAN_WR_WORD0(CAN0, 11, 0x00000000);
  CAN_WR_WORD0(CAN0, 12, 0x00000000);
  CAN_WR_WORD0(CAN0, 13, 0x00000000);
  CAN_WR_WORD0(CAN0, 14, 0x00000000);
  CAN_WR_WORD0(CAN0, 15, 0x00000000);
  
  CAN_WR_WORD1(CAN0, 0, 0x00000000);
  CAN_WR_WORD1(CAN0, 1, 0x00000000);
  CAN_WR_WORD1(CAN0, 2, 0x00000000);
  CAN_WR_WORD1(CAN0, 3, 0x00000000);
  CAN_WR_WORD1(CAN0, 4, 0x00000000);
  CAN_WR_WORD1(CAN0, 5, 0x00000000);
  CAN_WR_WORD1(CAN0, 6, 0x00000000);
  CAN_WR_WORD1(CAN0, 7, 0x00000000);
  CAN_WR_WORD1(CAN0, 8, 0x00000000);
  CAN_WR_WORD1(CAN0, 9, 0x00000000);
  CAN_WR_WORD1(CAN0, 10, 0x00000000);
  CAN_WR_WORD1(CAN0, 11, 0x00000000);
  CAN_WR_WORD1(CAN0, 12, 0x00000000);
  CAN_WR_WORD1(CAN0, 13, 0x00000000);
  CAN_WR_WORD1(CAN0, 14, 0x00000000);
  CAN_WR_WORD1(CAN0, 15, 0x00000000);
  
  /* CAN_WR_CS_CODE(CAN0, 0, 0x0);
  CAN_WR_CS_CODE(CAN0, 1, 0x0);
  CAN_WR_CS_CODE(CAN0, 2, 0x0);
  CAN_WR_CS_CODE(CAN0, 3, 0x0);
  CAN_WR_CS_CODE(CAN0, 4, 0x0);
  CAN_WR_CS_CODE(CAN0, 5, 0x0);
  CAN_WR_CS_CODE(CAN0, 6, 0x0);
  CAN_WR_CS_CODE(CAN0, 7, 0x0);
  CAN_WR_CS_CODE(CAN0, 8, 0x0);
  CAN_WR_CS_CODE(CAN0, 9, 0x0);
  CAN_WR_CS_CODE(CAN0, 10, 0x0);
  CAN_WR_CS_CODE(CAN0, 11, 0x0);
  CAN_WR_CS_CODE(CAN0, 12, 0x0);
  CAN_WR_CS_CODE(CAN0, 13, 0x0);
  CAN_WR_CS_CODE(CAN0, 14, 0x0);
  CAN_WR_CS_CODE(CAN0, 15, 0x0); */
  
  /* ======================set rx mailbox1============================*/
  /* Make sure IDE and SRR are not set*/
  CAN0->MB[1].CS &= ~(CAN_CS_IDE_MASK | CAN_CS_SRR_MASK);

  /* Set the length of data in bytes*/
  CAN0->MB[1].CS &= ~CAN_CS_DLC_MASK;
  CAN0->MB[1].CS |= (1) << CAN_CS_DLC_SHIFT;

  /* ID[28-18]*/
  CAN0->MB[1].ID &= ~CAN_ID_STD_MASK;
  CAN0->MB[1].ID |= CAN_ID_STD(0x2AA);

  /* Set MB CODE*/
  CAN0->MB[1].CS &= ~CAN_CS_CODE_MASK;
  CAN0->MB[1].CS |= CAN_CS_CODE(0x0);
  CAN0->MB[1].CS &= ~CAN_CS_CODE_MASK;
  CAN0->MB[1].CS |= CAN_CS_CODE(0x4);
  
  CAN_BWR_MCR_HALT(CAN0, 0x0);
  CAN_BWR_MCR_FRZ(CAN0, 0x0);

  /* Wait till exit freeze mode*/
  while (CAN_BRD_MCR_FRZACK(CAN0)){}
  
  /* enable & setup interrupts */
  //NVIC_EnableIRQ(PIT0_IRQn);                                                  /* enable Interrupt */
  //NVIC_SetPriority(PIT0_IRQn, 3);                                             /* set priority to interrupt */
}

void sendFrame(uint32_t MBH, uint32_t MBL, uint32_t ID, uint16_t MBn)
{
  int i = 0;
  
  //CAN0->MB[0].WORD0 = 0x5;

  /* ID[28-18]*/
  CAN0->MB[0].ID &= ~CAN_ID_STD_MASK;
  CAN0->MB[0].ID |= CAN_ID_STD(0x2AA);

  /* make sure IDE and SRR are not set*/
  CAN0->MB[0].CS &= ~(CAN_CS_IDE_MASK | CAN_CS_SRR_MASK);

  /* Set the length of data in bytes*/
  CAN0->MB[0].CS &= ~CAN_CS_DLC_MASK;
  CAN0->MB[0].CS |= (1) << CAN_CS_DLC_SHIFT;

  /* Reset MB CODE*/
  CAN0->MB[0].CS &= ~CAN_CS_CODE_MASK;

  /* Set the code*/
  CAN0->MB[0].CS |= CAN_CS_CODE(0x8);
  
  CAN0->MB[0].WORD0 = 0x05000000;
  
  /* Reset MB CODE*/
  CAN0->MB[0].CS &= ~CAN_CS_CODE_MASK;

  /* Set the code*/
  CAN0->MB[0].CS |= CAN_CS_CODE(0xC);
        
  /* 1. clear corresponding interrupt *
  while((CAN_RD_IFLAG1(CAN0) & (1<<MBn)) == (1<<MBn))
  {
    CAN_WR_IFLAG1(CAN0, (CAN_RD_IFLAG1(CAN0) | (1<<MBn)));
  }
  
  /* 2. abort transmission if MB is active *
  if((CAN_RD_CS_CODE(CAN0, MBn) & 0x1) == 0x1)
  //if(CAN_RD_CS_CODE(CAN0, MBn) == 0xC)
  {
    CAN_WR_CS_CODE(CAN0, MBn, 0x9);
    while((CAN_RD_IFLAG1(CAN0) & (1<<MBn)) != (1<<MBn)){}
    CAN_WR_IFLAG1(CAN0, (CAN_RD_IFLAG1(CAN0) | (1<<MBn)));
  }
  
  /* 3. write the ID word *
  //CAN_WR_ID_STD(CAN0, MBn, 0x2AA);
  CAN_WR_ID_STD(CAN0, MBn, ID);
  
  /* 4. write the data word *
  CAN_WR_WORD0(CAN0, MBn, MBL);
  CAN_WR_WORD1(CAN0, MBn, MBH);
  
  /* 5. write the DLC, Control, CODE to actiate MB *
  CAN_WR_CS_DLC(CAN0, MBn, 8);
  CAN_WR_CS_SRR(CAN0, MBn, 0);
  CAN_WR_CS_IDE(CAN0, MBn, 0);
  CAN_WR_CS_RTR(CAN0, MBn, 0);
  CAN_WR_CS_CODE(CAN0, MBn, 0xC); */
}