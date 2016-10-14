/******************************************************************************
| includes
|----------------------------------------------------------------------------*/
#include "fsl_device_registers.h"

/******************************************************************************
| defines
|----------------------------------------------------------------------------*/
#define FTM0_MODULO  (7400)  // 10khz PWM对应时钟周期
#define FTM1_MODULO  (60000)
#define FTM3_MODULO 3700  // FTM3计数上限
#define TRUE 1
#define FALSE 0

/******************************************************************************
| exported functions
|----------------------------------------------------------------------------*/
extern void InitFTM0();
extern void InitFTM1();
extern void InitFTM3();
