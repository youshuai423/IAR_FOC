/******************************************************************************
| includes
|----------------------------------------------------------------------------*/
#include "fsl_device_registers.h"

/******************************************************************************
| defines
|----------------------------------------------------------------------------*/
#define PIT_MODULO  (2470000)  // 100ms��Ӧʱ����
#define TRUE 1

/******************************************************************************
| exported functions
|----------------------------------------------------------------------------*/
extern void InitPIT();