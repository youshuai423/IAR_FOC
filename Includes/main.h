/*******************************************************************************
* @file     main.h
*******************************************************************************/
#ifndef _MAIN_H_
#define _MAIN_H_

/******************************************************************************
| includes
|----------------------------------------------------------------------------*/
//#include "mcdrv_hvp-kv46f.h"
#include "fsl_device_registers.h"
#include "ysPORT.h"
#include "ysADC.h"
#include "ysPWM.h"

/******************************************************************************
| constants
|----------------------------------------------------------------------------*/

/******************************************************************************
| types
|----------------------------------------------------------------------------*/

/******************************************************************************
| local functions prototypes
|----------------------------------------------------------------------------*/
double roundn(double);  // 截断小数点后位数
void SVMUdq(double Ud, double Uq, unsigned int *Tinv);  // 给定Ud、Uq的SVM

/******************************************************************************
| exported functions
|----------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
  

#ifdef __cplusplus
}
#endif

#endif /* _MAIN_H_ */
/*
 *######################################################################
 *                           End of File
 *######################################################################
*/