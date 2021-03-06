/******************************************************************************
| includes
|----------------------------------------------------------------------------*/
//#include "fsl_device_registers.h"

/******************************************************************************
| constants
|----------------------------------------------------------------------------*/
#define Rs 0.4
#define Ls 0.289368
#define Rr 3.3278
#define Lr 0.289368
#define Lm 0.27325
#define Tr 0.0869557

#define Ts 1e-4
#define period 4625

/******************************************************************************
| types
|----------------------------------------------------------------------------*/
#define pi 3.1415926

/******************************************************************************
| local functions prototypes
|----------------------------------------------------------------------------*/

/******************************************************************************
| exported functions
|----------------------------------------------------------------------------*/
/* Forward conversion */  
extern void S3toR2(double *abc, double *dq, double theta);
extern void S3toS2(double *abc, double *albe);
extern void S2toR2(double *albe, double *dq, double theta);

/* Backward conversion */  
extern void R2toS3(double *dq, double *abc, double theta);
extern void S2toS3(double *albe, double *abc);
extern void R2toS2(double *dq, double *albe, double theta);

/* calculate lamdar */  
extern double lamdarCal(double lamdar, double ism);
extern void lamdardqCal();
extern void lamdaralbeCal();

/* calculate position and speed */  
extern void wrCal();
extern void positonCal(double wr, double lamdar, double ist);

/* PI module */  
extern double PImodule(double Kp, double Ki, double err, double *Isum, double Uplim, double Downlim);
extern double Integrator(double paramin, double sum);

/* SVM */  
extern void positionSVM();
extern void ualbeSVM();
extern void udqSVM();

#ifdef __cplusplus
extern "C" {
#endif
  

#ifdef __cplusplus
}
#endif