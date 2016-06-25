/******************************************************************************
| includes
|----------------------------------------------------------------------------*/
//#include "fsl_device_registers.h"

/******************************************************************************
| constants
|----------------------------------------------------------------------------*/
// IM parameters
#define Rs 0.4
#define Ls 0.289368
#define Rr 3.3278
#define Lr 0.289368
#define Lm 0.27325
#define Tr 0.0869557

// period
#define Ts 1e-4
#define period 4625

// command values
#define nset 400
#define idset 2

// PI parameters
#define ud_Kp 0
#define ud_Ki 0
#define ud_Uplimit 0
#define ud_Downlimit 0

#define iqset_Kp 0
#define iqset_Ki 0
#define iqset_Uplimit 0
#define iqset_Downlimit 0

#define uq_Kp 0
#define uq_Ki 0
#define uq_Uplimit 0
#define uq_Downlimit 0

// auxiliary
#define pi 3.1415926
#define M 0.95  // modulation factor
#define Ud 60

/******************************************************************************
| types
|----------------------------------------------------------------------------*/
typedef struct
{
  double a, b, c;
} PHASE_ABC;

typedef struct
{
  double al, be;
} PHASE_ALBE;

typedef struct
{
  double d,q;
} PHASE_DQ;

extern double n;
extern double wr;
extern double iqset;

extern double ud_Isum;
extern double uq_Isum;
extern double iqset_Isum;
extern int period_count;

/******************************************************************************
| local functions prototypes
|----------------------------------------------------------------------------*/
double roundn(double input, int digit);

/******************************************************************************
| exported functions
|----------------------------------------------------------------------------*/
/* Forward conversion */  
extern void S3toR2(PHASE_ABC *abc, PHASE_DQ *dq, double theta);
extern void S3toS2(PHASE_ABC *abc, PHASE_ALBE *albe);
extern void S2toR2(PHASE_ALBE *albe, PHASE_DQ *dq, double theta);

/* Backward conversion */  
extern void R2toS3(PHASE_DQ *dq, PHASE_ABC *abc, double theta);
extern void S2toS3(PHASE_ALBE *albe, PHASE_ABC *abc);
extern void R2toS2(PHASE_DQ *dq, PHASE_ALBE *albe, double theta);

/* calculate lamdar */  
extern double lamdarCal(double lamdar, double ism);
extern void lamdardqCal();
extern void lamdaralbeCal();

/* calculate position and speed */  
extern void wrCal();
extern double positonCal(double wr, double lamdar, double ist, double theta);

/* PI module */  
extern double PImodule(double Kp, double Ki, double err, double *Isum, double Uplim, double Downlim);
extern double Integrator(double paramin, double sum, double ts);

/* SVM */  
extern void positionSVM();
extern void ualbeSVM(double Ual, double Ube, unsigned int *Tinv);
extern void udqSVM();

#ifdef __cplusplus
extern "C" {
#endif
  

#ifdef __cplusplus
}
#endif