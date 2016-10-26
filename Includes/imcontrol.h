/******************************************************************************
| includes
|----------------------------------------------------------------------------*/
#include "fsl_device_registers.h"
#include "math.h"
#include "ysFTM.h"
#include "ysPWM.h"

/******************************************************************************
| defines
|----------------------------------------------------------------------------*/
/* IM parameters */
#define Rs 0.4
#define Ls 0.289368
#define Rr 3.3278
#define Lr 0.289368
#define Lm 0.27325
#define Tr 0.0869557
#define np 2

/* control period */
#define Ts 1e-4
#define period M1_PWM_MODULO / 2
//#define period M1_FTM1_MODULO

/* lamdar��Сֵ���� */
#define lamdarlimit_L 0.01

/* PI parameters */
  // id�ջ�
#define ud_Kp 6
#define ud_Ki 0
#define udlimit_H 60
#define udlimit_L -60

  // �ٶȱջ�
#define spdthd 400  // speed threshold
#define iq_Kp1 2
#define iq_Ki1 0
#define iq_Kp2 1
#define iq_Ki2 20
#define iqlimit_H 0
#define iqlimit_L 0

  // iq�ջ�
#define uq_Kp 3
#define uq_Ki 10
#define uqlimit_H 60
#define uqlimit_L -60

/* speed ramp */
#define spdramp 60  // б��
#define spdlimit_H 450  // ת������
#define spdlimit_L 0  // ת������

/* V/spd curve */
#define VSpdramp 0.024  // б��
#define Voltlimit_H 40  // ��ѹ����
#define Voltlimit_L 5  // ��ѹ����

/* auxiliary */
#define pi 3.1415926
#define Z 64  // �����������
#define digit 1e6  // roundn����

/******************************************************************************
| types
|----------------------------------------------------------------------------*/
/* ��ֹ��������ϵ */
typedef struct
{
  double a, b, c;
} PHASE_ABC;

/* ��ֹ��������ϵ */
typedef struct
{
  double al, be;
} PHASE_ALBE;

/* ��ת����ϵ */
typedef struct
{
  double d,q;
} PHASE_DQ;

/******************************************************************************
| global variables
|----------------------------------------------------------------------------*/
/* �۲�ֵ */
  // ��ѹ
extern double Ud;
extern PHASE_ABC uabc;
extern PHASE_ALBE ualbe;
extern PHASE_DQ udq;
  // ����
extern PHASE_ABC iabc;
extern PHASE_ALBE ialbe;
extern PHASE_DQ idq;
  // ����
extern double lamdar;
extern PHASE_ALBE lamdaralbe;
extern double theta;
  // ת��
extern double speed;

/* ����ֵ */
  // ��ѹ
extern double u_cmd;
extern PHASE_ALBE ualbe_cmd;
extern PHASE_DQ udq_cmd;
  // ����
extern PHASE_DQ idq_cmd;
  // ת��
extern double spd_cmd;  // ת�ٸ���
extern double spd_req;  // ת���趨

/* PI ���� */
extern double idlasterr;
extern double iqlasterr;
extern double spdlasterr;

/******************************************************************************
| local functions prototypes
|----------------------------------------------------------------------------*/

/******************************************************************************
| exported functions
|----------------------------------------------------------------------------*/
/* Forward conversion */  
extern void S3toR2(PHASE_ABC abc, PHASE_DQ *dq, double theta);
extern void S3toS2(PHASE_ABC abc, PHASE_ALBE *albe);
extern void S2toR2(PHASE_ALBE albe, PHASE_DQ *dq, double cosIn, double sinIn);

/* Backward conversion */  
extern void R2toS3(PHASE_DQ dq, PHASE_ABC *abc, double theta);
extern void S2toS3(PHASE_ALBE albe, PHASE_ABC *abc);
extern void R2toS2(PHASE_DQ dq, PHASE_ALBE *albe, double cosIn, double sinIn);

/* SVM */  
extern void positionSVM(uint16_t *Tinv);
extern void ualbeSVM(double Ual, double Ube, double Ud, uint16_t *Tinv);

/* calculate lamdar */  
extern double lamdarCal(double lamdar, double id);
//extern void lamdaralbeCal(PHASE_ALBE ualbe, PHASE_ALBE ialbe, double *ualsum, \
        double *ubesum, double *ialsum, double *ibesum, PHASE_ALBE *lamdaralbe);

/* calculate position and speed */  
extern double spdCal_M();
//extern double wrCal_lamdar(PHASE_ALBE *lamdaralbe, double *anglek, PHASE_ALBE ualbe, PHASE_ALBE ialbe, double ts);
extern double positonCal(double wr, double lamdar, double iq, double theta);

/* PI module */  
extern double PImodule(double Kp, double Ki, double inputk, double err, double *lasterr, double Uplim, double Downlim);
extern double Integrator(double paramin, double sum, double ts);

/* Auxiliary Function */
extern double RAMP(double ramp, double initial, double increment, double Hlimit, double Llimit);
extern double roundn(double input, int _digit);