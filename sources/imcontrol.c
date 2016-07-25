/******************************************************************************
| includes                          
|----------------------------------------------------------------------------*/
#include "imcontrol.h"

/******************************************************************************
| local variable definitions                          
|----------------------------------------------------------------------------*/
double angle = 0;  // ������������н�
uint16_t FTM1cnt = 0;  // �洢�ϸ����ڵı�����������
int period_count = 0;  // PWM�жϴ���
uint16_t sector = 0;  // SVM����

/******************************************************************************
| global variable definitions                          
|----------------------------------------------------------------------------*/
/* �۲�ֵ */
  // ��ѹ
double Ud = 0;
PHASE_ABC uabc = {0, 0, 0};
PHASE_ALBE ualbe = {0, 0};
PHASE_DQ udq = {0, 0};
  // ����
PHASE_ABC iabc = {0, 0, 0};
PHASE_ALBE ialbe = {0, 0};
PHASE_DQ idq = {0, 0};
  // ����
double lamdar = 0;
PHASE_ALBE lamdaralbe = {0, 0};
double theta = 0;
  // ת��
double speed = 0;

/* ����ֵ */
  // ��ѹ
double u_cmd = 0;
PHASE_ALBE ualbe_cmd = {0, 0};
PHASE_DQ udq_cmd = {0, 0};
  // ����
PHASE_DQ idq_cmd = {0, 0};
  // ת��
double spd_cmd = 0;  // ת�ٸ���
double spd_req = 0;  // ת���趨

/* PI ���� */
double idlasterr = 0;
double iqlasterr = 0;
double spdlasterr = 0;

/*==============================================================================
=========================== Coordinate Transform ===============================
==============================================================================*/

/******************************************************************************
@brief   3s/2r ����任

@param   abc -- ��������
         dq -- ������ת������*ָ�봫��*��
         theta -- �ϳ������Ƕ�

@return  N/A
******************************************************************************/
void S3toR2(PHASE_ABC abc, PHASE_DQ *dq, double theta)
{
  //dq->d = sqrt(2.0/3.0) * (cos(theta) * abc->a + cos(theta - 2.0/3.0*pi) * abc->b + cos(theta + 2.0/3.0*pi) * abc->c);
  //dq->q = -sqrt(2.0/3.0) * (sin(theta) * abc->a + sin(theta - 2.0/3.0*pi) * abc->b + sin(theta + 2.0/3.0*pi) * abc->c);
  dq->d = sqrt(2.0) * (cos(theta - 1.0/6.0*pi) * abc.a + sin(theta) * abc.b);
  dq->q = -sqrt(2.0) * (sin(theta - 1.0/6.0*pi) * abc.a - cos(theta) * abc.b);
}

/******************************************************************************
@brief   3s/2s ����任

@param   abc -- ��������
         albe -- ���ྲֹ������*ָ�봫��*��

@return  N/A
******************************************************************************/
void S3toS2(PHASE_ABC abc, PHASE_ALBE *albe)
{
  //albe->al = sqrt(2.0/3.0) * (abc->a - 0.5 * abc->b - 0.5 * abc->c);
  //albe->be = sqrt(2.0/3.0) * (sqrt(3)/2.0 * abc->b - sqrt(3)/2.0 * abc->c);
  albe->al = sqrt(3.0/2.0) * abc.a;
  albe->be = 1.0/sqrt(2) * abc.a + sqrt(2) * abc.b;
}

/******************************************************************************
@brief   2s/2r ����任

@param   albe -- ���ྲֹ����
         dq -- ������ת������*ָ�봫��*��
         theta -- �ϳ������Ƕ�

@return  N/A
******************************************************************************/
void S2toR2(PHASE_ALBE albe, PHASE_DQ *dq, double theta)
{
  dq->d = cos(theta) * albe.al + sin(theta) * albe.be;
  dq->q = -sin(theta) * albe.al + cos(theta) * albe.be;
}

/******************************************************************************
@brief   2r/3s ����任

@param   dq -- ������ת����
         abc -- ����������*ָ�봫��*��
         theta -- �ϳ������Ƕ�

@return  N/A
******************************************************************************/
void R2toS3(PHASE_DQ dq, PHASE_ABC *abc, double theta)
{
  abc->a = sqrt(2.0/3.0) * (cos(theta) * dq.d - sin(theta) *dq.q);
  abc->b = sqrt(2.0/3.0) * (cos(theta - 2.0/3.0*pi) * dq.d - sin(theta - 2.0/3.0*pi) *dq.q);
  abc->c = sqrt(2.0/3.0) * (cos(theta + 2.0/3.0*pi) * dq.d - sin(theta + 2.0/3.0*pi) *dq.q);
}

/******************************************************************************
@brief   2s/3s ����任

@param   albe -- ���ྲֹ����
         abc -- ����������*ָ�봫��*��

@return  N/A
******************************************************************************/
void S2toS3(PHASE_ALBE albe, PHASE_ABC *abc)
{
  abc->a = sqrt(2.0/3.0) * albe.al;
  abc->b = sqrt(2.0/3.0) * (-0.5 * albe.al + sqrt(3)/2.0 * albe.be);
  abc->c = sqrt(2.0/3.0) * (-0.5 * albe.al - sqrt(3)/2.0 * albe.be);
}

/******************************************************************************
@brief   2r/2s ����任

@param   dq -- ������ת����
         albe -- ���ྲֹ������*ָ�봫��*��
         theta -- �ϳ������Ƕ�

@return  N/A
******************************************************************************/
void R2toS2(PHASE_DQ dq, PHASE_ALBE *albe, double theta)
{
  albe->al = cos(theta) * dq.d - sin(theta) * dq.q;
  albe->be = sin(theta) * dq.d + cos(theta) * dq.q;
}

/*==============================================================================
==================================== SVM =======================================
==============================================================================*/

/******************************************************************************
@brief   λ��ʽSVM -- ���ڿ���

@param   Tinv -- ����PWM�Ƚ�ֵ��*ָ�봫��*��

@return  N/A
******************************************************************************/
void positionSVM(uint16_t *Tinv)
{
    double Dm = 0, Dn = 0, D0 = 0;
    
    /* V/spd���߼����ѹ����ֵ */
    u_cmd = RAMP(VSpdramp, 0, spd_cmd, Voltlimit_H, Voltlimit_L);
  
    /* �������нǼ��� */
    theta += 2 * pi * (spd_cmd / 30.0) * 0.0001;
    if (theta > 2 * pi)  theta -= 2*pi;
    angle = fmod(theta,1/3.0 * pi);
    sector = (int)floor( theta / (1/3.0 * pi)) + 1;
    
    /* ռ�ձȼ��� */
    Dm = u_cmd / Ud * sin(1/3.0 * pi - angle);
    Dn = u_cmd / Ud * sin(angle);
    D0 = (1 - Dm - Dn) / 2.0;
    Dm = roundn(Dm, 8);
    Dn = roundn(Dn, 8);
    D0 = roundn(D0, 8);
    if (D0 < 0) D0 = 0;
  
    /* ����PWM�Ƚ�ֵ���� */
    switch (sector)
    {
    case 1:
      Tinv[0] = (int)(period * (Dm + Dn + D0));
      Tinv[1] = (int)(period * (D0 + Dn));
      Tinv[2] = (int)(period * (D0));
      break;
    case 2:
      Tinv[0] = (int)(period * (Dm + D0));
      Tinv[1] = (int)(period * (Dm + Dn + D0));
      Tinv[2] = (int)(period * (D0));
      break;
    case 3:
      Tinv[0] = (int)(period * (D0));
      Tinv[1] = (int)(period * (Dm + Dn + D0));
      Tinv[2] = (int)(period * (Dn + D0));
      break;
    case 4:
      Tinv[0] = (int)(period * (D0));
      Tinv[1] = (int)(period * (Dm + D0));
      Tinv[2] = (int)(period * (Dm + Dn + D0));
      break;
    case 5:
      Tinv[0] = (int)(period * (Dn + D0));
      Tinv[1] = (int)(period * (D0));
      Tinv[2] = (int)(period * (Dm + Dn + D0));
      break;
    case 6:
      Tinv[0] = (int)(period * (Dm + Dn + D0));
      Tinv[1] = (int)(period * (D0));
      Tinv[2] = (int)(period * (Dm + D0));
    }
}

/******************************************************************************
@brief   ualbeSVM -- ��alpha-beta����ֵ����SVM

@param   Ual -- alpha���ѹ����ֵ
         Ube -- beta���ѹ����ֵ
         Ud -- ֱ����ѹֵ
         Tinv -- ����PWM�Ƚ�ֵ��*ָ�봫��*��

@return  N/A
******************************************************************************/
void ualbeSVM(double Ual, double Ube, double Ud, uint16_t *Tinv)
{
  double dm, dn, d0;
  double k = Ube / Ual;
  
  /* �����жϼ�ռ�ձȼ��� */
  if (Ual > 0 && Ube >= 0 && k >= 0 && k < sqrt(3))
  {
    sector = 1;
    dm = (Ual - Ube/sqrt(3)) / Ud;
    dn = 2/sqrt(3) * Ube / Ud;
  }
  else if (Ube > 0 && (k >= sqrt(3) || k < -sqrt(3)))
  {
    sector = 2;
    dm = (Ual + Ube/sqrt(3)) / Ud;
    dn = (-Ual + Ube/sqrt(3)) / Ud;
  }
  else if (Ual < 0 && Ube > 0 && k >= -sqrt(3) && k < 0)
  {
    sector = 3;
    dm = 2/sqrt(3) * Ube / Ud;
    dn = (-Ual - Ube/sqrt(3)) / Ud;
  }
  else if (Ual < 0 && Ube <= 0 && k >= 0 && k < sqrt(3))
  { 
    sector = 4;
    dm = (-Ual + Ube/sqrt(3)) / Ud;
    dn = -2/sqrt(3) * Ube / Ud;
  }
  else if (Ube < 0 && (k >= sqrt(3) || k < -sqrt(3)))
  {
    sector = 5;
    dm = (-Ual - Ube/sqrt(3)) / Ud;
    dn = (Ual - Ube/sqrt(3)) / Ud;
  }
  else if (Ual > 0 && Ube < 0 && k >= -sqrt(3) && k < 0)
  {
    sector = 6;
    dm = -2/sqrt(3) * Ube / Ud;
    dn = (Ual + Ube/sqrt(3)) / Ud;
  }
  else
  {
    sector = 1;
    dm = 0;
    dn = 0;
  }
  
  if (dm + dn >= 1)
  {
    double temp = dm / (dm + dn);
    dn = dn / (dm + dn);
    dm = temp;
    d0 = 0;
  }
  else
    d0 = 0.5 * (1 - dm - dn);
  
  /* ����PWM�Ƚ�ֵ���� */
  switch (sector)
  {
  case 1:
    {      
      Tinv[0] = (int)floor(period * (dm + dn + d0));
      Tinv[1] = (int)floor(period * (dn + d0));
      Tinv[2] = (int)floor(period * d0);
      break;
    }
  case 2:
    {
      Tinv[0] = (int)floor(period * (dm + d0));
      Tinv[1] = (int)floor(period * (dm + dn + d0));
      Tinv[2] = (int)floor(period * d0);
      break;
    }
  case 3:
    {
      Tinv[0] = (int)floor(period * (d0));
      Tinv[1] = (int)floor(period * (dm + dn + d0));
      Tinv[2] = (int)floor(period * (dn + d0));
      break;
    }
  case 4:
    {
      Tinv[0] = (int)floor(period * (d0));
      Tinv[1] = (int)floor(period * (dm + d0));
      Tinv[2] = (int)floor(period * (dm + dn + d0));
      break;
    }
  case 5:
    {
      Tinv[0] = (int)floor(period * (dn + d0));
      Tinv[1] = (int)floor(period * (d0));
      Tinv[2] = (int)floor(period * (dm + dn + d0));
      break;
    }
  case 6:
    {
      Tinv[0] = (int)floor(period * (dm + dn + d0));
      Tinv[1] = (int)floor(period * (d0));
      Tinv[2] = (int)floor(period * (dm + d0));
      break;
    }
  default:
    {
      Tinv[0] = period + 1;
      Tinv[1] = period + 1;
      Tinv[2] = period + 1;
    }
  }
}

/*==============================================================================
======================== Rotor Flux Calculation ================================
==============================================================================*/

/******************************************************************************
@brief   ת�Ӵ�����ֵ����

@param   lamddar -- �ϸ�����ת�Ӵ�����ֵ
         id -- d�����ֵ

@return  ת�Ӵ�����ֵ
******************************************************************************/
double lamdarCal(double lamdar, double id)
{
  return (1.0 - Ts/Tr) * lamdar + Lm*Ts/Tr * id;
}

/*void lamdaralbeCal(PHASE_ALBE ualbe, PHASE_ALBE ialbe, double *ualsum, double *ubesum, double *ialsum, double *ibesum, PHASE_ALBE *lamdaralbe)
{
  double tempal, tempbe;
  tempal = Integrator(ualbe.al, *ualsum, Ts) - Rs * Integrator(ialbe.al, *ialsum, Ts) - (Ls*Lr/Lm - Lm) * ialbe.al;
  tempbe = Integrator(ualbe.be, *ubesum, Ts) - Rs * Integrator(ialbe.be, *ibesum, Ts) - (Ls*Lr/Lm - Lm) * ialbe.be;
  lamdaralbe->al = tempal * Lr/Lm;
  lamdaralbe->be = tempbe * Lr/Lm;
}*/

/*==============================================================================
==================== Calculate Position and Speed ==============================
==============================================================================*/

/******************************************************************************
@brief   ת�ټ��� -- M��

@param   N/A

@return  ת��ֵ
******************************************************************************/
double spdCal_M()
{
    double speed = 0;
    
    if (FTM_RD_SC_TOF(FTM1) == 0)  // ����ֵδ���
      speed = (int)((FTM_RD_CNT(FTM1) - FTM1cnt) * 2.34375);
    else  // ���
    {
      speed = (int)((FTM_RD_CNT(FTM1) + FTM1_MODULO - FTM1cnt) * 2.34375);
      FTM_WR_SC_TOF(FTM1, 0);
    }
    
    FTM1cnt = FTM_RD_CNT(FTM1);
    
    return speed;
}

/*double wrCal_lamdar(PHASE_ALBE *lamdaralbe, double *anglek, PHASE_ALBE ualbe, PHASE_ALBE ialbe, double ts)
{
  double angle = 0;
  double we =0, wsl = 0;
  
  lamdaralbeCal(ualbe, ialbe, &ualsum, &ubesum, &ialsum, &ibesum, lamdaralbe);
  if (lamdaralbe->al != 0)
  {    
    angle = atan(lamdaralbe->be / lamdaralbe->al);
    if (fabs(angle - *anglek) < 0.5 * pi)
      we = (angle - *anglek) / ts;
    else if (angle <= 0)
      we = (angle - *anglek + pi) / ts;
    else
      we = (angle - *anglek - pi);
    
    *anglek = angle;
    
    wsl = Lm/Tr * (lamdaralbe->be - lamdaralbe->al) / (pow(lamdaralbe->al, 2) + pow(lamdaralbe->be, 2));
  }
  else
  {
    we = 0;
    wsl = 0;
  }
  
  if (fabs(we - wsl) < 90)
    return we - wsl;
  else
    return 0;
}*/

/******************************************************************************
@brief   ����λ�ü���

@param   wr -- ת��ת�٣�rad/s��
         lamdar -- ת�Ӵ���
         iq -- q�����
         theta -- ������ת�Ӵ����Ƕ�

@return  ת�Ӵ����Ƕ�
******************************************************************************/
double positonCal(double wr, double lamdar, double iq, double theta)
{
  double we = 0;  
  
  /* ͬ��ת�ٹ��� */
  if (lamdar > lamdarlimit_L)
    we = Lm/Tr * iq / lamdar + wr;  // we = wsl + wr;
  else
    we = 0;
  
  return Integrator(we, theta, Ts);  // theta = ���֣�we��
} 

/*==============================================================================
============================= Auxiliary Functions ==============================
==============================================================================*/

/******************************************************************************
@brief   PI ģ��

@param   Kp -- ����ϵ��
         Ki -- ����ϵ��
         lastout -- ���������
         err -- �������
         lasterr -- ������������� ��*ָ�봫��*��
         Uplim -- �����޷���
         Downlim -- �����޷���

@return  PI���
******************************************************************************/
double PImodule(double Kp, double Ki, double lastout, double err, double *lasterr, double Uplim, double Downlim)
{
  lastout += Kp * (err - *lasterr) + Ki * Ts * err;
  *lasterr = err;
  
  if (lastout >= Downlim && lastout <= Uplim)
    return lastout;
  else if (lastout > Uplim)
    return Uplim;
  else
    return Downlim;
}

/******************************************************************************
@brief   ����ģ��

@param   paramin -- ��������
         sum -- �����ڻ���ֵ
         ts -- ��������

@return  ����ֵ���
******************************************************************************/
double Integrator(double paramin, double sum, double ts)
{
  return paramin * ts + sum;
}

/******************************************************************************
@brief   RAMP -- ����ʽб�º���

@param   ramp -- б��
         initial -- Ӧ������ʼֵ
         increment -- �Ա�������
         Hlimit -- ����
         Llimit -- ����

@return  Ӧ������ֵ
******************************************************************************/
double RAMP(double ramp, double initial, double increment, double Hlimit, double Llimit)
{
  double temp = ramp * increment + initial;
  if (temp > Hlimit)
    return Hlimit;
  else if (temp < Llimit)
    return Llimit;
  else
    return temp;
}

/******************************************************************************
@brief   roundn -- ������ȡָ��λ��

@param   input -- ����
         digit -- ����С�����λ��

@return  ����ָ��λ�����ֵ
******************************************************************************/
double roundn(double input, int digit)
{
  double temp;
  temp = input * pow(10, digit);
  temp = floor(temp);
  temp = temp / (pow(10, digit) * 1.0);
  return temp;
}