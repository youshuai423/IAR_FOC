/******************************************************************************
| includes                          
|----------------------------------------------------------------------------*/
#include "imcontrol.h"

/******************************************************************************
| local variable definitions                          
|----------------------------------------------------------------------------*/
double angle = 0;  // 向量与扇区间夹角
uint16_t FTM1cnt = 0;  // 存储上个周期的编码器脉冲数
int period_count = 0;  // PWM中断次数
uint16_t sector = 0;  // SVM扇区

/******************************************************************************
| global variable definitions                          
|----------------------------------------------------------------------------*/
/* 观测值 */
  // 电压
double Ud = 40;
PHASE_ABC uabc = {0, 0, 0};
PHASE_ALBE ualbe = {0, 0};
PHASE_DQ udq = {0, 0};
  // 电流
PHASE_ABC iabc = {0, 0, 0};
PHASE_ALBE ialbe = {0, 0};
PHASE_DQ idq = {0, 0};
  // 磁链
double lamdar = 0;
PHASE_ALBE lamdaralbe = {0, 0};
double theta = 0;
  // 转速
double speed = 0;

/* 给定值 */
  // 电压
double u_cmd = 0;  // 线电压幅值
PHASE_ALBE ualbe_cmd = {0, 0};
PHASE_DQ udq_cmd = {0, 0};
  // 电流
PHASE_DQ idq_cmd = {0, 0};
  // 转速
double spd_cmd = 0;  // 转速给定
double spd_req = 450;  // 转速设定

/* PI 变量 */
double idlasterr = 0;
double iqlasterr = 0;
double spdlasterr = 0;

/*==============================================================================
=========================== Coordinate Transform ===============================
==============================================================================*/

/******************************************************************************
@brief   3s/2r 坐标变换

@param   abc -- 三相向量
         dq -- 两相旋转向量（*指针传递*）
         theta -- 合成向量角度

@return  N/A
******************************************************************************/
void S3toR2(PHASE_ABC abc, PHASE_DQ *dq, double theta)
{
  // sqrt(2) = 1.414213562, 1.0/6.0*pi = 0.52359878
  dq->d = 1.414213562 * (cos(theta - 0.52359878) * abc.a + sin(theta) * abc.b);
  dq->q = -1.414213562 * (sin(theta - 0.52359878) * abc.a - cos(theta) * abc.b);
}

/******************************************************************************
@brief   3s/2s 坐标变换

@param   abc -- 三相向量
         albe -- 两相静止向量（*指针传递*）

@return  N/A
******************************************************************************/
void S3toS2(PHASE_ABC abc, PHASE_ALBE *albe)
{
  // sqrt(3.0/2.0) = 1.22474487, 1.0/sqrt(2) = 0.7071067812, sqrt(2) = 1.414213562
  albe->al = 1.22474487 * abc.a;
  albe->be = 0.7071067812 * abc.a + 1.414213562 * abc.b;
}

/******************************************************************************
@brief   2s/2r 坐标变换

@param   albe -- 两相静止向量
         dq -- 两相旋转向量（*指针传递*）
         theta -- 合成向量角度

@return  N/A
******************************************************************************/
void S2toR2(PHASE_ALBE albe, PHASE_DQ *dq, double cosIn, double sinIn)
{
  dq->d = cosIn * albe.al + sinIn * albe.be;
  dq->q = -sinIn * albe.al + cosIn * albe.be;
}

/******************************************************************************
@brief   2r/3s 坐标变换

@param   dq -- 两相旋转向量
         abc -- 三相向量（*指针传递*）
         theta -- 合成向量角度

@return  N/A
******************************************************************************/
void R2toS3(PHASE_DQ dq, PHASE_ABC *abc, double theta)
{
  // sqrt(2.0/3.0) = 0.81649658, 2.0/3.0*pi = 2.094395102
  abc->a = 0.81649658 * (cos(theta) * dq.d - sin(theta) *dq.q);
  abc->b = 0.81649658 * (cos(theta - 2.094395102) * dq.d - sin(theta - 2.094395102) *dq.q);
  abc->c = 0.81649658 * (cos(theta + 2.094395102) * dq.d - sin(theta + 2.094395102) *dq.q);
}

/******************************************************************************
@brief   2s/3s 坐标变换

@param   albe -- 两相静止向量
         abc -- 三相向量（*指针传递*）

@return  N/A
******************************************************************************/
void S2toS3(PHASE_ALBE albe, PHASE_ABC *abc)
{
  // sqrt(2.0/3.0) = 0.81649658, sqrt(3)/2.0 = 0.8660254
  abc->a = 0.81649658 * albe.al;
  abc->b = -0.40824829 * albe.al + 0.70710678 * albe.be; // abc->b = sqrt(2.0/3.0) * (-0.5 * albe.al + sqrt(3)/2.0 * albe.be)
  abc->c = -0.40824829 * albe.al - 0.70710678 * albe.be; // sqrt(2.0/3.0) * (-0.5 * albe.al - sqrt(3)/2.0 * albe.be);
}

/******************************************************************************
@brief   2r/2s 坐标变换

@param   dq -- 两相旋转向量
         albe -- 两相静止向量（*指针传递*）
         theta -- 合成向量角度

@return  N/A
******************************************************************************/
void R2toS2(PHASE_DQ dq, PHASE_ALBE *albe, double cosIn, double sinIn)
{
  albe->al = cosIn * dq.d - sinIn * dq.q;
  albe->be = sinIn * dq.d + cosIn * dq.q;
}

/*==============================================================================
==================================== SVM =======================================
==============================================================================*/

/******************************************************************************
@brief   位置式SVM -- 用于开环

@param   Tinv -- 三相PWM比较值（*指针传递*）

@return  N/A
******************************************************************************/
void positionSVM(uint16_t *Tinv)
{
    double Dm = 0, Dn = 0, D0 = 0;
    
    /* V/spd曲线计算电压给定值 */
    u_cmd = RAMP(VSpdramp, 0, spd_cmd, Voltlimit_H, Voltlimit_L);
  
    /* 扇区及夹角计算 */
    //theta += 2 * pi * (spd_cmd / 30.0) * 0.0001;  ==========================================================
    theta += 0.0000418879 * spd_cmd; // theta += 2 * pi * (spd_cmd / 30.0) * 0.0002; 
    if (theta > 6.2831852)  // 2 * pi = 6.2831852
      theta -= 6.2831852;
    
    angle = fmod(theta, 1.047197551);  // 1/3.0 * pi = 1.047197551
    sector = (int)floor(theta * 0.9549296586) + 1;  // 1 / (1/3.0 * pi) = 0.9549296586
    
    /* 占空比计算 */
    Dm = u_cmd / Ud * sin(1.047197551 - angle);  // 1/3.0 * pi = 1.047197551
    Dn = u_cmd / Ud * sin(angle);
    D0 = (1 - Dm - Dn) * 0.5;
    Dm = roundn(Dm, digit);
    Dn = roundn(Dn, digit);
    D0 = roundn(D0, digit);
    if (D0 < 0) D0 = 0;
  
    /* 三相PWM比较值计算 */
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
@brief   ualbeSVM -- 以alpha-beta给定值进行SVM

@param   Ual -- alpha轴电压给定值
         Ube -- beta轴电压给定值
         Ud -- 直流电压值
         Tinv -- 三相PWM比较值（*指针传递*）

@return  N/A
******************************************************************************/
void ualbeSVM(double Ual, double Ube, double Ud, uint16_t *Tinv)
{
  double dm, dn, d0;
  double k = Ube / Ual;
  double reciUd = 1.0 / Ud;
  
  /* 扇区判断及占空比计算 */
  // sqrt(3) = 1.7320508, sqrt(3) / 2.0 = 0.8660254044
  // 1 / sqrt(3) = 0.57735027 
  if (Ual > 0 && Ube >= 0 && k >= 0 && k < 1.7320508)
  {
    sector = 1;
    dm = 0.8660254044 * (Ual - Ube * 0.57735027) * reciUd;
    dn = Ube * reciUd;
  }
  else if (Ube > 0 && (k >= 1.7320508 || k < -1.7320508))
  {
    sector = 2;
    dm = 0.8660254044 * (Ual + Ube * 0.57735027) * reciUd;
    dn = 0.8660254044 * (-Ual + Ube * 0.57735027) * reciUd;
  }
  else if (Ual < 0 && Ube > 0 && k >= -1.7320508 && k < 0)
  {
    sector = 3;
    dm = Ube * reciUd;
    dn = 0.8660254044 * (-Ual - Ube * 0.57735027) * reciUd;
  }
  else if (Ual < 0 && Ube <= 0 && k >= 0 && k < 1.7320508)
  { 
    sector = 4;
    dm = 0.8660254044 * (-Ual + Ube * 0.57735027) * reciUd;
    dn = -Ube * reciUd;
  }
  else if (Ube < 0 && (k >= 1.7320508 || k < -1.7320508))
  {
    sector = 5;
    dm = 0.8660254044 * (-Ual - Ube * 0.57735027) * reciUd;
    dn = 0.8660254044 * (Ual - Ube * 0.57735027) * reciUd;
  }
  else if (Ual > 0 && Ube < 0 && k >= -1.7320508 && k < 0)
  {
    sector = 6;
    dm = -Ube * reciUd;
    dn = 0.8660254044 * (Ual + Ube * 0.57735027) * reciUd;
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
  
  /* 三相PWM比较值计算 */
  switch (sector)
  {
  case 1:
    {      
      Tinv[0] = (int)(period * (dm + dn + d0));
      Tinv[1] = (int)(period * (dn + d0));
      Tinv[2] = (int)(period * d0);
      break;
    }
  case 2:
    {
      Tinv[0] = (int)(period * (dm + d0));
      Tinv[1] = (int)(period * (dm + dn + d0));
      Tinv[2] = (int)(period * d0);
      break;
    }
  case 3:
    {
      Tinv[0] = (int)(period * (d0));
      Tinv[1] = (int)(period * (dm + dn + d0));
      Tinv[2] = (int)(period * (dn + d0));
      break;
    }
  case 4:
    {
      Tinv[0] = (int)(period * (d0));
      Tinv[1] = (int)(period * (dm + d0));
      Tinv[2] = (int)(period * (dm + dn + d0));
      break;
    }
  case 5:
    {
      Tinv[0] = (int)(period * (dn + d0));
      Tinv[1] = (int)(period * (d0));
      Tinv[2] = (int)(period * (dm + dn + d0));
      break;
    }
  case 6:
    {
      Tinv[0] = (int)(period * (dm + dn + d0));
      Tinv[1] = (int)(period * (d0));
      Tinv[2] = (int)(period * (dm + d0));
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
@brief   转子磁链幅值估计

@param   lamddar -- 上个周期转子磁链幅值
         id -- d轴电流值

@return  转子磁链幅值
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
@brief   转速计算 -- M法

@param   N/A

@return  转速值
******************************************************************************/
double spdCal_M()
{
    double speed = 0;
    
    if (FTM_RD_SC_TOF(FTM1) == 0)  // 计数值未溢出
      speed = (int)((FTM_RD_CNT(FTM1) - FTM1cnt) * 2.34375);
    else  // 溢出
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
@brief   磁链位置计算

@param   wr -- 转子转速（rad/s）
         lamdar -- 转子磁链
         iq -- q轴电流
         theta -- 上周期转子磁链角度

@return  转子磁链角度
******************************************************************************/
double positonCal(double wr, double lamdar, double iq, double theta)
{
  double we = 0;  
  
  /* 同步转速估计 */
  if (lamdar > lamdarlimit_L)
    we = Lm/Tr * iq / lamdar + wr;  // we = wsl + wr;
  else
    we = 0;
  
  return Integrator(we, theta, Ts);  // theta = 积分（we）
} 

/*==============================================================================
============================= Auxiliary Functions ==============================
==============================================================================*/

/******************************************************************************
@brief   PI 模块

@param   Kp -- 比例系数
         Ki -- 积分系数
         lastout -- 上周期输出
         err -- 误差输入
         lasterr -- 上周期误差输入 （*指针传递*）
         Uplim -- 积分限幅高
         Downlim -- 积分限幅低

@return  PI输出
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
@brief   积分模块

@param   paramin -- 积分输入
         sum -- 上周期积分值
         ts -- 控制周期

@return  积分值输出
******************************************************************************/
double Integrator(double paramin, double sum, double ts)
{
  return paramin * ts + sum;
}

/******************************************************************************
@brief   RAMP -- 增量式斜坡函数

@param   ramp -- 斜率
         initial -- 应变量起始值
         increment -- 自变量增量
         Hlimit -- 上限
         Llimit -- 下限

@return  应变量终值
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
@brief   roundn -- 有理数取指定位数

@param   input -- 输入
         digit -- 保留小数点后位数

@return  舍弃指定位数后的值
******************************************************************************/
double roundn(double input, int _digit)
{
  double temp;
  temp = input * _digit;
  temp = floor(temp);
  temp = temp / _digit;
  return temp;
}