/******************************************************************************
| includes                          
|----------------------------------------------------------------------------*/
#include "imcontrol.h"
#include "math.h"

/******************************************************************************
| local variable definitions                          
|----------------------------------------------------------------------------*/

/******************************************************************************
| global variable definitions                          
|----------------------------------------------------------------------------*/
double n = 0;
double wr = 0;
double iqset = 0;

double ud_Isum = 0;
double uq_Isum = 0;
double iqset_Isum = 0;
int period_count = 0;

/******************************************************************************
@brief   Coordinate Transform
******************************************************************************/
void S3toR2(PHASE_ABC *abc, PHASE_DQ *dq, double theta)
{
  dq->d = sqrt(2.0/3.0) * (cos(theta) * abc->a + cos(theta - 2.0/3.0*pi) * abc->b + cos(theta + 2.0/3.0*pi) * abc->c);
  dq->q = -sqrt(2.0/3.0) * (sin(theta) * abc->a + sin(theta - 2.0/3.0*pi) * abc->b + sin(theta + 2.0/3.0*pi) * abc->c);
}

void S3toS2(PHASE_ABC *abc, PHASE_ALBE *albe)
{
  albe->al = sqrt(2.0/3.0) * (abc->a - 0.5 * abc->b - 0.5 * abc->c);
  albe->be = sqrt(2.0/3.0) * (sqrt(3)/2.0 * abc->b - sqrt(3)/2.0 * abc->c);
}

void S2toR2(PHASE_ALBE *albe, PHASE_DQ *dq, double theta)
{
  dq->d = cos(theta) * albe->al + sin(theta) * albe->be;
  dq->q = -sin(theta) * albe->al + cos(theta) * albe->be;
}

void R2toS3(PHASE_DQ *dq, PHASE_ABC *abc, double theta)
{
  abc->a = sqrt(2.0/3.0) * (cos(theta) * dq->d - sin(theta) *dq->q);
  abc->b = sqrt(2.0/3.0) * (cos(theta - 2.0/3.0*pi) * dq->d - sin(theta - 2.0/3.0*pi) *dq->q);
  abc->c = sqrt(2.0/3.0) * (cos(theta + 2.0/3.0*pi) * dq->d - sin(theta + 2.0/3.0*pi) *dq->q);
}

void S2toS3(PHASE_ALBE *albe, PHASE_ABC *abc)
{
  abc->a = sqrt(2.0/3.0) * albe->al;
  abc->b = sqrt(2.0/3.0) * (-0.5 * albe->al + sqrt(3)/2.0 * albe->be);
  abc->c = sqrt(2.0/3.0) * (-0.5 * albe->al - sqrt(3)/2.0 * albe->be);
}

void R2toS2(PHASE_DQ *dq, PHASE_ALBE *albe, double theta)
{
  albe->al = cos(theta) * dq->d - sin(theta) * dq->q;
  albe->be = sin(theta) * dq->d + cos(theta) * dq->q;
}

/******************************************************************************
@brief   Rotor Flux Calculation
******************************************************************************/
/* calculate lamdar */  
double lamdarCal(double lamdar, double ism)
{
  return (1.0 - Ts/Tr) * lamdar + Lm*Ts/Tr * ism;
}

void lamdardqCal()
{
}

void lamdaralbeCal()
{
}

/******************************************************************************
@brief   Calculate Position and Speed 
******************************************************************************/
void wrCal()
{
}

double positonCal(double wr, double lamdar, double ist, double theta)
{
  double we = 0;  
  
  if (lamdar > 0.01)
    we = Lm/Tr * ist / lamdar + wr;
  else
    we = 0;
  
  return Integrator(we, theta, Ts);
} 

/******************************************************************************
@brief   PI Module 
******************************************************************************/
double PImodule(double Kp, double Ki, double err, double *Isum, double Uplim, double Downlim)
{
  *Isum += Ki * Ts * err;
  if ((Kp * err + *Isum) >= Downlim && (Kp * err + *Isum) <= Uplim)
    return Kp * err + *Isum;
  else if (Kp * err + *Isum > Uplim)
    return Uplim;
  else
    return Downlim;
}

double Integrator(double paramin, double sum, double ts)
{
  return paramin * ts + sum;
}

/******************************************************************************
@brief   SVM 
******************************************************************************/ 
void positionSVM(unsigned int *Tinv)
{
  double Angle = 0;
  double theta = 0;
  int sector = 0;
  double Dm = 0, Dn = 0, D0 = 0;  // Dutycycle
  
  Angle = fmod((10 * pi * (period_count / 1000.0)), (2 * pi));
  theta = fmod(Angle,1/3.0 * pi);
  sector = (int)floor( Angle / (1/3.0 * pi)) + 1;
  Dm = M * sin(1/3.0 * pi - theta) / 2.0;
  Dn = M * sin(theta) / 2.0;
  D0 = (0.5 - Dm - Dn) / 2.0;
  Dm = roundn(Dm, 8);
  Dn = roundn(Dn, 8);
  D0 = roundn(D0, 8);
  if (D0 < 0) D0 = 0;
  
  switch (sector)
  {
  case 1:
    Tinv[0] = period - (int)floor(period * (D0));
    Tinv[1] = period - (int)floor(period * (D0 + Dm));
    Tinv[2] = period - (int)floor(period * (D0 + Dm + Dn));
    break;
  case 2:
    Tinv[0] = period - (int)floor(period * (D0 + Dn));
    Tinv[1] = period - (int)floor(period * (D0));
    Tinv[2] = period - (int)floor(period * (D0 + Dm + Dn));
    break;
  case 3:
    Tinv[0] = period - (int)floor(period * (D0 + Dm + Dn));
    Tinv[1] = period - (int)floor(period * (D0));
    Tinv[2] = period - (int)floor(period * (D0 + Dm));
    break;
  case 4:
    Tinv[0] = period - (int)floor(period * (D0 + Dm + Dn));
    Tinv[1] = period - (int)floor(period * (D0 + Dn));
    Tinv[2] = period - (int)floor(period * (D0));
    break;
  case 5:
    Tinv[0] = period - (int)floor(period * (D0 + Dm));
    Tinv[1] = period - (int)floor(period * (D0 + Dm + Dn));
    Tinv[2] = period - (int)floor(period * (D0));
    break;  
  case 6:
    Tinv[0] = period - (int)floor(period * (D0));
    Tinv[1] = period - (int)floor(period * (D0 + Dm + Dn));
    Tinv[2] = period - (int)floor(period * (D0 + Dn));
  }   
}

void ualbeSVM(double Ual, double Ube, unsigned int *Tinv)
{
  double dm, dn, d0;
  int sector;
  double k = Ube / Ual;
  
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
    dn = (-Ual - Ube/sqrt(3)) / Ud;
  }
  else if (Ual < 0 && Ube > 0 && k >= -sqrt(3) && k < 0)
  {
    sector = 3;
    dm = 2/sqrt(3) * Ual / Ud;
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

void udqSVM()
{
}

/******************************************************************************
@brief   Auxiliary Function
******************************************************************************/
double roundn(double input, int digit)
{
  double temp;
  temp = input * pow(10, digit);
  temp = floor(temp);
  temp = temp / (pow(10, digit) * 1.0);
  return temp;
}