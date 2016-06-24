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
int period_count = 0;

/******************************************************************************
@brief   Coordinate transform

@param   N/A

@return  N/A
******************************************************************************/
void S3toR2(PHASE_ABC *abc, PHASE_DQ *dq, double theta)
{
  dq->d = sqrt(2.0/3.0) * (cos(theta) * abc->a + cos(theta - 2.0/3.0*pi) * abc->b + cos(theta + 2.0/3.0*pi) * abc->c);
  dq->q = -sqrt(2.0/3.0) * (sin(theta) * abc->a + sin(theta - 2.0/3.0*pi) * abc->b + sin(theta + 2.0/3.0*pi) * abc->c);
}

void S3toS2(double *abc, double *albe)
{
  albe[0] = sqrt(2.0/3.0) * (abc[0] - 0.5 * abc[1] - 0.5 * abc[2]);
  albe[1] = sqrt(2.0/3.0) * (sqrt(3)/2.0 * abc[1] - sqrt(3)/2.0 * abc[2]);
}

void S2toR2(double *albe, double *dq, double theta)
{
  dq[0] = cos(theta) * albe[0] + sin(theta) * albe[1];
  dq[1] = -sin(theta) * albe[0] + cos(theta) * albe[1];
}

void R2toS3(double *dq, double *abc, double theta)
{
  abc[0] = sqrt(2.0/3.0) * (cos(theta) * dq[0] - sin(theta) *dq[1]);
  abc[1] = sqrt(2.0/3.0) * (cos(theta - 2.0/3.0*pi) * dq[0] - sin(theta - 2.0/3.0*pi) *dq[1]);
  abc[2] = sqrt(2.0/3.0) * (cos(theta + 2.0/3.0*pi) * dq[0] - sin(theta + 2.0/3.0*pi) *dq[1]);
}

void S2toS3(double *albe, double *abc)
{
  abc[0] = sqrt(2.0/3.0) * albe[0];
  abc[1] = sqrt(2.0/3.0) * (-0.5 * albe[0] + sqrt(3)/2.0 * albe[1]);
  abc[2] = sqrt(2.0/3.0) * (-0.5 * albe[0] - sqrt(3)/2.0 * albe[1]);
}

void R2toS2(double *dq, double *albe, double theta)
{
  albe[0] = cos(theta) * dq[0] - sin(theta) * dq[1];
  albe[1] = sin(theta) * dq[0] + cos(theta) * dq[1];
}

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

/* calculate position and speed */  
void wrCal()
{
}

void positonCal(double wr, double lamdar, double ist)
{
  double we = 0;
  double theta = 0;
  
  we = Lm/Tr * ist / lamdar + wr;
  theta = Integrator(we, theta);
} 

/* PI module */  
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

double Integrator(double paramin, double sum)
{
  return paramin * Ts + sum;
}

/* SVM */  
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

double roundn(double input, int digit)
{
  double temp;
  temp = input * pow(10, digit);
  temp = floor(temp);
  temp = temp / (pow(10, digit) * 1.0);
  return temp;
}