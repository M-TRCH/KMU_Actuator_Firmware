
/** 19-03-22
 *  Created by Teerachot Mueangchamnong 
 *  Brushless Motor Control for Quadruped Leg Project
 *  Mechatronics Engineering Technology, KMUTNB
 *  Refer https://www.switchcraft.org/learning/2017/3/15/space-vector-pwm-intro
*/

#include "SVPWM.h"

SVPWM::SVPWM(float vect, float pwm)
{
  Vs = vect;
  pwmRes = pwm;
  m = -2 * Vs / pwmRes;
  c = Vs;
}
SVPWM::~SVPWM()
{
}
float SVPWM::getPwmA()
{
  return pwmA;  
}
float SVPWM::getPwmB()
{
  return pwmB;  
}
float SVPWM::getPwmC()
{
  return pwmC;  
}
float SVPWM::getVx()
{
  return Vx;
}
float SVPWM::getVy()
{
  return Vy;
}
float SVPWM::getVa()
{
  return Va;
}
float SVPWM::getVb()
{
  return Vb;
}
float SVPWM::getVc()
{
  return Vc;
}
void SVPWM::debugPrint()
{
  Serial.print(pwmA); Serial.print("\t");
  Serial.print(pwmB); Serial.print("\t"); 
  Serial.println(pwmC);  
}
void SVPWM::updateClarke(float Vx, float Vy, float theta)
{
  theta *= degToRad;
  float Vr = sqrt(Vx * Vx + Vy * Vy);  
  Vr = constrain(Vr, 0, Vs);
  
  Va = _2divided_by3 * Vx;
  Vb = (Vy - root3_divided_by3 * Vx) / root3;
  Vc = 2.0f * (Va - Vx) - Vb;

  thirdTheta = theta * 3.0f;
  Vt = Vr * thirdRatio * cos(thirdTheta + PI);
  Va = Va * firstRatio + Vt;
  Vb = Vb * firstRatio + Vt;
  Vc = Vc * firstRatio + Vt;
  
  pwmA = (Va - c) / m;
  pwmB = (Vb - c) / m;
  pwmC = (Vc - c) / m;
}
void SVPWM::updatePark(float Vd, float Vq, float theta)
{
  theta *= degToRad;
  float C = cos(theta);
  float S = sin(theta);
  float Vx = Vd * C - Vq * S;
  float Vy = Vd * S + Vq * C;
  float Vr = sqrt(Vx * Vx + Vy * Vy);  
  Vr = constrain(Vr, 0, Vs);
  
  Va = _2divided_by3 * Vx;
  Vb = (Vy - root3_divided_by3 * Vx) / root3;
  Vc = 2.0f * (Va - Vx) - Vb;

  thirdTheta = theta * 3.0f;
  Vt = Vr * thirdRatio * cos(thirdTheta + PI);
  Va = Va * firstRatio + Vt;
  Vb = Vb * firstRatio + Vt;
  Vc = Vc * firstRatio + Vt;
  
  pwmA = (Va - c) / m;
  pwmB = (Vb - c) / m;
  pwmC = (Vc - c) / m;
}
void SVPWM::updateVector(float Vr, float theta)
{
  theta *= degToRad;
  Vr = constrain(Vr, 0, Vs);
  
  Vx = Vr * cos(theta);
  Vy = Vr * sin(theta);
  Va = _2divided_by3 * Vx;
  Vb = (Vy - root3_divided_by3 * Vx) / root3;
  Vc = 2.0f * (Va - Vx) - Vb;
  
  thirdTheta = theta * 3.0f;
  Vt = Vr * thirdRatio * cos(thirdTheta + PI);
  Va = Va * firstRatio + Vt;
  Vb = Vb * firstRatio + Vt;
  Vc = Vc * firstRatio + Vt;

  pwmA = (Va - c) / m;
  pwmB = (Vb - c) / m;
  pwmC = (Vc - c) / m;
}

    
