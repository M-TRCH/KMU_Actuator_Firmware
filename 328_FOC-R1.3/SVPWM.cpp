
/** 28-05-22
 *  Created by Teerachot Mueangchamnong 
 *  Brushless Motor Control for Quadruped Leg Project
 *  Mechatronics Engineering Technology, KMUTNB
 *  Refer https://www.switchcraft.org/learning/2017/3/15/space-vector-pwm-intro
 */

#include "SVPWM.h"

SVPWM::SVPWM()
{

}
SVPWM::~SVPWM()
{
  
}
void SVPWM::setVoltageSource(float vect)
{
  Vs = vect;
}
void SVPWM::setPWMResolution(float pwm)
{
  pwmRes = pwm;
  m = -2 * Vs / pwmRes;
  c = Vs;  
}
float SVPWM::getPwm(uint8_t i)
{
  return pwm[i];  
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
  Serial.print(pwm[0]); Serial.print("\t");
  Serial.print(pwm[1]); Serial.print("\t"); 
  Serial.println(pwm[2]);  
}
void SVPWM::updateClarke(float Vx, float Vy, float theta)
{
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
  
  pwm[0] = (Va - c) / m;
  pwm[1] = (Vb - c) / m;
  pwm[2] = (Vc - c) / m;
}
void SVPWM::updatePark(float Vd, float Vq, float theta)
{
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
  
  pwm[0] = (Va - c) / m;
  pwm[1] = (Vb - c) / m;
  pwm[2] = (Vc - c) / m;
}
void SVPWM::updateVector(float Vr, float theta)
{
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

  pwm[0] = (Va - c) / m;
  pwm[1] = (Vb - c) / m;
  pwm[2] = (Vc - c) / m;
}
void SVPWM::updateClarkeDeg(float Vx, float Vy, float theta)
{
  theta *= degToRad;
  updateClarkeDeg(Vx, Vy, theta);
}
void SVPWM::updateParkDeg(float Vd, float Vq, float theta)
{
  theta *= degToRad;
  updatePark(Vd, Vq, theta);
}
void SVPWM::updateVectorDeg(float Vr, float theta)
{
  theta *= degToRad;
  updateVector(Vr, theta);
}


    
