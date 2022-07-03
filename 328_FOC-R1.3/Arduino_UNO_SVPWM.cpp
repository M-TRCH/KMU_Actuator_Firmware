
/** 29-05-22
 *  Created by Teerachot Mueangchamnong 
 *  Brushless Motor Control for Quadruped Leg Project
 *  Mechatronics Engineering Technology, KMUTNB
 *  Refer https://www.circuito.io/blog/arduino-uno-pinout/
 */

#include "Arduino_UNO_SVPWM.h"

Arduino_UNO_SVPWM::Arduino_UNO_SVPWM(float v, float p, bool d)
{
  setVoltageSource(v);
  setPWMResolution(p);
  if(d)
    dir = 0;
  else
    dir = 2;
}
Arduino_UNO_SVPWM::~Arduino_UNO_SVPWM()
{
  
}
void Arduino_UNO_SVPWM::outputInit()
{
  /* Three PWM pin configuration */
  DDRD |= 0x08;   // D3/OCR2B config as output.
  DDRB |= 0x06;   // D9/OCR1A, D10/OCR1B config as output.
  TCCR1A = 0xF1;  // Config PWM mode as phase correct. 
  TCCR2A = 0x31;  
  TCCR1B = 0x02;  // 0x02 = Config timer1, 2 prescaler as 8 (3.9 kHz).
  TCCR2B = 0x02;  
  TCNT1 = 0;      // Reset counter of timer1, 2 
  TCNT2 = 0;      
  /* Enable pin configuration */
  pinMode(enableA, OUTPUT); // A, B, C Phase
  pinMode(enableB, OUTPUT); 
  pinMode(enableC, OUTPUT); 
}
void Arduino_UNO_SVPWM::motorEnable()
{
  digitalWrite(enableA, HIGH);
  digitalWrite(enableB, HIGH);
  digitalWrite(enableC, HIGH);
}
void Arduino_UNO_SVPWM::motorDisable()
{
  digitalWrite(enableA, LOW);
  digitalWrite(enableB, LOW);
  digitalWrite(enableC, LOW);
  resetPwm();
}
void Arduino_UNO_SVPWM::resetPwm()
{
  OCR2B = 0;
  OCR1A = 0;
  OCR1B = 0;  
}
void Arduino_UNO_SVPWM::resetPwm(unsigned long t)
{
  resetPwm();
  delay(t);
}
void Arduino_UNO_SVPWM::setPwm()
{
//  dir = i = [2,0] 
//  
//  [i]
//  [1]
//  [2 - i]
//  
//  i=2 -> 2,1,0
//  i=0 -> 0,1,2

  // Set compare macth each PWM pin.
  OCR2B = getPwm(dir); 
  OCR1A = getPwm(1);
  OCR1B = getPwm(2 - dir); 
}
void Arduino_UNO_SVPWM::setNewDir(boolean d)
{
  if(d)
    dir = 0;
  else
    dir = 2;
}
void Arduino_UNO_SVPWM::driveClarke(float Vx, float Vy, float theta)
{
  updateClarke(Vx, Vy, theta);
  setPwm();
}
void Arduino_UNO_SVPWM::drivePark(float Vd, float Vq, float theta)
{
  updatePark(Vd, Vq, theta);
  setPwm();  
}
void Arduino_UNO_SVPWM::driveVector(float Vr, float theta)
{
  updateVector(Vr, theta);
  setPwm();  
}
