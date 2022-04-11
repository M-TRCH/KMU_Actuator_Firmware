
#ifndef OUTPUT_h
#define OUTPUT_h

#include "SVPWM.h"
#define InvertMotorRotate // select motor rotate (cw, ccw)
#define pwmRes 255.0f
#define Vs 12.0f
SVPWM sv(Vs, pwmRes);

void outputInit()
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
}
void motorStop()
{
  OCR2B = 0;
  OCR1A = 0;
  OCR1B = 0;
}
void motorStop(long time)
{
  OCR2B = 0;
  OCR1A = 0;
  OCR1B = 0;
  delay(time);
}

#ifdef InvertMotorRotate
void setPwm()
{
  // Set compare macth each PWM pin.
  OCR2B = sv.getPwmC(); 
  OCR1A = sv.getPwmB();
  OCR1B = sv.getPwmA(); 
}
#else
void setPwm()
{
  // Set compare macth each PWM pin.
  OCR2B = sv.getPwmA(); 
  OCR1A = sv.getPwmB();
  OCR1B = sv.getPwmC();  
}
#endif

#endif // #OUTPUT_h
