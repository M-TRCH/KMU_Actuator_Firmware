#include "function.h"
volatile uint8_t step = 0;  // Starting to step0
volatile uint8_t spd = 150; // 175 - Speed of motor.

void setup() 
{
  Serial.begin(115200);
  A_SET();  B_SET();  C_SET();
  bemf_init();  input_init();
}
void loop() 
{
  if(spd >= 0) // 183
  { /* Motor on */
    ACSR |= (1<<ACIE); //  Enable analog comparator interrupt.
    set_pwm(spd-50);
    uint16_t count = 2000;
    while(count > 500) 
    {
      delayMicroseconds(count);
      set_step(step);
      step++; step %= 6;
      count -= 10;
    }
    while(spd >= 0) // 183
    {
      set_pwm(spd);  
//      set_step(step);
//      step++; step %= 6;
//      delay(1);
    }
  }
  if(spd < 175) spd = 175;
  else
  { /* Motor off */
    A_OFF();  B_OFF();  C_OFF();
    ACSR &= ~(1<<ACIE); //  Disable analog comparator interrupt.
    ACSR |= (1<<ACI);   //  Clear interrupt flag.
  }
}
ISR(ANALOG_COMP_vect) 
{ /* Step complete detector */
  for(uint16_t count = 0; count < 300; count++) 
  {     
    if(step & 1)  // If step1, 3 or 5
    { /* Read a ACO bit is low. */
      if(!(ACSR & B00100000)) count -= 1;
    }
    else                             
    { /* Read a ACO bit is high. */
      if((ACSR & B00100000))  count -= 1; 
    }
  }
  set_step(step);
  ACSR |= (1<<ACI);   //  Clear interrupt flag.
  step++; step %= 6;
}
//ISR(INT0_vect)
//{ /* Increase motor speed by tactor switch. */
//  delayMicroseconds(5);
//  if(PIND&(1<<PD2)) 
//  {
//    spd += 8; 
//    Serial.println(spd);
//  }
//}
