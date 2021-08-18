#include "function.h"
#define thresholdSpd 5      // value < speed threshold = motor off 
volatile uint8_t step = 0;  // Starting to step0
volatile uint8_t spd = 0;   // 175 - Speed of motor.
#define minCount  50
#define maxCount  600
volatile uint16_t setCount = minCount;
unsigned long prevTime = 0;

void setup() 
{
  Serial.begin(115200);
  gate_init();  bemf_init();
  DDRB |= 20; //  Config D13(LED) as output.
}
void loop() 
{  
  if(Serial.available())
    spd = (int)Serial.read();
  
  if(spd >= thresholdSpd)
  { /* Motor on */
    set_pwm(255);
//    uint16_t count = 7;
//    while(count > 0) 
//    {
//      delayMicroseconds(2000);
//      set_step(step);
//      step++; step %= 6;
//      count -= 1;
//    }
    ACSR |= (1<<ACIE);  //  Enable analog comparator interrupt.
    PORTB |= (1<<PB5);  //  Set D13 as high.
    ACSR &= ~(1<<ACI);  //  Dummy interrupt flag.
  
    while(spd >= thresholdSpd) 
    {
      
      if(Serial.available())
        spd = (int)Serial.read();

      set_pwm(spd);
    }
  }
  else
  { /* Motor off */
    PORTB &= ~(1<<PB5); // Set D13 as low.
    A_OFF();  B_OFF();  C_OFF();
    ACSR &= ~(1<<ACIE); //  Disable analog comparator interrupt.
    ACSR |= (1<<ACI);   //  Clear interrupt flag.
  }
}
ISR(ANALOG_COMP_vect) 
{ /* Step complete detector */
  for(uint16_t count = 0; count < setCount; count++) 
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
  ACSR |= (1<<ACI); //  Clear interrupt flag.
  step++; step %= 6;
}
