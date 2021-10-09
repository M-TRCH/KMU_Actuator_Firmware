#include "function.h"
#define thresholdSpd 25     // value < speed threshold = motor off 
volatile uint8_t step = 0;  // Starting to step0
volatile uint8_t spd = 255; // Speed of motor.
#define max_spd_now 30
float spd_now = max_spd_now;
float soft_rev = 2.5f;
boolean soft_flag = true;
#define maxCount 800
#define minCount 20
volatile uint16_t setCount = minCount;
volatile unsigned long previousTime = 0;
volatile unsigned long currentTime = 0;
volatile float deltaTime = 0;

void setup() 
{  
  Serial.begin(4800);
  gate_init();  bemf_init();
  DDRB |= 0x20; //  Config D13(LED) as output.
}
void loop() 
{  
  if(Serial.available())
    spd = (int)Serial.read();
  
  if(spd >= thresholdSpd)
  { /* Motor on */
//    ACSR |= (1<<ACIE);  //  Enable analog comparator interrupt.
//   ACSR &= ~(1<<ACIE); //  Disable analog comparator interrupt.
//    ACSR &= ~(1<<ACI);   //  Set interrupt flag. 
    
    int sp = 0;
    set_pwm(spd);
    while(sp < 6) // 121
    {     
      set_step(step);
      step++; step %= 6;
      sp++; 
      delay(5); // 5
      Serial.println(sp); 
    }
    step++; step %= 6;
    ACSR |= (1<<ACIE);  //  Enable analog comparator interrupt.
    ACSR &= ~(1<<ACI);  //  Dummy interrupt flag. 
    
    // long timer = millis();
    while(spd >= thresholdSpd) 
    {  
      PORTB &= ~(1<<PB5); // Set D13 as low.
//      if(Serial.available())
//        spd = (int)Serial.read();
//        
//      if(millis()-timer > 100)
//        setCount = minCount;
      // Serial.println(deltaTime);

      if(soft_flag)
      {
        spd_now += soft_rev;
        if(spd_now >= spd)
        {
          spd_now = spd;
          soft_flag = false;
        }
      }
      set_pwm(spd_now);
      
//      set_step(step);
//      step++; step %= 6;
//      delay(1000);
//      delayMicrosevonds(1000);
   }
  }
  else      
  { /* Motor off */
    PORTB &= ~(1<<PB5); // Set D13 as low.
    ACSR &= ~(1<<ACIE); //  Disable analog comparator interrupt.
    ACSR |= (1<<ACI);   //  Clear interrupt flag.
    A_OFF();  B_OFF();  C_OFF();
    // setCount = maxCount;
  }
}
ISR(ANALOG_COMP_vect) 
{ /* Step complete detector */
//  for(uint16_t count = 0; count < setCount; count++) 
//  {     
//    if(step & 1)  // If step1, 3 or 5
//    { /* Read a ACO bit is low. */
//      if(!(ACSR & B00100000)) count -= 1;
//    }
//    else                             
//    { /* Read a ACO bit is high. */
//      if((ACSR & B00100000))  count -= 1; 
//    }
//  }
//  if(!step)
//  {
//    currentTime = micros(); 
//    deltaTime = currentTime - previousTime;
//    previousTime = currentTime;
//  }
  set_step(step);
  step++; step %= 6;
  ACSR |= (1<<ACI);   //  Clear interrupt flag.
  PORTB |= (1<<PB5);  //  Set D13 as high.
  soft_flag = true;
  spd_now = max_spd_now;
}
