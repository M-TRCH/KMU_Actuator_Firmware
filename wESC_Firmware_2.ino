#include "function.h"
#define thresholdSpd 25     // value < speed threshold = motor off 
volatile uint8_t step = 0;  // Starting to step0
volatile uint8_t spd = 50; // Speed of motor.
#define maxCount 800
#define minCount 50
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
    ACSR &= ~(1<<ACIE); //  Disable analog comparator interrupt.
//    ACSR &= ~(1<<ACI);   //  Set interrupt flag. 
    
    int sp = 0;
    set_pwm(spd);
    while(sp < 121)
    {
      
      set_step(5-step);
      step++; step %= 6;
      sp++; 
      delay(100); // 5
      Serial.println(sp); 
    }
    for(;;);
    step++; step %= 6;
    ACSR |= (1<<ACIE);  //  Enable analog comparator interrupt.
    ACSR &= ~(1<<ACI);  //  Dummy interrupt flag. 
    
    long timer = millis();
    while(spd >= thresholdSpd) 
    {  
      PORTB &= ~(1<<PB5); // Set D13 as low.
      if(Serial.available())
        spd = (int)Serial.read();
        
//      if(millis()-timer > 100)
//        setCount = minCount;
      Serial.println(deltaTime);
      set_pwm(spd);
      
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
  if(!step)
  {
    currentTime = micros(); 
    deltaTime = currentTime - previousTime;
    previousTime = currentTime;
  }
  set_step(step);
  step++; step %= 6;
  ACSR |= (1<<ACI);   //  Clear interrupt flag.
  PORTB |= (1<<PB5);  //  Set D13 as high.
}
