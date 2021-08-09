#include <TimerOne.h>

/*___________PWM Frequency table____________
          3     5     6     9    10    11
  |   1|31250|62500|62500|31250|31250|31250|
  |   8| 3906| 7812| 7812| 3906| 3906| 3906|
  |  32|  976|  -  |  -  |  -  |  -  |  976|
  |  64|  488|  976|  976|  488|  488|  488|
  | 128|  244|  -  |  -  |  -  |  -  |  244|
  | 256|  122|  244|  244|  122|  122|  122|
  |1024|   30|   61|   61|   30|   30|   30|
  ____________________________________________*/

/*__________________Setting_____________________
  | pole   |  +  |  -  |  +  |  -  |  +  |  -  |
  | mosfet |  Q1 |  Q2 |  Q3 |  Q4 |  Q5 |  Q6 |
  | pin    |  D3 |  D8 |  D5 |  D9 |  D6 | D10 |
  | freq   | 976 | 000 | 976 | 000 | 976 | 000 |
  ________________________________________________*/

#define potPin A0 
#define intPin 0
#define maxFreq 900
#define minFreq 0
#define SwingError 3
#define minSpdStart 30
float freq = 0, 
      period = 0, 
      prevFreq = 0;
volatile boolean finishTime = true, 
                 finishBEMF = true; 
boolean enableIRS = false;
uint8_t amp = 127, 
        sector = 0;
void finishTime_update()
{
  finishTime = true;
}
void finishBEMF_update()
{
  if(enableIRS)
  {
//     for(int i=0; i<5; i++)
//    {
//      if(sector == 1 && sector == 3 && sector == 5)
//      {
//        if(digitalRead(2) == LOW) i -= 1;
//      }
//      else
//      {
//        if(digitalRead(2) == HIGH) i -= 1;
//      }
//    }
    finishBEMF = true;
  }
}
void setMux(uint8_t sector)
{
       if(sector == 0 || sector == 2 || sector == 4) attachInterrupt(intPin, finishBEMF_update, RISING); 
  else if(sector == 1 || sector == 3 || sector == 5) attachInterrupt(intPin, finishBEMF_update, FALLING); 
         
  if(sector == 0 || sector == 3)
  { // 010
    PORTC = ~(1 << 1) & PORTC;
    PORTC =  (1 << 2) | PORTC;
    PORTC = ~(1 << 3) & PORTC;
  } 
  else if(sector == 1 || sector == 4)
  { // 100
    PORTC =  (1 << 1) | PORTC;
    PORTC = ~(1 << 2) & PORTC;
    PORTC = ~(1 << 3) & PORTC;
  } 
  else if(sector == 2 || sector == 5)
  { // 000
    PORTC = ~(1 << 1) & PORTC;
    PORTC = ~(1 << 2) & PORTC;
    PORTC = ~(1 << 3) & PORTC;
  } 
}
void setSector(uint8_t PWM, int8_t sector)
{
  if(sector == -1)
  { // 000
    PORTB = ~(1 << 0) & PORTB;
    PORTB = ~(1 << 1) & PORTB;
    PORTB = ~(1 << 2) & PORTB;
    TCCR0A = 0;
    TCCR2A = 0;
//    OCR0B = 0; // 5
//    OCR0A = 0; // 6
//    OCR2A = 0; // 11
  }
  else if(sector == 0)
  { // +-0
//    OCR0B = PWM; // 5
//    OCR2A = 0; // 11
    PORTB = ~(1 << 2) & PORTB;
    TCCR0A |= B00100111; // 5
    TCCR2A = 0; // 11  
  }
  else if(sector == 1)
  { // +0-
    
//    OCR0B = PWM; // 5
    PORTB = ~(1 << 1) & PORTB;
    PORTB =  (1 << 2) | PORTB;

    TCCR0A |= B00100111;; // 5
  }
  else if(sector == 2)
  { // 0+-
//    OCR0B = 0; // 5
//    OCR0A = PWM; // 6

    TCCR0A |= B10000111; // 5 6
  }
  else if(sector == 3)
  { // -+0
//    OCR0A = PWM; // 6
    PORTB =  (1 << 0) | PORTB;
    PORTB = ~(1 << 2) & PORTB; 

    TCCR0A |= B10000111; // 6
  }
  else if(sector == 4)
  { // -0+
//    OCR0A = 0; // 6
//    OCR2A = PWM; // 11

   TCCR0A = 0; // 6
   TCCR2A |= B10000111; // 11
  }
  else if(sector == 5)
  { // 0-+
    
//    OCR2A = PWM; // 11
    PORTB = ~(1 << 0) & PORTB;
    PORTB =  (1 << 1) | PORTB; 

    TCCR2A |= B10000111; // 11
  }
}
void setup()
{
  Serial.begin(115200);
  Timer1.attachInterrupt(finishTime_update);
  DDRB |= 0x0F; // กำหนดพอร์ต b พิน 0(D8) 1(D9) 2(D10) 3(D11*) เป็น output
  DDRC |= 0x0E; // กำหนดพอร์ต c พิน 1(A1) 2(A2) 3(A3) เป็น output
  DDRD |= 0x60; // d 5(D5) 6(D6)
  
  TCCR0A = 0; 
//  TCCR0A |= B10100111; // COM0A1, COM0B1, WGM00, WGM01, WGM02
  TCCR0B |= 0x03; // กำหนด prescale ของ timer2(5, 6) เป็น 64

  TCCR2A = 0;
//  TCCR2A |= B10100111; // กำหนด COM2A1, COM2B1, WGM21, WGM20
  TCCR2B |= 0x03; // กำหนด prescale ของ timer2(3, 11) เป็น 32


    OCR0B = amp; // 5
    OCR0A = amp; // 6
    OCR2A = amp; // 11
}
void loop()
{
  freq = map(analogRead(potPin), 0, 1023, minFreq, maxFreq);
  if(freq > prevFreq+SwingError || freq < prevFreq-SwingError)
  {
    prevFreq = freq;
//    period = 12000000;
    period = 1000000/abs(freq);
    Timer1.initialize(period/6);
  }

  if(freq > minSpdStart)
  {
    if(finishTime) // && finishBEMF)
    {
      enableIRS = false;
      
      sector++;
      sector %= 6; 
      setMux(sector);
      
      finishTime = false;
      finishBEMF = false;
    }
    else
    {
      enableIRS = true;
      setSector(amp, sector);
    }
   
  }
  else
  {
    setSector(0, -1);
    finishBEMF = true;
  }
}
