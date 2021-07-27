
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
  | pin    |  D9 |  D6 | D10 |  D5 | D11 |  D3 |
  | freq   | 488 | 244 | 488 | 244 | 488 | 244 |
  ________________________________________________*/

String cmdTopic;  //  topic of command
int cmdVal;       //  value of command

boolean dir = true;         //  true = cw direction
int ampMax = 0, amp = 127;  //  amplitude
float freq = 0, period = 0; //  frequency and period
unsigned long prevTime = 0; //  start timer

int nGate[7] = {-1, 9, 7, 10, 8, 11, 12};
#define deadTime 120 //  micro senconds unit
#define ON 255       //  100% duty cycle 
#define OFF  0       //    0% duty cycle

boolean allowChange = true;
uint8_t sectorOut = 0;
double count = 0;
boolean enableIRS = false;

float currTime = 0;
int sector = 1;

  
String getSplit(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++)
  {
    if (data.charAt(i) == separator || i == maxIndex)
    {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
float getPeriod(float freq)
{
  return 1000000 / freq;
}
void setPwmFrequency(int pin, int divisor)
{
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10)
  {
    switch (divisor)
    {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6)
    {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    }
    else
    {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  }
  else if (pin == 3 || pin == 11)
  {
    switch (divisor)
    {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
} 
void setSector()
{
  if(enableIRS)
  {
//    for(int i=0; i<3; i++)
//    {
//      if(sectorOut == 2 && sectorOut == 4 && sectorOut == 6)
//      {
//        if(digitalRead(2) == LOW) i -= 1;
//      }
//      else
//      {
//        if(digitalRead(2) == HIGH) i -= 1;
//      }
//    }
    allowChange = true;
  }
}
void setup()
{
  Serial.begin(115200);
  while(!Serial);
  attachInterrupt(0, setSector, RISING);
  setPwmFrequency(3, 128);
  setPwmFrequency(5, 256);
  setPwmFrequency(6, 256);
//  setPwmFrequency(9, 64);
//  setPwmFrequency(10, 64);
//  setPwmFrequency(11, 64);
  setPwmFrequency(9, 8);
  setPwmFrequency(10, 8);
  setPwmFrequency(11, 8);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(nGate[1], OUTPUT);
  pinMode(nGate[2], OUTPUT);
  pinMode(nGate[3], OUTPUT);
  pinMode(nGate[4], OUTPUT);
  pinMode(nGate[5], OUTPUT);
  pinMode(nGate[6], OUTPUT);
//  pinMode(2, INPUT_PULLUP);
}
void loop()
{
//  Serial.println(analogRead(A0));
//  if (Serial.available())
//  {
//    String cmdRaw = Serial.readString();
//    cmdTopic = getSplit(cmdRaw, '=', 0);
//    String val = getSplit(cmdRaw, '=', 1);
//    cmdVal = val.toInt();
//    //    Serial.println("topic: " + cmdTopic + ", value: " + cmdVal);
//
//    if (cmdTopic == "a")
//    {
//      ampMax = cmdVal;
//      Serial.print("amplitude: ");
//      Serial.println(ampMax);
//    }
//    else if (cmdTopic == "d") dir = cmdVal;
//    else if (cmdTopic == "f")
//    {
//      freq = cmdVal;
//      period = getPeriod(freq);
//      Serial.print("period: ");
//      Serial.println(period);
//    }
//  }


  freq = map(analogRead(A0), 0, 1023, 0, 1200);
  period = getPeriod(abs(freq));
  float timeSector = period/6;
//  float timeSector = 300000; // for test
  
//  Serial.println(freq);
//  amp = ampMax; // set amplitude
//  prevTime = micros();

//  Serial.println(allowChange);
  
  if(freq > 80 || freq < -80)
  {
//    while(currTime < period)
//    {
      currTime =  micros() - prevTime;
      if(currTime >= timeSector &&  allowChange)
      {
        enableIRS = false;
//        pulseGen(0, 0);
        sector++;
        if(sector >= 7) sector = 1;
        setMux(sector);
   
        allowChange = false;
        prevTime = micros();
      }
      else
      {
        enableIRS = true;
//        sectorOut = sector;
//        if(freq < 0)
//        {
//          sectorOut = 7-sector;
//        }
        pulseGen(amp, sector); 
//        Serial.println(sector);
      }
//    }
  }
  else
  {
    pulseGen(0, 0);
    prevTime = micros();
    allowChange = true;
    //  stop motor
  }
}
void selectInput(boolean bitA, boolean bitB, boolean bitC)
{
  digitalWrite(A1, bitA);
  digitalWrite(A2, bitB);
  digitalWrite(A3, bitC);
}
void pulseGen(uint8_t PWM, uint8_t sector)
{
  if(sector == 0)
  {
    analogWrite(nGate[1], 0); digitalWrite(nGate[2], 0); // 0
    analogWrite(nGate[3], 0); digitalWrite(nGate[4], 0); // 0
    analogWrite(nGate[5], 0); digitalWrite(nGate[6], 0); // 0
    delayMicroseconds(deadTime);
  }
  else if(sector == 1)
  {
    analogWrite(nGate[1], PWM); digitalWrite(nGate[2], 0); // +
    analogWrite(nGate[3], 0);   digitalWrite(nGate[4], 1); // -
    analogWrite(nGate[5], 0);   digitalWrite(nGate[6], 0); // 0
  }
  else if(sector == 2)
  {
    analogWrite(nGate[1], PWM); digitalWrite(nGate[2], 0); // +
    analogWrite(nGate[3], 0);   digitalWrite(nGate[4], 0); // 0
    analogWrite(nGate[5], 0);   digitalWrite(nGate[6], 1); // -
  }
  else if(sector == 3)
  {
    analogWrite(nGate[1], 0);   digitalWrite(nGate[2], 0); // 0
    analogWrite(nGate[3], PWM); digitalWrite(nGate[4], 0); // +
    analogWrite(nGate[5], 0);   digitalWrite(nGate[6], 1); // -
  }
  else if(sector == 4)
  {
    analogWrite(nGate[1], 0);   digitalWrite(nGate[2], 1); // -
    analogWrite(nGate[3], PWM); digitalWrite(nGate[4], 0); // +
    analogWrite(nGate[5], 0);   digitalWrite(nGate[6], 0); // 0
  }
  else if(sector == 5)
  {
    analogWrite(nGate[1], 0);   digitalWrite(nGate[2], 1); // -
    analogWrite(nGate[3], 0);   digitalWrite(nGate[4], 0); // 0
    analogWrite(nGate[5], PWM); digitalWrite(nGate[6], 0); // + 
  }
  else if(sector == 6)
  {
    analogWrite(nGate[1], 0);   digitalWrite(nGate[2], 0); // 0
    analogWrite(nGate[3], 0);   digitalWrite(nGate[4], 1); // -
    analogWrite(nGate[5], PWM); digitalWrite(nGate[6], 0); // +
  }
}
void setMux(int sector)
{
       if(sector == 1){ selectInput(0, 1, 0); attachInterrupt(0, setSector, RISING);  }
  else if(sector == 2){ selectInput(1, 0, 0); attachInterrupt(0, setSector, FALLING); }
  else if(sector == 3){ selectInput(0, 0, 0); attachInterrupt(0, setSector, RISING);  }
  else if(sector == 4){ selectInput(0, 1, 0); attachInterrupt(0, setSector, FALLING); }
  else if(sector == 5){ selectInput(1, 0, 0); attachInterrupt(0, setSector, RISING);  }
  else if(sector == 6){ selectInput(0, 0, 0); attachInterrupt(0, setSector, FALLING); } 
}
