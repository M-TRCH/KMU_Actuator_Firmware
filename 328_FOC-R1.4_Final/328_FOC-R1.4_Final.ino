
/** 12-07-22
 *  Created by Teerachot Mueangchamnong 
 *  Brushless Motor Control for Quadruped Leg Project
 *  Mechatronics Engineering Technology, KMUTNB
 *  Refer https://drive.google.com/drive/mobile/folders/1C3AG5cfEpnGPkBm_-8elAVHii3mLRHgn?usp=sharing
 */

#include "Arduino_UNO_SVPWM.h"
Arduino_UNO_SVPWM sv(12, 255, false);  
// Arduino_UNO_SVPWM -> Constructor argument list.
// float(voltage soucre), float(pwm resolution), boolean(invert direction) 

#include "AbsoluteEncoder.h"
AbsoluteEncoder en(8, 20, true, 9000, 8860, 0); 
// AbsoluteEncoder -> Constructor argument list.
// uint8_t(Slave select pin), uint8_t(motor pole pair), boolean(Invert direction counter), uint8_t(Rotor position offset 'CW'), uint8_t(Rotor position offset 'CCW'), uint8_t(Shaft position offset)

#include "Data_Logger.h"
// variables storage

/* >>>>>>>>>>>>>>>>>>>>> mode options */
//#define RotorPositionAlignment
#define EncoderEnable
#define PositionControl
//#define TrajectoryControl
//#define VelocityControl
//#define OpenLoopDriveCW
//#define OpenLoopDriveCCW
//#define CloseLoopDriveCW
//#define CloseLoopDriveCCW
#define currentSafety
//#define tempSafety

/* >>>>>>>>>>>>>>>>>>>>> position control */
float magnitude, angle;
float pos_setpoint = 0, commandPos = 0, pos_actual,
      pos_error, pos_integral; 
#define posGap 0.01740
// 1.0 => 0.01740 rad
// 0.5 => 0.00873 rad
// 0.2 => 0.00349 rad << use
#define pos_I_Limit 2000.0f //-> old = 5000
// velocity generate
#define periodVel 2     // ms unit
unsigned long periodVelCounter = millis();
#define velDrive limitVel * GearRatio // rad/s unit
#define velRevMax velDrive * periodVel / 1000.0f
float velRev = velRevMax;
float difPos = 0;
// acceleration generate
#define periodAccel 2   // ms unit
unsigned long periodAccelCounter = millis();
#define accelDrive accel * GearRatio  // rad/s^2 unit
#define decelDrive decel * GearRatio  // rad/s^2 unit
#define accelRev accelDrive * periodAccel / 1000.0f * (periodVel / 1000.0f) 
#define decelRev decelDrive * periodAccel / 1000.0f * (periodVel / 1000.0f)
#define criticalStartAngle  (velDrive * velDrive) / (2.0f * accelDrive)
#define criticalStopAngle   (velDrive * velDrive) / (2.0f * decelDrive)

/* >>>>>>>>>>>>>>>>>>>>> velocity control */
#ifdef VelocityControl
  #include <Filters.h>
  FilterOnePole vel(LOWPASS, 1.4);  //  old -> 0.6, NEW -> 1.2
#endif
float vel_setpoint = 0, commandVel = 0, vel_actual,
      vel_error, vel_integral;
#define velGap 0.002 
#define vel_I_Limit 10.0f // old -> 3
#define maxAvgCount 5
uint8_t avgCount = 0;
float sumVel;
unsigned long prevTimeSampling = millis();

/* >>>>>>>>>>>>>>>>>>>>> motor safety */
unsigned long prevTimeSafety = millis();
#define safetyPeriod 3000
// current sensor
#define currentPin A0
int16_t ADC_thres = 512;
#define ADC_to_mV 4.887585533f 
#define mV_to_A 0.01f
#define getCurrent float(analogRead(currentPin) - ADC_thres) * ADC_to_mV * mV_to_A
#include <Filters.h>
FilterOnePole currentLPF(LOWPASS, 0.4);
// temp sensor
#include <M2M_LM75A.h>
M2M_LM75A temp;

//////////////////////////////////////////////////////////////////////////////////////

/* software reset */
void(*resetFunc) (void) = 0; 
/* safety interrupt */
void safetyEvent()
{
  sv.motorDisable();
  resetFunc();     
}
/* serial communication interrupt */
void serialEvent()
{    
  if(Serial.find(controlHeader))
  {  
    idBuf = Serial.parseInt();
    cmdBuf = Serial.parseFloat();
    if(idBuf == -1)
    {
      safetyEvent();
    }
    if(idBuf == motorID)
    {
      command = cmdBuf;
      idBuf = 255;
    }
  }   
}
/* call permanent data -> update buffer data */
void callEEPROM()
{
  for(uint8_t index=0; index<configLen; index++)
  {
    *bufVal[index] = readEEPROM(eepromAddress[index]);
//    Serial.println(*bufVal[index]);
  }
}
/* "Invalid data" print */
void invalidData()
{
  Serial.println(F("Invalid data"));
}
/* actuator action func */
void ACTION_motorEnable(int data)
{
  switch(data)
  {
    case 0:
      sv.motorDisable();
      sv.resetPwm();
      Serial.println(F("Motor Disable"));
      break;
    case 1: // enable
      sv.outputInit();
      sv.motorEnable();   
      Serial.println(F("Motor Enable"));
      break;  
    default:
      invalidData();
      break;
  }
}
void ACTION_openLoopDrive(int data)
{
  if(data == 0 || data == 1)
  {
    ACTION_motorEnable(1);
    unsigned long prevTime = millis();
    float magnitude = Vs * voltTest / 100.0f;
    float angle = 0;
    int dir;
    if(data)
    {
      Serial.println(F("Open Loop Drive CCW"));
      dir = 1;
    }
    else
    {
      Serial.println(F("Open Loop Drive CW")); 
      dir = -1;
    }
    while(true)
    {
      if(millis()-prevTime > timeTest * 1000.0f)  break;
      sv.driveVector(magnitude, angle);
      angle = angle + velTest * dir;
    }
    ACTION_motorEnable(0);
  }
  else
  {
    invalidData();
  }
}
void ACTION_closeLoopDrive(int data)
{
  if(data == 0 || data == 1)
  {
    ACTION_motorEnable(1);
    unsigned long prevTime = millis();
    float magnitude = Vs * voltTest / 100.0f;
    float angle = 0;
    int dir;
    if(data)
    {
      Serial.println(F("Close Loop Drive CCW"));
      dir = 1;
    }
    else
    {
      Serial.println(F("Close Loop Drive CW")); 
      dir = -1;
    }
    while(true)
    {
      if(millis()-prevTime > timeTest * 1000.0f)  break;
      en.update();
      angle = en.getRotorAngle(dir>0? CCW: CW) + In90Deg * dir / abs(dir);
      sv.driveVector(magnitude, angle);
    }
    ACTION_motorEnable(0);
  }
  else
  {
    invalidData();
  }
}
void ACTION_motorAlign(int data)
{
  if(data == 1)
  {
    // go to zero vector
    ACTION_motorEnable(1);
    float magnitude = Vs * voltTest / 100.0f;
    float angle = 0;
    sv.driveVector(magnitude, angle);
    delay(timeTest * 1000.0f); 
    ACTION_motorEnable(0);
    
    // offset calculate  
    float rawAngle = en.readRaw();
    float rangeAngle = 16384.0f / pp;
    int offsetAngle = -1;
    for(int i=1; i<=pp; i++)
    {
      float offset = rawAngle - (rangeAngle * i);
      if(offset <= rangeAngle && offset >= 0)
      {
        offsetAngle = offset;
        break;
      }
    }
    Serial.print(F("Offset Angle\t")); 
    Serial.println(offsetAngle);
  }
  else if(data == 0)
  {
    Serial.print(F("Raw Angle\t")); 
    Serial.println(en.readRaw(), 0);
  }
  else
  {
    invalidData();
  }
}
void ACTION_system(int data)
{
  switch(data)
  {
    case 0:
      Serial.println(F("Reboot Actuator"));
      resetFunc();  
      break;
    case 1:
      for(uint8_t index=0; index<configLen; index++)
      {
        writeEEPROM(eepromAddress[index], *bufVal[index]);
      }
      Serial.println(F("Save Data"));
      break;
    default:
      invalidData();
      break;
  }
}
void ACTION_returnData(int data)
{
  switch(data)
  {
    case 0:
      for(uint8_t index=1; index<configLen; index++)
      {
        Serial.print(configPrint[index]); Serial.print("\t");
        Serial.println(*bufVal[index], dp[index]);
      }
      break;
    case 1:
      Serial.write('/');
      for(uint8_t index=1; index<configLen; index++)
      {
        Serial.print("\t");
        Serial.print(*bufVal[index], dp[index]);
      }
      Serial.println();
      break;
    default:
      invalidData();
      break;
  }
}
void ACTION_goControlMode(int data)
{
  if(data == 1)
  {
    if(temp.getTemperature() > tempLimit)
    {  
      Serial.println(F("Over-Temperature"));
      return;
    }
    ACTION_motorEnable(1);  
    configMode = false;
    Serial.println(F("Control Mode"));
  }
  else
  {
    invalidData();
  }
}
/* set and save parameter  */
void configLoop()
{ 
  while(configMode)
  {  
    // local serial event
    if(Serial.find(configHeader))
    {  
      addrBuf = Serial.parseInt();
      dataBuf = Serial.parseFloat();
    }
      
    // accept data
    if(addrBuf > -1)
    {
      // config data
      if(addrBuf < configLen) 
      {
        // return data
        if(dataBuf == -1)
        {
          Serial.print(configPrint[addrBuf]); 
          Serial.print("\t");
          Serial.println(*bufVal[addrBuf], dp[addrBuf]);
        }
        // set data
        else
        {
          if(dataBuf >= constn[addrBuf][0] && dataBuf <= constn[addrBuf][1])          
          {
            *bufVal[addrBuf] = dataBuf;  
            
            /* Config data inside class */
            if(addrBuf == 13) 
            // Sensitive setting, it should not be set register multiple times.
            { 
              en.setDirCount(encoderDir);      
            }
            sv.setVoltageSource(Vs);
            sv.setNewDir(motorDir);
            sv.setPWMResolution(pwmRes);
            en.setPolePair(pp);
            en.setRotorOffsetCW(rotorOffsetCW);
            en.setRotorOffsetCCW(rotorOffsetCCW);
            en.setShaftOffset(shaftOffset);
    
            Serial.print(configPrint[addrBuf]); Serial.print("\t");
            Serial.println(*bufVal[addrBuf], dp[addrBuf]);        
          }
          else  // deny data
          {
            invalidData();                
          }
        }
      }
      // actuator action
      else
      {
        switch(addrBuf)
        {
          case 26:
            ACTION_motorEnable(dataBuf);
            break;
          case 27:
            ACTION_openLoopDrive(dataBuf);
            break;
          case 28:
            ACTION_closeLoopDrive(dataBuf);
            break;
          case 29:
            ACTION_motorAlign(dataBuf);
            break;
          case 30:
            ACTION_system(dataBuf);
            break;
          case 31:
            ACTION_returnData(dataBuf);
            break;
          case 32:
            ACTION_goControlMode(dataBuf);
            break;  
          default:
            invalidData();  
            break;
        }
      }
      // reset buffer
      addrBuf = -1;
      dataBuf = -1;
    }
  }
}  
/* prepare motor control */
void prepareControlLoop(float magnitude, float gap, uint16_t timeOut)
{
  unsigned long startTime = millis();
  const float full = 16384;
  const float half = 8192;
  const float real = en.readRaw();
  float shaftOffset2;
  int8_t dir;
    
  if(shaftOffset < half)  
  {
    shaftOffset2 = shaftOffset + half;
    if(real >= 0 && real <= shaftOffset) dir = -1;  
    else if(real >= shaftOffset2 && real <= full) dir = -1; 
    else if(real >= shaftOffset && real <= shaftOffset2)  dir = 1;     
  }
  else
  {
    shaftOffset2 = shaftOffset - half;
    if(real >= 0 && real <= shaftOffset2) dir = 1;  
    else if(real >= shaftOffset && real <= full) dir = 1; 
    else if(real >= shaftOffset2 && real <= shaftOffset)  dir = -1;     
  }
  
  while(true)
  {    
    en.update();
    pos_actual = en.readRaw();
    
    if(pos_actual > shaftOffset - gap && pos_actual < shaftOffset + gap)
    {
      Serial.println("Motor Ready");
      en.zeroShaftAngle();
      break;
    }
    if(millis()-startTime > timeOut)
    {
      safetyEvent();  
    }

    angle = en.getRotorAngle(dir>0? CW: CCW) - In90Deg * dir;  
    sv.driveVector(magnitude, angle);
  }
}
/* main motor control */
void controlLoop()
{
//  commandVel = command;
  commandPos = command * GearRatio;
//  commandPos *= degToRad;
  pos_setpoint = commandPos;
/*_________________________________________________________________________________*/
#ifdef EncoderEnable
  /* absolute encoder update */    
  en.update();
  /* get shaft velocity -> rad/s unit */
//  vel_actual = en.getShaftVel();
//  vel.input((int)vel_actual);
  /* get vel with average method */
//  if(avgCount < maxAvgCount)
//  {
//    sumVel += en.getShaftVel();
//    avgCount++;
//  }
//  else
//  {
//    vel_actual = sumVel / (float)maxAvgCount; 
//    sumVel = 0;
//    avgCount = 0;
//    vel.input((int)vel_actual);
//  }
  /* get vel with sampling method */
//  if(millis - prevTimeSampling > 50)
//  {
//    vel_actual = en.getShaftVel();
//    vel.input((int)vel_actual);
//    prevTimeSampling = millis();
//  }
    
  /* get shaft position -> rad */
  pos_actual = en.getShaftAngle();
#endif
/*_________________________________________________________________________________*/
#ifdef PositionControl

  pos_error = pos_setpoint - pos_actual;      
  if(pos_error > -posGap && pos_error < posGap) pos_error = 0;
  pos_error = constrain(pos_error, -PI, PI);  
  pos_integral += pos_error;
       if(pos_integral >  pos_I_Limit)  pos_integral =  pos_I_Limit;
  else if(pos_integral < -pos_I_Limit)  pos_integral = -pos_I_Limit;

  /* vel output */
//  commandVel = pos_Kp * pos_error + pos_Ki * pos_integral;  
//  commandVel = constrain(commandVel, -maxVel, maxVel);   

  /* final output */
  magnitude = abs(pos_Kp * pos_error + pos_Ki * pos_integral);
  angle = en.getRotorAngle(pos_setpoint>0? CCW: CW) + In90Deg * pos_error / abs(pos_error);  

#endif
/*_________________________________________________________________________________*/
#ifdef TrajectoryControl

  /* distance calculate */
//  difPos = abs(commandPos - pos_setpoint);
//
//  /* acceleration generate */
//  if(difPos > criticalStartAngle && difPos > criticalStopAngle)
//  {
//    if(millis()-periodAccelCounter >= periodAccel)
//    {
//      periodAccelCounter = millis();
//      velRev += accelRev;
//    } 
//  }
//  else if(difPos <= criticalStopAngle)
//  {
//    if(millis()-periodAccelCounter >= periodAccel)
//    {
//      periodAccelCounter = millis();
//      velRev -= decelRev;
//    }
//  }
//  velRev = constrain(velRev, 0, velRevMax);

  // feed-forward velocity without acelleration
  velRev = velRevMax;
  
  /* velocity generate */    
  if(commandPos > pos_setpoint + posGap)
  {
    if(millis()-periodVelCounter >= periodVel)
    {
      periodVelCounter = millis();
      pos_setpoint += velRev;
//      Serial.println(pos_setpoint, 4);
    }
  }
  else if(commandPos < pos_setpoint - posGap)
  {
    if(millis()-periodVelCounter >= periodVel)
    {
      periodVelCounter = millis();
      pos_setpoint -= velRev;
//      Serial.println(pos_setpoint, 4);
    }
  }
  
  /* pure position control */
  pos_error = pos_setpoint - pos_actual;      
  if(pos_error > -posGap && pos_error < posGap) pos_error = 0;
  pos_error = constrain(pos_error, -PI, PI);  
  pos_integral += pos_error;
       if(pos_integral >  pos_I_Limit)  pos_integral =  pos_I_Limit;
  else if(pos_integral < -pos_I_Limit)  pos_integral = -pos_I_Limit;

  /* final output */ 
  magnitude = abs(pos_Kp * pos_error + pos_Ki * pos_integral);
  angle = en.getRotorAngle(pos_setpoint>0? CCW: CW) + In90Deg * pos_error / abs(pos_error);  
#endif
/*_________________________________________________________________________________*/
#ifdef VelocityControl
  /* sinusoidal signal */
//  vel_setpoint = limitVel * sin(0.8 * millis() / 1000.0f);
  /* step signal */
//  vel_setpoint = -6.0f;
  vel_setpoint = constrain(vel_setpoint, -limitVel, limitVel);  
  vel_error = fabs(vel_setpoint) - fabs(vel.output());   
//  vel_error = fabs(vel_setpoint) - fabs(vel_actual);   
  vel_integral += vel_error;
       if(vel_integral >  vel_I_Limit)  vel_integral =  vel_I_Limit;
  else if(vel_integral < -vel_I_Limit)  vel_integral = -vel_I_Limit;
  magnitude = vel_error * vel_Kp + vel_integral * vel_Ki;
  angle = en.getRotorAngle(vel_setpoint>0? CCW: CW) + In90Deg * vel_setpoint / abs(vel_setpoint);
#endif
/*_________________________________________________________________________________*/
#ifdef OpenLoopDriveCW
  magnitude = 12.0;
  angle -= 0.436f; // -> 25.0 deg       
#endif
/*_________________________________________________________________________________*/
#ifdef OpenLoopDriveCCW
  magnitude = 12.0;
  angle += 0.436f; // -> 25.0 deg       
#endif
/*_________________________________________________________________________________*/
#ifdef CloseLoopDriveCW
  magnitude = 6.0f;
  angle = en.getRotorAngle(CW) - In90Deg;
#endif
/*_________________________________________________________________________________*/
#ifdef CloseLoopDriveCCW
  magnitude = 6.0f;
  angle = en.getRotorAngle(CCW) + In90Deg;  
#endif
/*_________________________________________________________________________________*/
  /* general drive */
  sv.driveVector(magnitude, angle);
/*_________________________________________________________________________________*/
  /* motor safety */
#ifdef currentSafety  
  // current sensor
  currentLPF.input(getCurrent);
  if(currentLPF.output() > currentLimit) 
  {
    Serial.println(F("Over-Current"));
    safetyEvent();
  }
#endif

#ifdef tempSafety
  // temp sensor
  if(millis()-prevTimeSafety >= safetyPeriod)
  {
    prevTimeSafety = millis();
    if(temp.getTemperature() > tempLimit)
    {  
      Serial.println(F("Over-Temperature"));
      safetyEvent();
    }
  }
#endif
/*_________________________________________________________________________________*/  
  /* velocity control debug */
//  Serial.print(String(commandVel) + "\t");
//  Serial.print(String(vel_setpoint) + "\t");
//  Serial.println(vel_actual);
//  Serial.println(vel.output());
/*_________________________________________________________________________________*/
  /* trajectory control debug */
//  Serial.print(String(commandPos) + "\t");
//  Serial.println(velRev, 6);
//  Serial.print(pos_setpoint, 4);  Serial.print("\t");
//  Serial.print(pos_actual, 4);   Serial.print("\t");
//  Serial.println(millis() / 1000.0f, 3);
 /* position control */
  
}

//////////////////////////////////////////////////////////////////////////////////////

void setup()
{ 
  Serial.begin(baudrate);
  Serial.setTimeout(timeout);
  delay(1000);  

  callEEPROM(); 
  configLoop();

  /* sensor init */
  ADC_thres = analogRead(currentPin);
  temp.begin();

  prepareControlLoop(7.5, 50, 5000);
   
//#ifdef PositionControl
//  Serial.println(F("Position Control"));
//#elif TrajectoryControl
//  Serial.println(F("Trajectory Control"));
//#elif VelocityControl
//  Serial.println(F("Velocity Control"));
//#endif
}
void loop() 
{ 
  controlLoop();
}





/**/
