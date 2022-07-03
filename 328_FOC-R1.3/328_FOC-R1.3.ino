
/** 12-06-22
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

/* position control */
float magnitude, angle;
float pos_setpoint = 0, commandPos = 0, pos_actual,
      pos_error, pos_integral; 
#define posGap 0.00349  
// 1.0 => 0.01740 rad
// 0.5 => 0.00873 rad
// 0.2 => 0.00349 rad
#define pos_I_Limit 5000.0f

#define periodVel 2     // ms unit
unsigned long periodVelCounter = millis();
#define velDrive limitVel * GearRatio // rad/s unit
#define velRevMax velDrive * periodVel / 1000.0f
float velRev = velRevMax;
float difPos = 0;

#define periodAccel 2   // ms unit
unsigned long periodAccelCounter = millis();
#define accelDrive accel * GearRatio  // rad/s^2 unit
#define decelDrive decel * GearRatio  // rad/s^2 unit
#define accelRev accelDrive * periodAccel / 1000.0f * (periodVel / 1000.0f) 
#define decelRev decelDrive * periodAccel / 1000.0f * (periodVel / 1000.0f)

#define criticalStartAngle  (velDrive * velDrive) / (2.0f * accelDrive)
#define criticalStopAngle   (velDrive * velDrive) / (2.0f * decelDrive)



/* velocity control */
#include <Filters.h>
FilterOnePole vel(LOWPASS, 1.4);  //  old -> 0.6, NEW -> 1.2
float vel_setpoint = 0, commandVel = 0, vel_actual,
      vel_error, vel_integral;
#define velGap 0.002 
#define vel_I_Limit 10.0f // old -> 3
//#define periodAccel 2   // ms unit
//unsigned long periodAccelCounter = millis();
//#define accelRev accel * periodAccel / 1000.0f
//#define decelRev decel * periodAccel / 1000.0f
#define maxAvgCount 5
uint8_t avgCount = 0;
float sumVel;
unsigned long prevTimeSampling = millis();

/* mode options */
//#define RotorPositionAlignment
#define EncoderEnable
//#define PositionControl
#define PositionControlV2
//#define VelocityControl
//#define OpenLoopDriveCW
//#define OpenLoopDriveCCW
//#define CloseLoopDriveCW
//#define CloseLoopDriveCCW

void setup()
{ 
  Serial.begin(baudrate);
  Serial.setTimeout(timeout);
  while(!Serial);
  Serial.println(F("*** Welcome to Back-End! ***"));
  
  callEEPROMParameter(); 
  configState(); 
}

void loop() 
{ 
  controlRoutine();
}
/* software reset */
void(*resetFunc) (void) = 0;
/* parameter update */
void callEEPROMParameter()
{
  Vs              = readEEPROM(addr__VoltSource);
  pp              = readEEPROM(addr__motorPolePair);
  motorDir        = readEEPROM(addr__motorDirection);
  pwmRes          = readEEPROM(addr__pwmResolution);
  timeTest        = readEEPROM(addr__timeTest);
  voltTest        = readEEPROM(addr__voltTest);
  velTest         = readEEPROM(addr__velTest);
  encoderDir      = readEEPROM(addr__encoderDirection);
  rotorOffsetCW   = readEEPROM(addr__rotorOffsetCW);
  rotorOffsetCCW  = readEEPROM(addr__rotorOffsetCCW);
  shaftOffset     = readEEPROM(addr__shaftOffset);
  GearRatio       = readEEPROM(addr__GearRatio);
  limitVel = readEEPROM(addr__limitVelocity);  
  maxVel = readEEPROM(addr__maxVelocity);
  minVel = readEEPROM(addr__minVelocity);
  accel = readEEPROM(addr__Acceleration);
  decel = readEEPROM(addr__Deceleration);
  pos_Kp = readEEPROM(addr__posPGain);
  pos_Ki = readEEPROM(addr__posIGain);
  vel_Kp = readEEPROM(addr__velPGain);
  vel_Ki = readEEPROM(addr__velIGain);
      
  /* config class */
  sv.setVoltageSource(Vs);
  sv.setNewDir(motorDir);
  sv.setPWMResolution(pwmRes);
  en.setPolePair(pp);
  en.setDirCount(encoderDir);    
  en.setRotorOffsetCW(rotorOffsetCW);
  en.setRotorOffsetCCW(rotorOffsetCCW);
  en.setShaftOffset(shaftOffset);
}
/* serial communication interrupt function */
void serialEvent()
{    
  if(Serial.find(generalHeader))
  {  
    command = Serial.parseFloat();
    if(Serial.parseInt()) 
    {    
      sv.motorDisable();
      sv.resetPwm();
      resetFunc();
    }
  }  
}    
/** config mode 
 *  1   = __motorEnable       (Action)  (0:1)
 *  2   = __VoltSource        (Config)  (5-26:-1)  
 *  3   = __motorPolePair     (Config)  (2-100:-1)
 *  4   = __motorDirection    (Config)  (0:1:-1)
 *  5   = __dataLoggerAction  (Action)  (0:1:-1)
 *  6   = __pwmResolution     (Config)  (8-1023)
 *  7   = __timeTest          (Config)  (0.1-10:-1)
 *  8   = __voltTest          (Config)  (10-100:-1)
 *  9   = __velTest           (Config)  (0.05-6:-1)
 *  10  = __OpenLoopDrive     (Config)  (0:1)
 *  11  = __encoderDirection  (Config)  (0:1:-1)
 *  12  = __rotorOffsetCW     (Config)  (0-16383:-1)
 *  13  = __rotorOffsetCCW    (Config)  (0-16383:-1)
 *  14  = __shaftOffset       (Config)  (0-16383:-1)
 *  15  = __GearRatio         (Config)  (0.02-50:-1)
 *  16  = __rotorAlignment    (Action)  (0:1)
 *  17  = __CloseLoopDrive    (Action)  (0:1)
 *  18  = __limitVelocity     (Config)  (0.1-1000)
 *  19  = __maxVelocity       (Config)  (0.1-1000)  
 *  20  = __minVelocity       (Config)  (0.1-1000)
 *  21  = __Acceleration      (Config)  (0.1-1000)
 *  22  = __Deceleration      (Config)  (0.1-1000)
 *  23  = __posPGain          (Config)  (0-1000)
 *  24  = __posIGain          (Config)  (0-1000)
 *  25  = __velPGain          (Config)  (0-1000)
 *  26  = __velIGain          (Config)  (0-1000)
 *  27  = 
 *  28  = 
 */ 
void configState()
{   
  while(configMode)
  {
    if(Serial.find(configHeader))
    {
      addr = Serial.parseInt();
      data = Serial.parseFloat();
    }
    switch(addr)
    {  
      case 0: 
        break;
      case __motorEnable:
        Serial.print(F("__motorEnable:\t\t"));
        motorEnableFunc(data);
        break;
      case __VoltSource:
        Serial.print(F("__VoltSource:\t\t"));
        VoltSourceFunc(data);     
        break;
      case __motorPolePair:
        Serial.print(F("__motorPolePair:\t"));
        motorPolePairFunc(data);
        break;
      case __motorDirection:
        Serial.print(F("__motorDirection:\t"));
        motorDirectionFunc(data);
        break;
      case __dataLoggerAction: 
        Serial.print(F("__dataLoggerAction:\t"));
        dataLoggerActionFunc(data);
        break;
      case __pwmResolution:
        Serial.print(F("__pwmResolution:\t"));
        pwmResolutionFunc(data);
        break;
      case __timeTest:
        Serial.print(F("__timeTest:\t\t"));
        timeTestFunc(data);
        break;
      case __voltTest:
        Serial.print(F("__voltTest:\t\t"));
        voltTestFunc(data);
        break;
      case __velTest:
        Serial.print(F("__velTest:\t\t"));
        velTestFunc(data);
        break;
      case __OpenLoopDrive:
        Serial.print(F("__OpenLoopDrive:\t"));
        OpenLoopDriveFunc(data);
        break;
      case __encoderDirection:
        Serial.print(F("__encoderDirection:\t"));
        encoderDirectionFunc(data);
        break;
      case __rotorOffsetCW:
        Serial.print(F("__rotorOffsetCW:\t"));
        rotorOffsetCWFunc(data);
        break;
      case __rotorOffsetCCW:
        Serial.print(F("__rotorOffsetCCW:\t"));
        rotorOffsetCCWFunc(data);
        break;
      case __shaftOffset:
        Serial.print(F("__shaftOffset:\t\t"));
        shaftOffsetFunc(data);
        break; 
      case __GearRatio:
        Serial.print(F("__GearRatio:\t\t"));
        GearRatioFunc(data);
        break;
      case __rotorAlignment:
        Serial.print(F("__rotorAlignment:\t"));
        rotorAlignmentFunc(data);
        break;    
      case __CloseLoopDrive:
        Serial.print(F("__CloseLoopDrive:\t"));
        CloseLoopDriveFunc(data);     
        break;
      case __limitVelocity:
        Serial.print(F("__limitVelocity\t\t"));
        limitVelocityFunc(data);
        break;
      case __maxVelocity:
        Serial.print(F("__maxVelocity\t\t"));
        maxVelocityFunc(data);
        break;
      case __minVelocity:
        Serial.print(F("__minVelocity\t\t"));
        minVelocityFunc(data);
        break;   
      case __Acceleration:
        Serial.print(F("__Acceleration\t\t"));
        AccelerationFunc(data);
        break;   
      case __Deceleration:
        Serial.print(F("__Deceleration\t\t"));
        DecelerationFunc(data);
        break;    
      case __posPGain:
        Serial.print(F("__posPGain\t\t"));
        posPGainFunc(data);
        break;
      case __posIGain:
        Serial.print(F("__posIGain\t\t"));
        posIGainFunc(data);
        break;
      case __velPGain:
        Serial.print(F("__velPGain\t\t"));
        velPGainFunc(data);
        break;
      case __velIGain:
        Serial.print(F("__velIGain\t\t"));
        velIGainFunc(data);
        break;
      default:
        Serial.println(F("Invalid address!"));
        break;        
    }
    addr = 0; 
    data = -1;
  }
} 
/* sub function in config function */
void printInvalidData()
{
  Serial.println(F("Invalid data!"));
}
void motorEnableFunc(int8_t cmd)
{
  switch(cmd)
  {
    case 0: // disable
      sv.motorDisable();
      Serial.println(F("Disable"));
      break;
    case 1: // enable
      sv.outputInit();
      sv.motorEnable();   
      Serial.println(F("Enable"));
      break;  
    default:
      printInvalidData();
      break;
  }
}
void VoltSourceFunc(float cmd)
{
  if(cmd > 4 && cmd < 26)
  {
    Vs = cmd; 
    Serial.println(String(Vs) + " V");
  }
  else if(cmd == -1)
  {
    Serial.println(String(Vs) + " V");
  }
  else
  {
    printInvalidData();
  }
}
void motorPolePairFunc(int8_t cmd)
{
  if(cmd > 1 && cmd < 101)
  {
    pp = cmd;   
    Serial.println(pp);
  }
  else if(cmd == -1)
  {
    Serial.println(pp);
  }
  else
  {
    printInvalidData();
  }
}
void motorDirectionFunc(int8_t cmd)
{
  if(cmd == 0)
  {
    motorDir = false;
    Serial.println(F("Non invert"));
  }
  else if(cmd == 1)
  {
    motorDir = true;
    Serial.println(F("Invert"));
  }
  else if(cmd == -1)
  {
    Serial.println(motorDir? "Invert": "Non invert");
  }
  else
  {
    printInvalidData();
  }
}
void dataLoggerActionFunc(int8_t cmd)
{
  switch(cmd)
  {
    case -1:  // reset
      Serial.println(F("Reset"));
      resetFunc();
      break;
    case 0: // go to void loop
      configMode = false;
      Serial.println(F("Go to routine"));
      break;
    case 1: // save
      writeEEPROM(addr__VoltSource, Vs);
      writeEEPROM(addr__motorPolePair, pp);
      writeEEPROM(addr__motorDirection, motorDir);
      writeEEPROM(addr__pwmResolution, pwmRes);
      writeEEPROM(addr__timeTest, timeTest);
      writeEEPROM(addr__voltTest, voltTest);
      writeEEPROM(addr__velTest, velTest);
      writeEEPROM(addr__encoderDirection, encoderDir);
      writeEEPROM(addr__rotorOffsetCW, rotorOffsetCW);
      writeEEPROM(addr__rotorOffsetCCW, rotorOffsetCCW);
      writeEEPROM(addr__shaftOffset, shaftOffset);
      writeEEPROM(addr__GearRatio, GearRatio);
      writeEEPROM(addr__limitVelocity, limitVel);  
      writeEEPROM(addr__maxVelocity, maxVel);
      writeEEPROM(addr__minVelocity, minVel);
      writeEEPROM(addr__Acceleration, accel);
      writeEEPROM(addr__Deceleration, decel);
      writeEEPROM(addr__posPGain, pos_Kp);
      writeEEPROM(addr__posIGain, pos_Ki);
      writeEEPROM(addr__velPGain, vel_Kp);
      writeEEPROM(addr__velIGain, vel_Ki);
      Serial.println(F("Save"));
      break;  
    default:
      printInvalidData();
      break;
  }
}
void pwmResolutionFunc(int16_t cmd)
{
  if(cmd > 7 && cmd < 1024)
  {
    pwmRes = cmd;
    Serial.println(pwmRes);
  }
  else if(cmd == -1)
  {
    Serial.println(pwmRes);
  }
  else
  {
    printInvalidData();
  }
}
void timeTestFunc(float cmd)
{
  if(cmd > 0.09 && cmd < 10.01)
  {
    timeTest = cmd; 
    Serial.println(String(timeTest) + " s");
  }
  else if(cmd == -1)
  {
    Serial.println(String(timeTest) + " s");
  }
  else
  {
    printInvalidData();
  }
}
void voltTestFunc(int8_t cmd)
{
  if(cmd > 9 && cmd < 101)
  {
    voltTest = cmd; 
    Serial.println(String(voltTest) + " %");
  }
  else if(cmd == -1)
  {
    Serial.println(String(voltTest) + " %");
  }
  else
  {
    printInvalidData();
  }  
}
void velTestFunc(float cmd)
{
  if(cmd > 0.049 && cmd < 6.01)
  {
    velTest = cmd; 
    Serial.println(velTest);
  }
  else if(cmd == -1)
  {
    Serial.println(velTest);
  }
  else
  {
    printInvalidData();
  }    
}    
void OpenLoopDriveFunc(int8_t cmd)
{
  if(cmd == 0 || cmd == 1)
  {
    unsigned long prevTime = millis();
    float magnitude = Vs * voltTest / 100.0f;
    float angle = 0;
    if(cmd == 0)
    {
      Serial.println(F("CCW"));
      cmd = 1;
    }
    else
    {
      Serial.println(F("CW")); 
      cmd = -1;
    }
    while(true)
    {
      if(millis()-prevTime > timeTest * 1000.0f)  break;
      sv.driveVector(magnitude, angle);
      angle = angle + velTest * cmd;
    }   
    sv.resetPwm();
  }
  else
  {
    printInvalidData();
  }
}
void encoderDirectionFunc(int8_t cmd)
{
  switch(cmd)
  {
    case 0: // non invert
      encoderDir = false;
      Serial.println(F("Non invert"));
      break;
    case 1: // invert
      encoderDir = true;
      Serial.println(F("Invert"));
      break;  
    case -1:
      Serial.println(encoderDir? "Invert": "Non invert");
      break;    
    default:
      printInvalidData();
      break;
  }
}
void rotorOffsetCWFunc(int16_t cmd)
{
  if(cmd > -1 && cmd < 16384)
  {
    rotorOffsetCW = cmd;
    Serial.println(rotorOffsetCW);
  }
  else if(cmd == -1)
  {
    Serial.println(rotorOffsetCW);
  }
  else
  {
    printInvalidData();
  }
}
void rotorOffsetCCWFunc(int16_t cmd)
{
  if(cmd > -1 && cmd < 16384)
  {
    rotorOffsetCCW = cmd;
    Serial.println(rotorOffsetCCW);
  }
  else if(cmd == -1)
  {
    Serial.println(rotorOffsetCCW);
  }
  else
  {
    printInvalidData();
  }  
}
void shaftOffsetFunc(int16_t cmd)
{
  if(cmd > -1 && cmd < 16384)
  {
    shaftOffset = cmd;
    Serial.println(shaftOffset);
  }
  else if(cmd == -1)
  {
    Serial.println(shaftOffset);
  }
  else
  {
    printInvalidData();
  }    
}
void GearRatioFunc(float cmd)
{
  if(cmd > 0.019 && cmd < 50.1)
  {
    GearRatio = cmd;
    Serial.println(GearRatio);
  }
  else if(cmd == -1)
  {
    Serial.println(GearRatio);
  }
  else
  {
    printInvalidData();
  }  
}
void rotorAlignmentFunc(int8_t cmd)
{
  if(cmd == 1)
  {
    float magnitude = Vs * voltTest / 100.0f;
    float angle = 0;
    sv.driveVector(magnitude, angle);
    delay(timeTest * 1000.0f);
    Serial.println("Force zero position -> " + String(en.readRaw()));
    sv.resetPwm();
  }
  else if(cmd == 0)
  {
    Serial.println("Freestyle position -> " + String(en.readRaw()));
  }
  else 
  {
    printInvalidData();
  }
}
void CloseLoopDriveFunc(int8_t cmd)
{
  if(cmd == 0 || cmd == 1)
  {
    unsigned long prevTime = millis();
    float magnitude = Vs * voltTest / 100.0f;
    float angle = 0;
    if(cmd == 0)
    {
      Serial.println(F("CCW"));
      cmd = 1;
    }
    else
    {
      Serial.println(F("CW")); 
      cmd = -1;
    }
    while(true)
    {
      if(millis()-prevTime > timeTest * 1000.0f)  break;
      en.update();
      angle = en.getRotorAngle(cmd>0? CCW: CW) + In90Deg * cmd / abs(cmd);
      sv.driveVector(magnitude, angle);
    }   
    sv.resetPwm();
  }
  else
  {
    printInvalidData();
  }
}
void limitVelocityFunc(float cmd)
{
  if(cmd > 0.09 && cmd < 1000)
  {
    limitVel = cmd;
    Serial.println(String(limitVel) + " rad/s");
  }
  else if(cmd == -1)
  {
    Serial.println(String(limitVel) + " rad/s");
  }
  else
  {
    printInvalidData();
  }
}
void maxVelocityFunc(float cmd)
{
  if(cmd > 0.09 && cmd < 1000)
  {
    maxVel = cmd;
    Serial.println(String(maxVel) + " rad/s");
  }
  else if(cmd == -1)
  {
    Serial.println(String(maxVel) + " rad/s");
  }
  else
  {
    printInvalidData();
  }
}
void minVelocityFunc(float cmd)
{
  if(cmd > 0.09 && cmd < 1000)
  {
    minVel = cmd;
    Serial.println(String(minVel) + " rad/s");
  }
  else if(cmd == -1)
  {
    Serial.println(String(minVel) + " rad/s");
  }
  else
  {
    printInvalidData();
  }
}
void AccelerationFunc(float cmd)
{
  if(cmd > 0.09 && cmd < 1000)
  {
    accel = cmd;
    Serial.println(String(accel) + " rad/s^2");
  }
  else if(cmd == -1)
  {
    Serial.println(String(accel) + " rad/s^2");
  }
  else
  {
    printInvalidData();
  }
}
void DecelerationFunc(float cmd)
{
  if(cmd > 0.09 && cmd < 1000)
  {
    decel = cmd;
    Serial.println(String(decel) + " rad/s^2");
  }
  else if(cmd == -1)
  {
    Serial.println(String(decel) + " rad/s^2");
  }
  else
  {
    printInvalidData();
  }
}
void posPGainFunc(float cmd)
{
  if(cmd > 0 && cmd < 1000)
  {
    pos_Kp = cmd;
    Serial.println(pos_Kp, 3);
  }
  else if(cmd == -1)
  {
    Serial.println(pos_Kp, 3);
  }
  else
  {
    printInvalidData();
  }
}
void posIGainFunc(float cmd)
{
  if(cmd > 0 && cmd < 1000)
  {
    pos_Ki = cmd;
    Serial.println(pos_Ki, 6);
  }
  else if(cmd == -1)
  {
    Serial.println(pos_Ki, 6);
  }
  else
  {
    printInvalidData();
  }
}
void velPGainFunc(float cmd)
{
  if(cmd > 0 && cmd < 1000)
  {
    vel_Kp = cmd;
    Serial.println(vel_Kp, 3);
  }
  else if(cmd == -1)
  {
    Serial.println(vel_Kp, 3);
  }
  else
  {
    printInvalidData();
  } 
}
void velIGainFunc(float cmd)
{
  if(cmd > 0 && cmd < 1000)
  {
    vel_Ki = cmd;
    Serial.println(vel_Ki, 6);
  }
  else if(cmd == -1)
  {
    Serial.println(vel_Ki, 6);
  }
  else
  {
    printInvalidData();
  }
}
/* main motor control program */
void controlRoutine()
{
  commandPos = command * GearRatio;
//  commandPos *= degToRad;
  
//  commandVel = command;
//  pos_setpoint = commandPos * degToRad;

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

#ifdef PositionControl
  pos_error = pos_setpoint - pos_actual;      
  if(pos_error > -posGap && pos_error < posGap) pos_error = 0;
  pos_error = constrain(pos_error, -PI, PI);  
  pos_integral += pos_error;
       if(pos_integral >  pos_I_Limit)  pos_integral =  pos_I_Limit;
  else if(pos_integral < -pos_I_Limit)  pos_integral = -pos_I_Limit;

  commandVel = pos_Kp * pos_error + pos_Ki * pos_integral;  
  commandVel = constrain(commandVel, -maxVel, maxVel);   
#endif

#ifdef PositionControlV2

  difPos = abs(commandPos - pos_setpoint);


  
  if(difPos > criticalStartAngle && difPos > criticalStopAngle)
  {
    if(millis()-periodAccelCounter >= periodAccel)
    {
      periodAccelCounter = millis();
      velRev += accelRev;
    } 
  }
  else if(difPos <= criticalStopAngle)
  {
    if(millis()-periodAccelCounter >= periodAccel)
    {
      periodAccelCounter = millis();
      velRev -= decelRev;
    }
  }
  velRev = constrain(velRev, 0, velRevMax);


      
  if(commandPos > pos_setpoint + posGap)
  {
    if(millis()-periodVelCounter >= periodVel)
    {
      periodVelCounter = millis();
      pos_setpoint += velRev;
    }
  }
  else if(commandPos < pos_setpoint - posGap)
  {
    if(millis()-periodVelCounter >= periodVel)
    {
      periodVelCounter = millis();
      pos_setpoint -= velRev;
    }
  }
  
  
  
  pos_error = pos_setpoint - pos_actual;      
  if(pos_error > -posGap && pos_error < posGap) pos_error = 0;
  pos_error = constrain(pos_error, -PI, PI);  
  pos_integral += pos_error;
       if(pos_integral >  pos_I_Limit)  pos_integral =  pos_I_Limit;
  else if(pos_integral < -pos_I_Limit)  pos_integral = -pos_I_Limit;

  magnitude = abs(pos_Kp * pos_error + pos_Ki * pos_integral);
  angle = en.getRotorAngle(pos_setpoint>0? CCW: CW) + In90Deg * pos_error / abs(pos_error);
  
#endif

#ifdef VelocityControl
  /* sinusoidal signal */
//  vel_setpoint = limitVel * sin(0.8 * millis() / 1000.0f);
  /* step signal */
//  vel_setpoint = -6.0f;
  /* acceleration generate */
  if(commandVel > vel_setpoint + velGap)
  {
    if(millis()-periodAccelCounter > periodAccel)
    {
      vel_setpoint += accelRev;
      periodAccelCounter = millis();
    }
  }
  else if(commandVel < vel_setpoint - velGap)
  {
    if(millis()-periodAccelCounter > periodAccel)
    {
      vel_setpoint -= decelRev;
      periodAccelCounter = millis();
    }    
  }
  vel_setpoint = constrain(vel_setpoint, -limitVel, limitVel);  
  vel_error = fabs(vel_setpoint) - fabs(vel.output());   
//  vel_error = fabs(vel_setpoint) - fabs(vel_actual);   
  vel_integral += vel_error;
       if(vel_integral >  vel_I_Limit)  vel_integral =  vel_I_Limit;
  else if(vel_integral < -vel_I_Limit)  vel_integral = -vel_I_Limit;
  magnitude = vel_error * vel_Kp + vel_integral * vel_Ki;
  angle = en.getRotorAngle(vel_setpoint>0? CCW: CW) + In90Deg * vel_setpoint / abs(vel_setpoint);
#endif

#ifdef OpenLoopDriveCW
  magnitude = 12.0;
  angle -= 0.436f; // -> 25.0 deg       
#endif

#ifdef OpenLoopDriveCCW
  magnitude = 12.0;
  angle += 0.436f; // -> 25.0 deg       
#endif

#ifdef CloseLoopDriveCW
  magnitude = 6.0f;
  angle = en.getRotorAngle(CW) - In90Deg;
#endif
 
#ifdef CloseLoopDriveCCW
  magnitude = 6.0f;
  angle = en.getRotorAngle(CCW) + In90Deg;  
#endif
  
  /* general drive */
  sv.driveVector(magnitude, angle);

  /* print */
//  Serial.print(String(commandVel) + "\t");
//  Serial.print(String(vel_setpoint) + "\t");
//  Serial.println(vel_actual);
//  Serial.println(vel.output());
  
//  Serial.print(String(vel.output()) + "\t");


  /* position control v2 debug */
//  Serial.print(String(commandPos) + "\t");
//  Serial.println(velRev, 6);
  Serial.print(pos_setpoint, 4);  Serial.print("\t");
  Serial.print(pos_actual, 4);   Serial.print("\t");
  Serial.println(millis() / 1000.0f, 3);

}



//
