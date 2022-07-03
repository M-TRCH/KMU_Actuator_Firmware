
/** 05-06-22
 *  Created by Teerachot Mueangchamnong 
 *  Brushless Motor Control for Quadruped Leg Project
 *  Mechatronics Engineering Technology, KMUTNB
 */

#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H
#include "Arduino.h"
#include <EEPROM.h>

#define configHeader '/'
#define generalHeader '*'
#define baudrate 2000000
#define timeout 1
volatile int addr = 0;
volatile float data = -1;
volatile float command = 0;
volatile boolean configMode = true;

/* convert data type and eeprom manager */
union FLOAT_BYTE
{
  float Float;
  byte Byte[4];
} FloatToByte, ByteToFloat;
void writeEEPROM(int addr, float data)
{
  FloatToByte.Float = data;
  EEPROM.update(addr, FloatToByte.Byte[0]);
  EEPROM.update(addr + 1, FloatToByte.Byte[1]);
  EEPROM.update(addr + 2, FloatToByte.Byte[2]);
  EEPROM.update(addr + 3, FloatToByte.Byte[3]);
}
float readEEPROM(int addr)
{
  ByteToFloat.Byte[0] = EEPROM.read(addr);
  ByteToFloat.Byte[1] = EEPROM.read(addr + 1);
  ByteToFloat.Byte[2] = EEPROM.read(addr + 2);
  ByteToFloat.Byte[3] = EEPROM.read(addr + 3);
  return ByteToFloat.Float;
}

/* command address */
#define __motorEnable       0x01
#define __VoltSource        0x02
#define __motorPolePair     0x03
#define __motorDirection    0x04
#define __dataLoggerAction  0x05
#define __dataLoggerAction  0x05
#define __pwmResolution     0x06
#define __timeTest          0x07
#define __voltTest          0x08
#define __velTest           0x09
#define __OpenLoopDrive     0x0A
#define __encoderDirection  0x0B
#define __rotorOffsetCW     0x0C
#define __rotorOffsetCCW    0x0D
#define __shaftOffset       0x0E
#define __GearRatio         0x0F
#define __rotorAlignment    0x10
#define __CloseLoopDrive    0x11
#define __limitVelocity     0x12  
#define __maxVelocity       0x13
#define __minVelocity       0x14
#define __Acceleration      0x15
#define __Deceleration      0x16
#define __posPGain          0x17
#define __posIGain          0x18
#define __velPGain          0x19
#define __velIGain          0x1A

/* eeprom address */
#define addr__VoltSource        0x00
#define addr__motorPolePair     0x04
#define addr__motorDirection    0x08
#define addr__pwmResolution     0x0C
#define addr__timeTest          0x10
#define addr__voltTest          0x14
#define addr__velTest           0x18
#define addr__encoderDirection  0x1C
#define addr__rotorOffsetCW     0x20
#define addr__rotorOffsetCCW    0x24
#define addr__shaftOffset       0x28
#define addr__GearRatio         0x2C
#define addr__limitVelocity     0x30  
#define addr__maxVelocity       0x34
#define addr__minVelocity       0x38
#define addr__Acceleration      0x3C
#define addr__Deceleration      0x40
#define addr__posPGain          0x44
#define addr__posIGain          0x48
#define addr__velPGain          0x4C
#define addr__velIGain          0x50

// 50 54 58 5c

/* motor parameter variable */
float Vs;
uint8_t pp;
boolean motorDir;
uint16_t pwmRes;
float timeTest;
uint8_t voltTest;
float velTest;
boolean encoderDir;
uint16_t rotorOffsetCW;
uint16_t rotorOffsetCCW;
uint16_t shaftOffset;
float GearRatio;
/* motion control */
float limitVel;  
float maxVel; 
float minVel;
float accel;
float decel;
float pos_Kp;
float pos_Ki;
float vel_Kp;
float vel_Ki;






#endif // DATA_LOGGER_H
