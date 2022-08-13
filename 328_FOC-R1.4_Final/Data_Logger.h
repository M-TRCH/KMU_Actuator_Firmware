
/** 12-07-22
 *  Created by Teerachot Mueangchamnong 
 *  Brushless Motor Control for Quadruped Leg Project
 *  Mechatronics Engineering Technology, KMUTNB
 */

#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H
#include "Arduino.h"
#include <EEPROM.h>

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

#define configHeader '/'
#define controlHeader '*'
#define baudrate 9600
#define timeout 5
#define configLen 26
#define actionLen 7
volatile int addrBuf = 0;
volatile float dataBuf = 1;
volatile int idBuf = 255;
volatile float cmdBuf = 0;
volatile boolean configMode = true;
float command = 0;

int eepromAddress[] = 
{
  0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40,
  44, 48, 52, 56, 60, 64, 68, 72, 76, 80,
  84, 88, 92, 96, 100, 104, 108, 112, 116, 120
};

float blank;
/* motor parameter */
float motorID;
float controlMode;
float Vs;
float currentLimit;
float tempLimit;
float pp;
float motorDir;
float rotorOffsetCW;
float rotorOffsetCCW;
float shaftOffset;
float GearRatio;
float pwmRes;
float encoderDir;
float timeTest;
float voltTest;
float velTest;
/* motion control */
float maxVel; 
float minVel;
float limitVel;  
float accel;
float decel;
float pos_Kp;
float pos_Ki;
float vel_Kp;
float vel_Ki;

float *bufVal[configLen] = 
{
  &blank,
  &motorID,
  &controlMode,
  &Vs,
  &currentLimit,
  &tempLimit,
  &pp,
  &motorDir,
  &rotorOffsetCW,
  &rotorOffsetCCW,
  &shaftOffset,
  &GearRatio,
  &pwmRes,
  &encoderDir,
  &timeTest,
  &voltTest,
  &velTest,
  &maxVel, 
  &minVel,
  &limitVel,  
  &accel,
  &decel,
  &pos_Kp,
  &pos_Ki,
  &vel_Kp,
  &vel_Ki  
};

const String configPrint[configLen] =
{ 
/*                " <- limit length */
  "Config Mode",
  "Motor ID",
  "Control Mode",
  "Voltage Source",
  "Current Limit",
  "Temp Limit",
  "Motor Pole Pair",
  "Motor Direction",
  "Rotor CW",
  "Rotor CCW",
  "Shaft Offset",
  "Gear Ratio",
  "PWM Resolution",
  "Encoder Dir",
  "Time Debug",
  "Voltage Debug",
  "Res Debug",
  "Max Velocity",
  "Min Velocity",
  "Vel Limit",
  "Accel Limit",
  "Decel Limit",
  "Pos Kp\t",
  "Pos Ki\t",
  "Vel Kp\t",
  "Vel Ki\t"

//  "Position P Gain",
//  "Position I Gain",
//  "Velocity P Gain",
//  "Velocity I Gain"
};

/* decimal places */
const uint8_t dp[configLen] = 
{
  0,
  0,
  0,
  2,
  2,
  1,
  0,
  0,
  0,
  0,
  0,
  3,
  0,
  0,
  1,
  0,
  4,
  4,
  4,
  4,
  4,
  4,
  7,
  7,
  7,
  7
};

/* constrain value */
const float constn[configLen][2] = 
{ 
  {0,         1},
  {1,         254},
  {0,         2},
  {5,         26},
  {0.1,       8.5},
  {30,        80},
  {1,         24},
  {0,         1},
  {0,         16383},
  {0,         16383},
  {0,         16383},
  {0.001,     1000},
  {16,        1024},
  {0,         1},
  {0.5,       15},
  {10,        100},
  {0.00174,   2.617},
  {0.0174,    1884.95},
  {0.0174,    1884.95},
  {0.0174,    1884.95},
  {0.0174,    1884.95},
  {0.0174,    1884.95},
  {0.0, 1000},
  {0.0, 1000},
  {0.0, 1000},
  {0.0, 1000}  
};


#endif // DATA_LOGGER_H
