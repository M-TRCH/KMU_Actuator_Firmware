
/** 29-05-22
 *  Modified by Teerachot Mueangchamnong 
 *  Brushless Motor Control for Quadruped Leg Project
 *  Mechatronics Engineering Technology, KMUTNB
 *  Copyright by MIT License
 *  Refer https://github.com/Adrien-Legrand/AS5X47
 */

#ifndef AS5X47_H
#define AS5X47_H
#include "Arduino.h"
#include <SPI.h>

// Volatile Registers Addresses
#define NOP_REG			  0x0000
#define ERRFL_REG 		0x0001
#define PROG_REG		  0x0003
#define DIAGAGC_REG 	0x3FFC
#define MAG_REG 		  0x3FFD
#define ANGLE_REG 		0x3FFE
#define ANGLECOM_REG  0x3FFF

// Non-Volatile Registers Addresses
#define ZPOSM_REG 		0x0016
#define ZPOSL_REG 		0x0017
#define SETTINGS1_REG 0x0018
#define SETTINGS2_REG 0x0019

#define WRITE 0
#define READ  1

// ERRFL Register Definition
typedef union 
{
    uint16_t raw;
    struct __attribute__ ((packed)) 
    {
      uint16_t frerr:1;
      uint16_t invcomm:1;
      uint16_t parerr:1;
      uint16_t unused:13;
    } values;
} Errfl;
// PROG Register Definition
typedef union 
{
    uint16_t raw;
    struct __attribute__ ((packed)) 
    {
    	uint16_t progen:1;
    	uint16_t unused:1;
    	uint16_t otpref:1;
    	uint16_t progotp:1;
      uint16_t unused1:2;
      uint16_t progver:1;
      uint16_t unused2:9;
    } values;
} Prog;
// DIAAGC Register Definition
typedef union 
{
    uint16_t raw;
    struct __attribute__ ((packed)) 
    {
      uint16_t agc:8;
      uint16_t lf:1;
      uint16_t cof:1;
      uint16_t magh:1;
      uint16_t magl:1;
      uint16_t unused:4;
    } values;
} Diaagc;
// MAG Register Definition
typedef union 
{
  uint16_t raw;
  struct __attribute__ ((packed)) 
  {
    uint16_t cmag:14;
    uint16_t unused:2;
  } values;
} Mag;
// ANGLE Register Definition
typedef union 
{
  uint16_t raw;
  struct __attribute__ ((packed)) 
  {
    uint16_t cordicang:14;
    uint16_t unused:2;
  } values;
} Angle;
// ANGLECOM Register Definition
typedef union 
{
  uint16_t raw;
  struct __attribute__ ((packed)) 
  {
    uint16_t daecang:14;
    uint16_t unused:2;
  } values;
} Anglecom;
// ZPOSM Register Definition
typedef union 
{
  uint8_t raw;
  struct __attribute__ ((packed)) 
  {
    uint8_t zposm;
  } values;
} Zposm;
// ZPOSL Register Definition
typedef union 
{
  uint8_t raw;
  struct __attribute__ ((packed)) 
  {
    uint8_t zposl:6;
    uint8_t compLerrorEn:1;
    uint8_t compHerrorEn:1;
   } values;
} Zposl;
// SETTINGS1 Register Definition
typedef union 
{
  uint8_t raw;
  struct __attribute__ ((packed)) 
  {
    uint8_t factorySetting:1;
    uint8_t noiseset:1;
    uint8_t dir:1;
    uint8_t uvw_abi:1;
    uint8_t daecdis:1;
    uint8_t abibin:1;
    uint8_t dataselect:1;
    uint8_t pwmon:1;
  } values;
} Settings1;
// SETTINGS2 Register Definition
typedef union 
{
  uint8_t raw;
  struct __attribute__ ((packed)) 
  {
    uint8_t uvwpp:3;
  	uint8_t hys:2;
  	uint8_t abires:3;
  } values;
} Settings2;
// Command Frame  Definition
typedef union 
{
  uint16_t raw;
  struct __attribute__ ((packed)) 
  {
    uint16_t commandFrame:14;
    uint16_t rw:1;
    uint16_t parc:1;
  } values;
} CommandFrame;
// ReadData Frame  Definition
typedef union 
{
  uint16_t raw;
  struct __attribute__ ((packed)) 
  {
    uint16_t data:14;
    uint16_t ef:1;
    uint16_t pard:1;
  } values;
} ReadDataFrame;
// WriteData Frame  Definition
typedef union 
{
  uint16_t raw;
  struct __attribute__ ((packed)) 
  {
    uint16_t data:14;
    uint16_t low:1;
    uint16_t pard:1;
  } values;
} WriteDataFrame;

class AS5X47 
{
  private:
    uint8_t chipSelectPin;
    bool isEven(uint16_t data);
  
  public:
    AS5X47(uint8_t ssPin);
    void writeData(uint16_t command, uint16_t value);
    uint16_t readData(uint16_t command, uint16_t nopCommand);
    ReadDataFrame readRegister(uint16_t registerAddress);
    void writeRegister(uint16_t registerAddress, uint16_t registerValue); 
    void writeSettings1(Settings1 values);
    void writeSettings2(Settings2 values);
    void writeZeroPosition(Zposm zposm, Zposl zposl);
    void printDebugString();
    float readRaw();
    
};

#endif // #AS5X47_H
