
/** 30-05-22
 *  Created by Teerachot Mueangchamnong 
 *  Brushless Motor Control for Quadruped Leg Project
 *  Mechatronics Engineering Technology, KMUTNB
 *  Refer https://github.com/Adrien-Legrand/AS5X47
 */

#include "AbsoluteEncoder.h"

AbsoluteEncoder::AbsoluteEncoder(uint8_t ssPin, uint8_t pp, boolean dir, int16_t rOffsetCW, int16_t rOffsetCCW, int16_t sOffset) : AS5X47(ssPin)
{
  prevRawAngle = readRaw();
  prevTime = millis();
  setPolePair(pp);
  setDirCount(dir); 
  setRotorOffsetCW(rOffsetCW);
  setRotorOffsetCCW(rOffsetCCW);
  setShaftOffset(sOffset);
}
AbsoluteEncoder::~AbsoluteEncoder()
{
  
}    
void AbsoluteEncoder::setPolePair(uint8_t pp)
{
  pole_pair = pp;
}
void AbsoluteEncoder::setDirCount(boolean dir)
{
  Settings1 setting;
  setting.values.uvw_abi = 0; 
  setting.values.dir = dir;
  writeSettings1(setting);    
}
int AbsoluteEncoder::setRotorOffsetCW(int16_t rOffset)
{
  if(rOffset == 0)  rOffset = readRaw();
  rotorOffset[CW] = rOffset;
  return rotorOffset[CW];
}
int AbsoluteEncoder::setRotorOffsetCCW(int16_t rOffset)
{
  if(rOffset == 0)  rOffset = readRaw();
  rotorOffset[CCW] = rOffset;
  return rotorOffset[CCW];
}
int AbsoluteEncoder::setShaftOffset(int16_t sOffset)
{
  if(sOffset == 0)  sOffset = readRaw();
  shaftOffset = sOffset;
  return shaftOffset;
}
void AbsoluteEncoder::update()
{
  currRawAngle = readRaw();
  diffRawAngle = currRawAngle - prevRawAngle;
  prevRawAngle = currRawAngle;
  
  if(diffRawAngle > rawAngleThreshold)
    revCounter--;
  else if(diffRawAngle < -rawAngleThreshold)
    revCounter++;      
}
//float AbsoluteEncoder::getRotorAngle()
//{
//  return pole_pair * rawToRad * (((float)currRawAngle + 16384.0f * (float)revCounter) - (float)rotorOffset);
//}
float AbsoluteEncoder::getRotorAngle(uint8_t index)
{
  float returnVal = (float)currRawAngle - (float)rotorOffset[index];
  returnVal<0? returnVal+=16383.0f: returnVal;
  returnVal *= pole_pair * rawToRad;
  return returnVal;
}
float AbsoluteEncoder::getShaftAngle()
{
  return rawToRad * (((float)currRawAngle + 16384.0f * (float)revCounter) - (float)shaftOffset);
}
float AbsoluteEncoder::getShaftVel()
{
  currTime = millis();
  diffTime = currTime - prevTime;
  prevTime = currTime;
  currShaftAngle = getShaftAngle();
  diffShaftAngle = currShaftAngle - prevShaftAngle;
  prevShaftAngle = currShaftAngle;
  return diffShaftAngle / diffTime * 1000.f;
}
int16_t AbsoluteEncoder::getRevCounter()
{
  return revCounter; 
}
void AbsoluteEncoder::zeroShaftAngle()
{
  revCounter = 0;
}
float AbsoluteEncoder::print()
{
  return 0;
}
