
/** 30-05-22
 *  Created by Teerachot Mueangchamnong 
 *  Brushless Motor Control for Quadruped Leg Project
 *  Mechatronics Engineering Technology, KMUTNB
 *  Refer https://github.com/Adrien-Legrand/AS5X47
 */

#ifndef ABSOLUTEENCODER_H
#define ABSOLUTEENCODER_H
#include "AS5X47.h"

/* constant value */
#define rawToRad  0.000383495197f // 14-bit raw data to radian unit
#define In90Deg   1.570796327f  // 90 degree in radian unit
#define In120Deg  2.094395102f  // 120  ""
#define In150Deg  2.617993879f  // 150  ""
#define CW 0
#define CCW 1 

class AbsoluteEncoder : public AS5X47
{
  private: 
    uint8_t pole_pair;
    int16_t rotorOffset[2], shaftOffset; // 0=CW, 1=CCW
    signed long prevRawAngle, currRawAngle, diffRawAngle;
    const uint8_t rawAngleThreshold = 255;
    int16_t revCounter = 0;
    unsigned long currTime, prevTime, diffTime;
    float prevShaftAngle, currShaftAngle, diffShaftAngle;
     
  public: 
    AbsoluteEncoder(uint8_t ssPin, uint8_t pp, boolean dir, int16_t rOffsetCW, int16_t rOffsetCCW, int16_t sOffset);
    ~AbsoluteEncoder();
    void setSSPin(uint8_t ssPin);
    void setPolePair(uint8_t pp);
    void setDirCount(boolean dir);    
    int setRotorOffsetCW(int16_t rOffset);
    int setRotorOffsetCCW(int16_t rOffset);
    int setShaftOffset(int16_t sOffset);
    void update();
    float getRotorAngle(uint8_t index);
    float getShaftAngle();
    float getShaftVel();
    int16_t getRevCounter();
    void zeroShaftAngle();
    float print();
    
};

#endif // #ABSOLUTEENCODER_H
