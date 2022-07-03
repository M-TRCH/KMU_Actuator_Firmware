
/** 29-05-22
 *  Created by Teerachot Mueangchamnong 
 *  Brushless Motor Control for Quadruped Leg Project
 *  Mechatronics Engineering Technology, KMUTNB
 *  Refer https://www.circuito.io/blog/arduino-uno-pinout/
*/

#ifndef ARDUINO_UNO_SVPWM_H
#define ARDUINO_UNO_SVPWM_H
#include "SVPWM.h"  

#define enableA 5
#define enableB 6
#define enableC 7

class Arduino_UNO_SVPWM: public SVPWM
{   
  private:
    uint8_t dir;
  public:
    Arduino_UNO_SVPWM(float v, float p, boolean invert);
    ~Arduino_UNO_SVPWM();
    void outputInit();
    void motorEnable();
    void motorDisable();
    void resetPwm();
    void resetPwm(unsigned long t);
    void setPwm();
    void setNewDir(boolean d);
    void driveClarke(float Vx, float Vy, float theta);
    void drivePark(float Vd, float Vq, float theta);
    void driveVector(float Vr, float theta);
    
};

#endif // #ARDUINO_UNO_SVPWM_H
