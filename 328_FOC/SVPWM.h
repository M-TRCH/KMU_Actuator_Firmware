
/** 19-03-22
 *  Created by Teerachot Mueangchamnong 
 *  Brushless Motor Control for Quadruped Leg Project
 *  Mechatronics Engineering Technology, KMUTNB
 *  Refer https://www.switchcraft.org/learning/2017/3/15/space-vector-pwm-intro
*/

#ifndef SVPWM_h
#define SVPWM_h
#include "Arduino.h"

#define PI                3.14159265359f
#define degToRad          0.01745329251f
#define radToDeg          57.2957795131f
#define PIdivided_by3     1.04719755120f
#define _2divided_by3     0.66666666666f 
#define _1divided_by3     0.33333333333f
#define root3_divided_by2 0.86602540378f 
#define root3_divided_by3 0.57735026919f 
#define root3             1.73205080757f  
#define firstRatio        1.154f
#define thirdRatio        0.154f

class SVPWM 
{
  private:
    float Vs; // source voltage 
    float Va; // a, b, c phase voltage
    float Vb;   
    float Vc;  
    float Vx; // x, y reference axis voltage
    float Vy;  
    float Vt; // third hamornic voltage
    float thirdTheta; // angle of third hamornic wave
    float pwmRes;     // pwm resolution
    float m;          // slope of linear equation
    float c;          // y axis intercept point
    float pwmA;       // pwm output for a, b, c phase
    float pwmB;
    float pwmC;
  
  public: 
    SVPWM(float vect, float pwm);  ~SVPWM();
    float getPwmA();
    float getPwmB();
    float getPwmC();
    float getVx();
    float getVy();
    float getVa();
    float getVb();
    float getVc();
    void debugPrint();
    void updateClarke(float Vx, float Vy, float theta);
    void updatePark(float Vd, float Vq, float theta);
    void updateVector(float Vr, float theta);
    
};

#endif // #SVPWM_h
