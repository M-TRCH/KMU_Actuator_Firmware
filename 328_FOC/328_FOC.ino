
#include "Output.h"
#include "AbsoluteEncoder.h"

float thetaIn, thetaOut;

void setup() 
{
  Serial.begin(115200);

  outputInit();    
//  setEncoderDir(INVERT); // uncomment for first time
  rotorAlignment(true);

}

void loop() 
{
  /* get rotor position */
  en.shaftAngleUpdate();
  float shaft = en.getShaftAngle();
  thetaIn = en.getRotorAngle(); 
  
  
  /* position control */
  const float Kp = 0.2f;
  float sp = -450;
  float error = sp - shaft;
  error = constrain(error, -360, 360);  
  float Vr = Kp * abs(error);
  Vr = constrain(Vr, 0, Vs);  
//  float thetaOut = thetaIn + (error / abs(error)) * 90.0f;
  float thetaOut = thetaIn + (error / abs(error)) * map(Vr, 0, 12, 900, 1200) / 10.0f;

  /* commutation control */
//  float Vr = 12.0f;
//  thetaOut = thetaIn + 150.0f;  

  /* open loop drive */
//  float Vr = 6.0; 
//  thetaOut += 5.0f;       
//       if(thetaOut > 360) thetaOut = thetaOut - 360.0f;
//  else if(thetaOut < 0  ) thetaOut = 360.0f + thetaOut;  

  /* output */ 
  sv.updateVector(Vr, thetaOut);
  setPwm();

  /* debug */
//  Serial.print(en.getShaftAngle()); Serial.print("\t");
//  Serial.println(en.shaftAngleDif);
//  Serial.println(thetaIn);
//  delay(10);

}
