
/** 05-06-22
 *  Created by Teerachot Mueangchamnong 
 *  Brushless Motor Control for Quadruped Leg Project
 *  Mechatronics Engineering Technology, KMUTNB
 */

#ifdef Trash

/* current sensor */
#include <Filters.h>
FilterOnePole cps(LOWPASS, 0.8); 
#define currentA_pin A0
#define currentB_pin A1
int16_t ADC_thresA = 512;
int16_t ADC_thresB = 512;
#define ADC_to_mV 4.887585533f 
#define mV_to_A 0.01f
#define getCurrentA float(analogRead(currentA_pin) - ADC_thresA) * ADC_to_mV * mV_to_A
#define getCurrentB float(analogRead(currentB_pin) - ADC_thresB) * ADC_to_mV * mV_to_A
float vec_error, vec_integral = 0;
#define vec_Kp 0.08
#define vec_Ki 0.002
#define vec_I_Limit 25.0f

  /* zero adjust current sensor */
  ADC_thresA = analogRead(currentA_pin);
  ADC_thresB = analogRead(currentB_pin);


float vectorCompensate(float Ia, float Ib, float designTheta)
{
  float Ic = -(Ia + Ib);
  float Ix = Ia - Ib / 2.0f - Ic / 2.0f;             
  float Iy = root3_divided_by2 * Ib - root3_divided_by2 * Ic;
  float resultVect = sqrt(Ix * Ix + Iy * Iy);
//  float actualTheta = atan2(Iy, Ix);
//  float rawCps = fabs(sin(designTheta)) + fabs(sin(actualTheta)) - 1.0f;
//  rawCps = constrain(rawCps, -1, 1);
  
//  Serial.print(String(designTheta) + "\t"); 
//  Serial.println(designTheta-asin(rawCps));
  
//  vec_error = asin(rawCps);
//  vec_integral += vec_error;
//       if(vec_integral >  vec_I_Limit)  vec_integral =  vec_I_Limit;
//  else if(vec_integral < -vec_I_Limit)  vec_integral = -vec_I_Limit;
//  float out =  vec_error * vec_Kp + vec_integral * vec_Ki;
     
  Serial.println(resultVect);
  return 0;  
}

void select_mode()
{
  while(true)
  {
    if(Serial.find(configHeader))
      runMode = Serial.parseInt(); 
      
    if(runMode == 0)
    {
      Serial.println(F("Actuator:\t\tConfig Mode"));
      break;
    }
    else if(runMode == 1)
    {
      Serial.println(F("Actuator:\t\tRun Mode"));
      break;  
    }
  }
}

Serial.println(F("Invalid data!"));

/* software reset */
void(*resetFunc) (void) = 0; 
/* serial communication interrupt */
void serialEvent()
{    
  if(Serial.find(controlHeader))
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

int commandAddress[] = 
{
  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 
  11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 
  21, 22, 23, 24, 25, 26, 27, 28, 29, 30
};

  Serial.print(motorID);  Serial.print("\t");
  Serial.print(controlMode);  Serial.print("\t");
  Serial.print(Vs);  Serial.print("\t");
  Serial.print(currentLimit);  Serial.print("\t");
  Serial.print(tempLimit);  Serial.print("\t");
  Serial.print(pp);  Serial.print("\t");
  Serial.print(motorDir);  Serial.print("\t");
  Serial.print(rotorOffsetCW);  Serial.print("\t");
  Serial.print(rotorOffsetCCW);  Serial.print("\t");
  Serial.print(shaftOffset);  Serial.print("\t");
  Serial.print(GearRatio);  Serial.print("\t");
  Serial.print(pwmRes);  Serial.print("\t");
  Serial.print(encoderDir);  Serial.print("\t");
  Serial.print(timeTest);  Serial.print("\t");
  Serial.print(voltTest);  Serial.print("\t");
  Serial.print(velTest);  Serial.print("\t");
  Serial.print(maxVel);  Serial.print("\t"); 
  Serial.print(minVel);  Serial.print("\t");
  Serial.print(limitVel);  Serial.print("\t");  
  Serial.print(accel);  Serial.print("\t");
  Serial.print(decel);  Serial.print("\t");
  Serial.print(pos_Kp);  Serial.print("\t");
  Serial.print(pos_Ki);  Serial.print("\t");
  Serial.print(vel_Kp);  Serial.print("\t");
  Serial.println(vel_Ki);


/* torque control */
float getCur()
{
//  cps.input(int(getCurrent));
  float valOut = getCurrent; //cps.output();
  if(valOut <= 0.0f) 
  {
    valOut = 0;
  }
  Serial.println(valOut);
  return valOut;
}
void updateSp()
{
  if(getCur() >= limitCurrent)
  {
     en.update();
     pos_setpoint = en.getShaftAngle();    
  }
}

#endif
