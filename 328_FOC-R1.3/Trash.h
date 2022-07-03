
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


#endif
