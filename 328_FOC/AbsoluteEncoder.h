


#ifndef ABSOLUTEENCODER_h
#define ABSOLUTEENCODER_h

#include "Output.h"
#include "AS5X47.h"

#define INVERT 1
#define NON_INVERT 0
#define Vr_HIGH 6.0f  // alignment voltage
#define Vr_LOW  4.0f
#define ssPin 8       // slave select pin 
#define polePair 20   // pole pair of motor
#define shaftOffset 0
#define rotorOffset 0
AS5X47 en(ssPin, polePair);

void setEncoderDir(uint8_t rotate)
{
  Settings1 set;
  set.values.uvw_abi = 0; 
  set.values.dir = rotate;
  en.writeSettings1(set);
}
void rotorAlignment(boolean enable)
{
  /* shaft position offset */
  motorStop(2000);
  Serial.print("shaft position: "); Serial.print("\t");  
  Serial.println(en.setShaftOffset(shaftOffset));  
  
  /* rotor position offset */
  if(enable)
  {
    Serial.println("rotor position aligning");
    for(float deg=0.0f; deg<360; deg+=0.5f)
    { // -> cw rotate
      sv.updateVector(Vr_HIGH, deg);
      setPwm();
    }
    for(float deg=360.0f; deg>0; deg-=0.5f)
    { // -> ccw rotate
      sv.updateVector(Vr_LOW, deg);
      setPwm();
    }
    delay(500);  
  }
  Serial.print("rotor position: "); Serial.print("\t");
  Serial.println(en.setRotorOffset(rotorOffset)); 
  Serial.println("end of process");
}

#endif // #ABSOLUTEENCODER_h
