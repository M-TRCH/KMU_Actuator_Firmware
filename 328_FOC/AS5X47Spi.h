
/** 21-02-22
 *  Modified by Teerachot Mueangchamnong 
 *  Brushless Motor Control for Quadruped Leg Project
 *  Mechatronics Engineering Technology, KMUTNB
 *  Copyright by MIT License
 *  Refer https://github.com/Adrien-Legrand/AS5X47
*/

#ifndef AS5X47COMMUNICATION_h
#define AS5X47COMMUNICATION_h

#include "Arduino.h"
#include <SPI.h>

class AS5X47Spi 
{
  public:
    AS5X47Spi(uint8_t chipSelectPin);
    void writeData(uint16_t command, uint16_t value);
    uint16_t readData(uint16_t command, uint16_t nopCommand);

  private:
    uint8_t chipSelectPin;
};

#endif // #AS5X47COMMUNICATION_h
