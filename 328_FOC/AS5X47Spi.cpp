
/** 21-02-22
 *  Modified by Teerachot Mueangchamnong 
 *  Brushless Motor Control for Quadruped Leg Project
 *  Mechatronics Engineering Technology, KMUTNB
 *  Copyright by MIT License
 *  Refer https://github.com/Adrien-Legrand/AS5X47
*/

#include "AS5X47Spi.h"

AS5X47Spi::AS5X47Spi(uint8_t _chipSelectPin) 
{
	// Initialize SPI Communication
	chipSelectPin = _chipSelectPin;
  pinMode(chipSelectPin, OUTPUT);
	digitalWrite(chipSelectPin, HIGH);
	SPI.begin();
}
void AS5X47Spi::writeData(uint16_t command, uint16_t value) 
{
	// @todo Expose the SPI Maximum Frequency in library interface.
	SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE1));
	// Send command
	digitalWrite(chipSelectPin, LOW);
	SPI.transfer16(command);
	digitalWrite(chipSelectPin, HIGH);
//	delayMicroseconds(1);
	// Read data
	digitalWrite(chipSelectPin, LOW);
	SPI.transfer16(value);
	digitalWrite(chipSelectPin, HIGH);
	SPI.endTransaction();
//	delayMicroseconds(1);
}
uint16_t AS5X47Spi::readData(uint16_t command, uint16_t nopCommand) 
{
	SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE1));

	// Send Read Command
	digitalWrite(chipSelectPin, LOW);
	SPI.transfer16(command);
	digitalWrite(chipSelectPin, HIGH);
//	delayMicroseconds(1);
	// Send Nop Command while receiving data
	digitalWrite(chipSelectPin, LOW);
	uint16_t receivedData = SPI.transfer16(nopCommand);
	digitalWrite(chipSelectPin, HIGH);
	SPI.endTransaction();
//	delayMicroseconds(1);
	return receivedData;
}
