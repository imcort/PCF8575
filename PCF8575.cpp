/*
 * See header file for details
 *
 *  This program is free software: you can redistribute it and/or modify\n
 *  it under the terms of the GNU General Public License as published by\n
 *  the Free Software Foundation, either version 3 of the License, or\n
 *  (at your option) any later version.\n
 * 
 *  This program is distributed in the hope that it will be useful,\n
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of\n
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n
 *  GNU General Public License for more details.\n
 * 
 *  You should have received a copy of the GNU General Public License\n
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.\n
 */

/* Dependencies */
#include <Wire.h>
#include "PCF8575.h"

PCF8575::PCF8575() :
		_PORT(0), _PIN(0), _DDR(0), _address(0)
{
}

void PCF8575::begin(uint8_t address, const uint32_t speed) {

	/* Store the I2C address and init the Wire library */
	_address = address;
	Wire.begin();
	Wire.setClock(speed);
	readGPIO();
}

void PCF8575::pinMode(uint8_t pin, uint8_t mode) {

	/* Switch according mode */
	switch (mode) {
	case INPUT:
		_DDR &= ~(1 << pin);  //Set 0
		_PORT |= (1 << pin);  //Set 1
		break;

	case OUTPUT:
		_DDR |= (1 << pin);   //Set 1
		_PORT &= ~(1 << pin); //Set 0
		break;

	default:
		break;
	}

	/* Update GPIO values */
	updateGPIO();
}

void PCF8575::digitalWrite(uint8_t pin, uint8_t value) {

	/* Set PORT bit value */
	if (value)
		_PORT |= (1 << pin);
	else
		_PORT &= ~(1 << pin);

	/* Update GPIO values */
	updateGPIO();
}

uint8_t PCF8575::digitalRead(uint8_t pin) {

	/* Read GPIO */
	readGPIO();

	/* Read and return the pin state */
	return (_PIN & (1 << pin)) ? HIGH : LOW;
}

void PCF8575::write(uint16_t value) {

	/* Store pins values and apply */
	_PORT = value;

	/* Update GPIO values */
	updateGPIO();
}

uint16_t PCF8575::read() {

	/* Read GPIO */
	readGPIO();

	/* Return current pins values */
	return _PIN;
}

void PCF8575::clear() {

	/* User friendly wrapper for write() */
	write(0x0000);
}

void PCF8575::set() {

	/* User friendly wrapper for write() */
	write(0xFFFF);
}

void PCF8575::toggle(uint8_t pin) {

	/* Toggle pin state */
	_PORT ^= (1 << pin);

	/* Update GPIO values */
	updateGPIO();
}

void PCF8575::shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val) {
  uint8_t i;
  for (i = 0; i < 8; i++) {
    if (bitOrder == LSBFIRST)
      PCF8575::digitalWrite(dataPin, !!(val & (1 << i)));
    else
      PCF8575::digitalWrite(dataPin, !!(val & (1 << (7 - i))));

    PCF8575::digitalWrite(clockPin, HIGH);
    PCF8575::digitalWrite(clockPin, LOW);
  }

}

uint8_t PCF8575::shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) {
    uint8_t value = 0;
    uint8_t i;
	PCF8575::digitalWrite(dataPin, HIGH);
    for(i = 0; i < 8; ++i) {
        PCF8575::digitalWrite(clockPin, HIGH);
        if(bitOrder == LSBFIRST)
            value |= PCF8575::digitalRead(dataPin) << i;
        else
            value |= PCF8575::digitalRead(dataPin) << (7 - i);
        PCF8575::digitalWrite(clockPin, LOW);
    }
    return value;
}

void PCF8575::readGPIO() {

	/* Start request, wait for data and receive GPIO values as byte */
	Wire.requestFrom(_address, (uint8_t) 0x02);
	while (Wire.available() < 2)
		;
	_PIN = I2CREAD(); /* LSB first */
	_PIN |= I2CREAD() << 8;
}

void PCF8575::updateGPIO() {

	/* Compute new GPIO states */
	//uint8_t value = ((_PIN & ~_DDR) & ~(~_DDR & _PORT)) | _PORT; // Experimental
	uint16_t value = (_PIN & ~_DDR) | _PORT;

	/* Start communication and send GPIO values as byte */
	Wire.beginTransmission(_address);
	I2CWRITE(value & 0x00FF);
	I2CWRITE((value & 0xFF00) >> 8);
	Wire.endTransmission();
}
