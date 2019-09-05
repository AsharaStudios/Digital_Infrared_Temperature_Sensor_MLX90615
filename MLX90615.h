#ifndef __MLX90615_H__
#define __MLX90615_H__

#include <Arduino.h>
#include <I2cMaster.h>
#include <Wire.h>
#include <stdbool.h>

#define MLX90615_OBJECT_TEMPERATURE     0x27
#define MLX90615_AMBIENT_TEMPERATURE    0x26
#define AccessEEPROM                    0x13
#define Default_Emissivity              0x4000
#define DEVICE_ADDR                     0x5B



class MLX90615 {

protected:
	TwoWire *wbus;
	I2cMasterBase *bus;
	byte i2c_addr;
	uint8_t buffer[6];

	int writeI2C(uint8_t *buff, uint8_t len, bool stop = true) {
		int sent = 0;
		if (bus && !wbus) {
			byte dev = i2c_addr << 1;
			bus->start(dev | I2C_WRITE);
			for (uint8_t i = 0 ; i < len ; i++) {
				if (bus->write(buffer[i]))
					sent++;
				else
					break;
			}
			if (stop)
				bus->stop();
			else
				bus->restart(dev | I2C_READ);
			return sent;
		} else if (wbus && !bus) {
			wbus->beginTransmission(i2c_addr);
			wbus->write(buffer,len);
			wbus->endTransmission(stop);
			return sent;
		} else return -1;
	}

	int readI2C(uint8_t *buff, uint8_t len){
		int read = 0;
		if (bus && !wbus) {
			for (uint8_t i = 0 ; i < len ; i++ ) {
				buff[i] = bus->read(i == len-1);
			}
			bus->stop();
			return read;
		} else if (wbus && !bus) {
			read = wbus->requestFrom(i2c_addr,len);
			for (uint8_t i = 0 ; i < read ; i++ ) {
				buff[i] = wbus->read();
			}
			return read;
		} else return -1;
	}

public:

	/*******************************************************************
	 * Function Name: init
	 * Description:  initialize for i2c device.
	 * Parameters: sda pin, scl pin, i2c device address
	 * Return: null
	******************************************************************/
	MLX90615(byte addr, I2cMasterBase *i2c) {
		i2c_addr = addr;
		bus = i2c;
		wbus = 0;
	}

	MLX90615(byte addr, TwoWire *i2c) {
		i2c_addr = addr;
		bus = 0;
		wbus = i2c;
	}

	/****************************************************************
	 * Function Name: crc8_msb
	 * Description:  CRC8 check to compare PEC data
	 * Parameters: poly - x8+x2+x1+1, data - array to check, array size
	 * Return: 0 – data right; 1 – data Error
	****************************************************************/
	uint8_t crc8Msb(uint8_t poly, uint8_t* data, int size)	{
		uint8_t crc = 0x00;
		int bit;

		while (size--)
		{
			crc ^= *data++;
			for (bit = 0; bit < 8; bit++)
			{
				if (crc & 0x80) {
					crc = (crc << 1) ^ poly;
				} else {
					crc <<= 1;
				}
			}
		}

		return crc;
	}

	/****************************************************************
	 * Function Name: Print_Temperature
	 * Description: receive and print out temperature in Celcius and fahrenheit
	 * Parameters: Temperature_kind - choose ambient or object Temperature
	 * Return:  true for ok, false for failure
	****************************************************************/
	float getTemperature(int Temperature_kind, bool fahrenheit = false) {
		float celcius;
		const double tempFactor = 0.02; // 0.02 degrees per LSB (measurement resolution of the MLX90614)
		// This masks off the error bit of the high byte, then moves it left 8 bits and adds the low byte.
		double tempData = readRegS16(Temperature_kind) & 0x7FFF;
		tempData = (tempData * tempFactor) - 0.01;

		celcius = (float)(tempData - 273.15);

		return fahrenheit ? (celcius*1.8) + 32.0 : celcius;
	}

	float getTemperatureFahrenheit(float celcius){
		return (celcius*1.8) + 32;
	}
	/****************************************************************
	 * Function Name: readEEPROM
	 * Description:  read from eeprom to see what data in it.
	 * Parameters:
	 * Return: -1 for failure, 0 for ok
	****************************************************************/
	int readEEPROM() {
		delay(500);
		buffer[0] = AccessEEPROM;
		if (writeI2C(buffer,1,false) < 1)
			return -1;
		// read
		if (readI2C(buffer,3) < 3)
			return -1;

		uint16_t eepromData = ((((uint16_t)buffer[1] & 0x007F) << 8) + buffer[0]);

		Serial.print("eepromData: 0x");
		Serial.println(eepromData, HEX);

		return 0;
	}

	int write(uint8_t EEPROM_addr,uint16_t value)
	{
		buffer[0] = i2c_addr<<1;
		buffer[1] = EEPROM_addr;
		buffer[2] = value & 0xff;
		buffer[3] = (value >> 8) & 0xff;
		buffer[4] = crc8Msb(0x07, buffer, 4);
	//    Serial.println(*data, HEX);

	   if(crc8Msb(0x07,buffer, 5))
	   {
			 Serial.println("CRC8 Check Err...");
			 return -1;
	   }
	   Serial.println("CRC8 Check OK...");

		return writeI2C(&buffer[1],4);
	}

	/****************************************************************
	 * Function Name: writeEmissivity
	 * Description:  write EEPROM at address 0x13 to adjust emissivity
	 * Parameters: emissivity – data to write into EEPROM
	 * Return: 0 - success; -1 - crc8 check err.
	****************************************************************/
	int writeEmissivity(uint16_t emissivity){
		return write(AccessEEPROM,emissivity);
	}

	/****************************************************************
	 * Function Name: read8
	 * Description:  i2c read register for one byte
	 * Parameters:
	 * Return: sucess - true/false
	****************************************************************/
	bool read8(byte reg, uint8_t &result)
	{
		buffer[0] = reg;
		if (writeI2C(buffer,1,false) < 1)
			return false;
		if (readI2C(buffer,2)< 2)
			return false;
		result = buffer[0];

		return true;
	}

	/****************************************************************
	 * Function Name: read8
	 * Description:  i2c read register for one byte
	 * Parameters:
	 * Return: one byte data from i2c device
	****************************************************************/
	byte read8(byte reg)
	{
		byte result;
		read8(reg,result);
		return result;
	}

	/****************************************************************
	 * Function Name: readRegS16
	 * Description:  i2c read register for int data
	 * Parameters:
	 * Return: sucess - true/false
	****************************************************************/
	bool readRegS16(byte reg, uint16_t &result)
	{
		buffer[0] = reg;
		if (writeI2C(buffer,1,false) < 1)
			return false;
		if (readI2C(buffer,3)< 3)
			return false;
		result = ((uint16_t)buffer[1] << 8) | buffer[0];
		return true;
	}

	/****************************************************************
	 * Function Name: readRegS16
	 * Description:  i2c read register for int data
	 * Parameters:
	 * Return: int data
	****************************************************************/
	int readRegS16(byte reg)
	{
		uint16_t result;
		readRegS16(reg,result);
		return (int)result;
	}

	/****************************************************************
	 * Function Name: writeReg8
	 * Description:  i2c write register one byte
	 * Parameters:
	 * Return: success: true/false
	****************************************************************/
	bool writeReg8(byte data, byte reg)
	{
		buffer[0] = reg;
		buffer[1] = data;
		return writeI2C(buffer,2);
	}

	/****************************************************************
	 * Function Name: writeReg16
	 * Description:  i2c write register int data
	 * Parameters:
	 * Return: bytes sent
	****************************************************************/
	int writeReg16(uint16_t data, byte reg)
	{
		buffer[0] = reg;
		buffer[1] = (data >> 8) & 0xFF;
		buffer[2] = data & 0xFF;
		return writeI2C(buffer,3);
	}
};
#endif // __MLX90615_H__
