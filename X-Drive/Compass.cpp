
#include "Compass.hpp"


Compass::Compass(PinName sda, PinName scl) {
	_i2c = new I2C(sda, scl);
	//_i2c.frequency(400000);
	_ready = false;

	_compass_addr8bit = 0x0c << 1;
	_leds_addr8bit = 0x55 << 1;
}


ssize_t Compass::init() {
	char txBuf[2];
	char rxBuf[2] = {0, 0};


	txBuf[0] = 0x00;
	_i2c->write(_compass_addr8bit, txBuf, 1);
	_i2c->read(_compass_addr8bit, rxBuf, 1);

	if (rxBuf[0] != 0x48) {
		return -1;
	}

	txBuf[0] = 0x01;
	_i2c->write(_compass_addr8bit, txBuf, 1);
	_i2c->read(_compass_addr8bit, rxBuf, 1);

	if (rxBuf[0] != 0x09) {
		return -1;
	}


	// Set the mode:
	// 0x00 - sleep
	// 0x02, Mode 1: 10Hz continuous
	// 0x04, Mode 2: 20Hz continuous
	// 0x06, Mode 3: 50Hz continuous
	// 0x08, Mode 4: 100Hz continuous

	txBuf[0] = 0x31;
	txBuf[1] = 0x04;
	_i2c->write(_compass_addr8bit, txBuf, 2);

	_ready = true;
	return 0;
}

ssize_t Compass::get_data(int16_t *xyz) {

	char txBuf[2] = {0x11, 0x0};

	if (!_ready) {
		return 0;
	}

	_i2c->write(_compass_addr8bit, txBuf, 1);
	_i2c->read(_compass_addr8bit, (char*) xyz, 6);

	return 6;
}


int Compass::set_leds(uint8_t red, uint8_t green, uint8_t blue) {

	// Inputs range from 0 to 15 (4 bits)

	char txBuf[5] = {0x01, 0,0,0, 0x83};

	//txBuf[0] = 0x01;  // pwm0 register
	txBuf[1] = blue & 0x0f;
	txBuf[2] = green & 0x0f;
	txBuf[3] = red & 0x0f;
	//txBuf[4] = 0x83;  // enable, and stop auto-incrementing sequential writes
	_i2c->write(_leds_addr8bit, txBuf, 5);

	return 0;
}



