#include "AK8963.h"

/*=============================================================================*/
/*               Define functions to access magnetometer data                  */
/*=============================================================================*/

GeegrowAK8963::GeegrowAK8963() {
	setScaleFactor();
}

void GeegrowAK8963::setScaleFactor() {
	switch (sensitivity) {
		// Possible magnetometer scales (and their register bit settings) are:
		// 14 bit resolution (0) and 16 bit resolution (1)
		case MFS_14BITS:
			scaleFactor = 0.6f; // Proper scale to return uTesla
			break;
		case MFS_16BITS:
			scaleFactor = 0.15f; // Proper scale to return uTesla
			break;
	}
}

void GeegrowAK8963::read(int16_t * data) {
	uint8_t tmpData[8];
	readBytes(AK8963_ST1, 8, tmpData);
	if (deviceId != AK8963_WHOAMI) {
		debugPrint(F("Wrong deviceId. Emerg. exit"));
		return;
	}
	
	if (!(tmpData[0] & AK8963_DATA_READY)) {
		debugPrintln(F("AK8963: data is not ready"));
		return;
	}

	if (tmpData[7] & AK8963_OVERFLOW) {
		debugPrintln(F("AK8963: Overflow"));
		return;
	}

	data[0] = ((uint16_t)tmpData[2] << 8) | tmpData[1];
	data[1] = ((uint16_t)tmpData[4] << 8) | tmpData[3];
	data[2] = ((uint16_t)tmpData[6] << 8) | tmpData[5];
}

void GeegrowAK8963::update() {
	
	read(rawData);

	mx = (float) rawData[0] * (float) scaleFactor - bias[0];
	my = (float) rawData[1] * (float) scaleFactor - bias[1];
	mz = (float) rawData[2] * (float) scaleFactor - bias[2];
}

void GeegrowAK8963::init() {
	// First extract the factory calibration for each magnetometer axis
	uint8_t tmpData[3];  // x/y/z gyro calibration data stored here

	deviceId = readByte(AK8963_REG_WHOAMI);

	if (deviceId == AK8963_WHOAMI) {
		debugPrintln(F("AK8963: I AM 72"));
	} else {
		debugPrint(F("AK8963: I AM should be 72, but got deviceId "));
		debugPrintln(deviceId, DEC);
		debugPrintln(F("Chech that you initiolize MPU before AK8963. \nEmerg. exit"));
	}

	// Set power down bit
	setPowerDownBit();
	delay(10);
	// Set Fuse ROM access mode
	setFuseRomAccessBit();
	delay(10);

	// Read the x-, y-, and z-axis calibration values
	readBytes(AK8963_ASAX, 3, &tmpData[0]);

	// Return x-axis sensitivity adjustment values, etc.
	factoryCalibration[0] =  (float)(tmpData[0] - 128)/256. + 1.;
	factoryCalibration[1] =  (float)(tmpData[1] - 128)/256. + 1.;
	factoryCalibration[2] =  (float)(tmpData[2] - 128)/256. + 1.;
	
	// Set power down bit again
	setPowerDownBit();
	delay(10);

	// Configure magnetometer mode and sensitivity.
	writeByte(AK8963_CNTL1, sensitivity << 4 | measurementMode);
	delay(10);

	// Make sure scale factor has been calculated
	setScaleFactor();
}

// Power down magnetometer
void GeegrowAK8963::setPowerDownBit() {
	writeByte(AK8963_CNTL1, AK8963_POWER_DOWN);
}

// Set Fuse ROM access mode
void GeegrowAK8963::setFuseRomAccessBit() {
	writeByte(AK8963_CNTL1, AK8963_FUSE_ROM);
}

// Calculates the magnetometer bias in the x, y, and z axes after device initialization.
void GeegrowAK8963::calibrate() {
	uint16_t sampleCount = 10;
	int16_t maxValues[3] = {1, 1, 1};
	int16_t minValues[3] = {0, 0, 0};
	float avgBias[3]  = {0, 0, 0};

	if (deviceId != AK8963_WHOAMI) {
		debugPrint(F("Wrong deviceId. Emerg. exit"));
		return;
	}

	// Make sure scale factor has been calculated
	setScaleFactor();

	Serial.println(F("Mag calibration: Wave device in a figure 8 until done!"));
	Serial.println(F("Waiting 5 seconds to get ready followed by 10 seconds of sampling"));
	delay(5000);

	uint16_t i = 0;
	
	while ((measurementMode == M_100HZ && i < 1000) || i < 80) {
		debugPrint(F("Calibration itteration: "));
		debugPrintln(i);
		
		read(&rawData[0]);

		// Update X min/max values
		maxValues[0] = max(rawData[0], maxValues[0]);
		minValues[0] = min(rawData[0], minValues[0]);
		// Update Y min/max values
		maxValues[1] = max(rawData[1], maxValues[1]);
		minValues[1] = min(rawData[1], minValues[1]);
		// Update Z min/max values
		maxValues[2] = max(rawData[2], maxValues[2]);
		minValues[2] = min(rawData[2], minValues[2]);

		if (measurementMode == M_100HZ) {
			delay(12);
		} else {
			delay(135);
		}

		i++;
	}

	// Calculate 'average' x/y/z bias values
	avgBias[0]  = (float)(minValues[0] + maxValues[0]) / 2;
	avgBias[1]  = (float)(minValues[1] + maxValues[1]) / 2;
	avgBias[2]  = (float)(minValues[2] + maxValues[2]) / 2;
	
	// Save mag biases in G for main program
	bias[0] = avgBias[0] * scaleFactor * factoryCalibration[0];
	bias[1] = avgBias[1] * scaleFactor * factoryCalibration[1];
	bias[2] = avgBias[2] * scaleFactor * factoryCalibration[2];

	debugPrint(F("BIAS x: ")); debugPrintln(bias[0]);
	debugPrint(F("BIAS y: ")); debugPrintln(bias[1]);
	debugPrint(F("BIAS z: ")); debugPrintln(bias[2]);

	Serial.println(F("Mag calibration done!"));
}
