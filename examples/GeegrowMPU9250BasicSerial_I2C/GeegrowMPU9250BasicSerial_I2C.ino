#include <Geegrow_MPU9250.h>
//#include "quaternionFilters.h"

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling
bool ledState = false;

Geegrow_MPU9250 myIMU;
/*  In I2CTransport we can use either Wire or I2C clients. In case of using I2C client,
    prevent_freezing option can be enabled. I2C_PREVENT_FREEZING is defined
    in AK8963.h file. This option is disabled by default.
    To use I2C client you should uncomment the following line. Default client is Wire. */
#define USE_I2C_CLIENT

#ifdef USE_I2C_CLIENT
    I2C* i2c_client;
#endif /* USE_I2C_CLIENT */

void setup() {
    Serial.begin(9600);
    delay(2000);
    pinMode(myLed, OUTPUT);

    #ifdef USE_I2C_CLIENT
        i2c_client = new I2C();
        I2CTransport::setClient(i2c_client);
    #else
        Wire.begin();
        delay(2000);
        I2CTransport::setClient(&Wire);
    #endif /* USE_I2C_CLIENT */

    I2CTransport::begin();
    
    // Read the WHO_AM_I register, this is a good test of communication
    uint8_t c = myIMU.readByte(WHO_AM_I_MPU9250);
    Serial.print(F("MPU9250 I AM 0x"));
    Serial.print(c, HEX);
    Serial.print(F(" I should be 0x"));
    Serial.println(0x71, HEX);

    if (c == 0x71) {    // WHO_AM_I should always be 0x71
        Serial.println(F("MPU9250 is online..."));

        // Start by performing self test and reporting values
        myIMU.MPU9250SelfTest(myIMU.selfTest);
        Serial.print(F("x-axis self test: acceleration trim within : "));
        Serial.print(myIMU.selfTest[0], 1); Serial.println("% of factory value");
        Serial.print(F("y-axis self test: acceleration trim within : "));
        Serial.print(myIMU.selfTest[1], 1); Serial.println("% of factory value");
        Serial.print(F("z-axis self test: acceleration trim within : "));
        Serial.print(myIMU.selfTest[2], 1); Serial.println("% of factory value");
        Serial.print(F("x-axis self test: gyration trim within : "));
        Serial.print(myIMU.selfTest[3], 1); Serial.println("% of factory value");
        Serial.print(F("y-axis self test: gyration trim within : "));
        Serial.print(myIMU.selfTest[4], 1); Serial.println("% of factory value");
        Serial.print(F("z-axis self test: gyration trim within : "));
        Serial.print(myIMU.selfTest[5], 1); Serial.println("% of factory value");

        myIMU.setSampleRateDivider(8);
        
        myIMU.setGyroBandwidth(myIMU.GBW_41HZ);
        myIMU.setGyroSampleRate(myIMU.GFS_1000HZ);
        myIMU.setGyroScale(myIMU.GFS_250DPS);

        myIMU.setAccelBandwidth(myIMU.ABW_41HZ);
        myIMU.setAccelSampleRate(myIMU.AFS_1000HZ);
        myIMU.setAccelScale(myIMU.AFS_16G);

        // Calibrate gyro and accelerometers, must be done before running initMPU9250();
        myIMU.calibrate();
        
        // Initialize device for active mode read of acclerometer, gyroscope and temperature
        myIMU.initMPU9250();
        
        /* ! compass will not work if IMU is not initialized! */
        // myIMU.compass.disableDebugMode();
        // myIMU.compass.setAdcSensitivity16Bit();
        myIMU.compass.setMeasurementMode8hz();

        // The next call delays for 4 seconds, and then records about 15 seconds of
        // data to calculate magnetometer bias and scale.
        myIMU.compass.init();
        myIMU.calibrateCompass();

        delay(2000); // Add delay to see results before serial spew of data
    }
}

void loop() {
    myIMU.update();
  
    Serial.print("accel :   \t");
    Serial.print(myIMU.ax);
    Serial.print("\t");
    Serial.print(myIMU.ay);
    Serial.print("\t");
    Serial.println(myIMU.az);
    
    Serial.print("gyro :   \t");
    Serial.print(myIMU.gx);
    Serial.print("\t");
    Serial.print(myIMU.gy);
    Serial.print("\t");
    Serial.println(myIMU.gz);
    
    
    Serial.print("compass :\t");
    Serial.print(myIMU.mx);
    Serial.print("\t");
    Serial.print(myIMU.my);
    Serial.print("\t");
    Serial.println(myIMU.mz);
    Serial.println("");
    
    digitalWrite(myLed, !digitalRead(myLed));
    
    delay(300);

  /*
    // If intPin goes high, all data registers have new data
    // On interrupt, check if data ready interrupt
    if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
                 myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
                 myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
                 myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
    } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

    // Must be called before updating quaternions!
    myIMU.updateTime();

    // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
    // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
    // (+ up) of accelerometer and gyro! We have to make some allowance for this
    // orientationmismatch in feeding the output to the quaternion filter. For the
    // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
    // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
    // modified to allow any convenient orientation convention. This is ok by
    // aircraft orientation standards! Pass gyro rate as rad/s
    MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

    if (!AHRS) {
    myIMU.delt_t = millis() - myIMU.count;

    if (myIMU.delt_t > 500) {
      if(SerialDebug) {
        // Print acceleration values in milligs!
        Serial.print("X-acceleration: "); Serial.print(1000 * myIMU.ax);
        Serial.print(" mg ");
        Serial.print("Y-acceleration: "); Serial.print(1000 * myIMU.ay);
        Serial.print(" mg ");
        Serial.print("Z-acceleration: "); Serial.print(1000 * myIMU.az);
        Serial.println(" mg ");

        // Print gyro values in degree/sec
        Serial.print("X-gyro rate: "); Serial.print(myIMU.gx, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Y-gyro rate: "); Serial.print(myIMU.gy, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Z-gyro rate: "); Serial.print(myIMU.gz, 3);
        Serial.println(" degrees/sec");

        // Print mag values in degree/sec
        Serial.print("X-mag field: "); Serial.print(myIMU.mx);
        Serial.print(" mG ");
        Serial.print("Y-mag field: "); Serial.print(myIMU.my);
        Serial.print(" mG ");
        Serial.print("Z-mag field: "); Serial.print(myIMU.mz);
        Serial.println(" mG");

        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
        // Print temperature in degrees Centigrade
        Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);
        Serial.println(" degrees C");
      }

      myIMU.count = millis();
      digitalWrite(myLed, !digitalRead(myLed));  // toggle led
    } // if (myIMU.delt_t > 500)
    } // if (!AHRS)
    else
    {
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;

    // update LCD once per half-second independent of read rate
    if (myIMU.delt_t > 500)
    {
      if(SerialDebug)
      {
        Serial.print("ax = ");  Serial.print((int)1000 * myIMU.ax);
        Serial.print(" ay = "); Serial.print((int)1000 * myIMU.ay);
        Serial.print(" az = "); Serial.print((int)1000 * myIMU.az);
        Serial.println(" mg");

        Serial.print("gx = ");  Serial.print(myIMU.gx, 2);
        Serial.print(" gy = "); Serial.print(myIMU.gy, 2);
        Serial.print(" gz = "); Serial.print(myIMU.gz, 2);
        Serial.println(" deg/s");

        Serial.print("mx = ");  Serial.print((int)myIMU.mx);
        Serial.print(" my = "); Serial.print((int)myIMU.my);
        Serial.print(" mz = "); Serial.print((int)myIMU.mz);
        Serial.println(" mG");

        Serial.print("q0 = ");  Serial.print(*getQ());
        Serial.print(" qx = "); Serial.print(*(getQ() + 1));
        Serial.print(" qy = "); Serial.print(*(getQ() + 2));
        Serial.print(" qz = "); Serial.println(*(getQ() + 3));
      }

    // Define output variables from updated quaternion---these are Tait-Bryan
    // angles, commonly used in aircraft orientation. In this coordinate system,
    // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
    // x-axis and Earth magnetic North (or true North if corrected for local
    // declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the
    // Earth is positive, up toward the sky is negative. Roll is angle between
    // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
    // arise from the definition of the homogeneous rotation matrix constructed
    // from quaternions. Tait-Bryan angles as well as Euler angles are
    // non-commutative; that is, the get the correct orientation the rotations
    // must be applied in the correct order which for this configuration is yaw,
    // pitch, and then roll.
    // For more see
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // which has additional links.
      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                    * *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                    * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;

      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw  -= 8.5;
      myIMU.roll *= RAD_TO_DEG;
      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 500)
    } // if (AHRS)
  */
}
