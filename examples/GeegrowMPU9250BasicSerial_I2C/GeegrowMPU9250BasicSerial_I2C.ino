#include <Geegrow_MPU9250.h>
#include "quaternionFilters.h"

// Pin definitions
int myLed  = 13;  // Set up pin 13 led for toggling

Geegrow_MPU9250 myIMU;
/*  In I2CTransport we can use either Wire or I2C clients. In case of using I2C client,
    prevent_freezing option can be enabled. I2C_PREVENT_FREEZING is defined
    in AK8963.h file. This option is disabled by default.
    To use I2C client you should uncomment the following line. Default client is Wire. */
// #define USE_I2C_CLIENT

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
    // Serial.print(F("MPU9250 I AM 0x"));
    // Serial.print(c, HEX);
    // Serial.print(F(" I should be 0x"));
    // Serial.println(0x71, HEX);

    if (c == 0x71) {    // WHO_AM_I should always be 0x71
        Serial.println(F("MPU9250 is online..."));

        // // Start by performing self test and reporting values
        // myIMU.MPU9250SelfTest(myIMU.selfTest);
        // Serial.print(F("x-axis self test: acceleration trim within : "));
        // Serial.print(myIMU.selfTest[0], 1); Serial.println("% of factory value");
        // Serial.print(F("y-axis self test: acceleration trim within : "));
        // Serial.print(myIMU.selfTest[1], 1); Serial.println("% of factory value");
        // Serial.print(F("z-axis self test: acceleration trim within : "));
        // Serial.print(myIMU.selfTest[2], 1); Serial.println("% of factory value");
        // Serial.print(F("x-axis self test: gyration trim within : "));
        // Serial.print(myIMU.selfTest[3], 1); Serial.println("% of factory value");
        // Serial.print(F("y-axis self test: gyration trim within : "));
        // Serial.print(myIMU.selfTest[4], 1); Serial.println("% of factory value");
        // Serial.print(F("z-axis self test: gyration trim within : "));
        // Serial.print(myIMU.selfTest[5], 1); Serial.println("% of factory value");

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
    /* Update values from sensors */
    myIMU.update();

    // /* Print scaled values from sensors */
    // Serial.print("accel :   \t");
    // Serial.print(myIMU.ax);
    // Serial.print("\t");
    // Serial.print(myIMU.ay);
    // Serial.print("\t");
    // Serial.println(myIMU.az);

    // Serial.print("gyro :   \t");
    // Serial.print(myIMU.gx);
    // Serial.print("\t");
    // Serial.print(myIMU.gy);
    // Serial.print("\t");
    // Serial.println(myIMU.gz);

    // Serial.print("compass :\t");
    // Serial.print(myIMU.mx);
    // Serial.print("\t");
    // Serial.print(myIMU.my);
    // Serial.print("\t");
    // Serial.println(myIMU.mz);
    // Serial.println();

    /* Before using filter it's necessery to update time delta */
    myIMU.updateTime();

    /* Madgwick filter */
    MadgwickQuaternionUpdate(
        myIMU.ax, myIMU.ay, myIMU.az, 
        myIMU.gx*PI/180.0f, myIMU.gy*PI/180.0f, myIMU.gz*PI/180.0f, 
        myIMU.my, myIMU.mx, myIMU.mz,
        myIMU.deltat
    );

    float* q = getQ();
    /* Get pitch, roll and yaw */
    float pitch, yaw, roll;
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    yaw   -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    roll  *= 180.0f / PI;
    /* Print angles */
    Serial.print("yaw :\t");
    Serial.print(yaw);
    Serial.println();
    Serial.print("pitch :\t");
    Serial.print(pitch);
    Serial.println();
    Serial.print("roll :\t");
    Serial.print(roll);
    Serial.println(); Serial.println();


    digitalWrite(myLed, !digitalRead(myLed));
    delay(300);

}
