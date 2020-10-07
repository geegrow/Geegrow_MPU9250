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
        myIMU.init();
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

    /* Madgwick filter */
    Madgwick::update(
        myIMU.ax, myIMU.ay, myIMU.az, 
        myIMU.gx*PI/180.0f, myIMU.gy*PI/180.0f, myIMU.gz*PI/180.0f, 
        myIMU.my, myIMU.mx, myIMU.mz
    );

    /* Print angles */
    Serial.print("yaw :\t");
    Serial.print(Madgwick::yaw);
    Serial.println();
    Serial.print("pitch :\t");
    Serial.print(Madgwick::pitch);
    Serial.println();
    Serial.print("roll :\t");
    Serial.print(Madgwick::roll);
    Serial.println(); Serial.println();


    digitalWrite(myLed, !digitalRead(myLed));
    delay(300);

}
