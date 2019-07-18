#ifndef GEEGROW_AK8963_H
#define GEEGROW_AK8963_H
#include <Arduino.h>
#include <Wire.h>
#include <I2CTransport.h>

#define I2C_PREVENT_FREEZING false

/*=============================================================================*/
/*                      Magnetometer(AK8963) Registers                         */
/*=============================================================================*/
#define AK8963_REG_WHOAMI 0x00  //[Read-only]   Device ID (Fixed value 0x48)
#define INFO              0x01  //[Read-only]   Info (Device information for AKM)
#define AK8963_ST1        0x02  //[Read-only]   Statis 1
                                //              DRDY [bit 1] Data Ready:
                                //                      0 - Normal,
                                //                      1 - Data is ready.
                                //              DOR [bit 2] Data Overrun:
                                //                      0 - Normal,
                                //                      1 - Data overrun
#define AK8963_XOUT_L     0x03  //[Read-only]   HXL[7:0]: X-axis measurement data lower 8bit
#define AK8963_XOUT_H     0x04  //[Read-only]   HXH[15:8]: X-axis measurement data higher 8bit
#define AK8963_YOUT_L     0x05  //[Read-only]   HYL[7:0]: Y-axis measurement data lower 8bit
#define AK8963_YOUT_H     0x06  //[Read-only]   HYH[15:8]: Y-axis measurement data higher 8bit
#define AK8963_ZOUT_L     0x07  //[Read-only]   HZL[7:0]: Z-axis measurement data lower 8bit
#define AK8963_ZOUT_H     0x08  //[Read-only]   HZH[15:8]: Z-axis measurement data higher 8bit
#define AK8963_ST2        0x09  //[Read-only]   Statis 2
                                //              HOFL [bit 3] Overflow:
                                //                      0 - Normal,
                                //                      1 - Sensor overflow.
                                //              BITM [bit 4] Output setting:
                                //                      0 - 14-bit output,
                                //                      1 - 16-bit output
#define AK8963_CNTL1      0x0A  //[Write/read]  Control1
                                //              MODE[bit 3:0]:
                                //                      0000 - Power down,
                                //                      0001 - single-measurement,
                                //                      0010 - Cont. meas. mode 1 (8Hz),
                                //                      0110 - Cont. meas. mode 2 (100Hz),
                                //                      0100 - External trigger meas. mode,
                                //                      1000 - self-test,
                                //                      1111 - Fuse ROM
                                //              BIT[bit 4]:
                                //                      0 - 14-bit output
                                //                      1 - 16-bit output
#define AK8963_CNTL2      0x0B  //[Write/read]  Control2
                                //              SRST[bit 0] Soft reset:
                                //                      0 - Normal
                                //                      1 - Reset
                                // When “1” is set, all registers are initialized. After reset, SRST bit turns to “0” automatically.
#define AK8963_ASTC       0x0C  //[Write/read]  Self Test Control
                                //              SELF[bit 6] Self test:
                                //                      0 - Normal
                                //                      1 - Generate magnetic field for self-test
#define AK8963_ASAX       0x10  // [Read-only]  ASAX[7:0]: Magnetic sensor X-axis sensitivity adjustment value
#define AK8963_ASAY       0x11  // [Read-only]  ASAY[7:0]: Magnetic sensor Y-axis sensitivity adjustment value
#define AK8963_ASAZ       0x12  // [Read-only]  ASAZ[7:0]: Magnetic sensor Z-axis sensitivity adjustment value

/*=============================================================================*/
/*                      Magnetometer registers values                          */
/*=============================================================================*/
#define AK8963_ADDRESS      0x0C // Device I2C address, sould be 0x0C
#define AK8963_WHOAMI       0x48 // WHOAMI response, should be 0x48
// ST1 register
#define AK8963_DATA_READY   0x01
#define AK8963_DATA_OVERRUN 0x02
// ST2 register
// #define AK8963_OVERFLOW     0x80
#define AK8963_OVERFLOW     0x08
#define AK8963_16BIT_MODE   0x08
// CNTL1 reguster
#define AK8963_POWER_DOWN               0x00
#define AK8963_SINGLE_MEASUREMENT       0x01
#define AK8963_8HZ_CONTINUOUS_MODE      0x02
#define AK8963_100HZ_CONTINUOUS_MODE    0x06
#define AK8963_EXT_TRIGGER_MODE         0x04
#define AK8963_SELF_MODE                0x08
#define AK8963_FUSE_ROM                 0x0F
// CNTL2 register
#define AK8963_SOFT_RESET   0x01
// ASTC register
#define AK8963_SELF_TEST    0x01

class GeegrowAK8963 {

    public:
        enum ADCSensitivity {
          MFS_14BITS = 0, // 14-bit output [0.6 mG per LSB]
          MFS_16BITS      // 15-bit output [0.15 mG per LSB]
        };

        enum MeasurementMode {
          M_8HZ     = AK8963_8HZ_CONTINUOUS_MODE,  // 8 Hz update
          M_100HZ   = AK8963_100HZ_CONTINUOUS_MODE // 100 Hz update
        };

        int16_t rawData[3] = {0, 0, 0};
        uint8_t deviceAddr = AK8963_ADDRESS;
        uint8_t deviceId;

        // Magnetometer scale factor
        float scaleFactor;
        // Variables to hold latest sensor data values
        float mx, my, mz;
        // Factory mag calibration and mag bias
        float factoryCalibration[3] = {0, 0, 0},
              bias[3] = {0, 0, 0};

        // Public method declarations
        GeegrowAK8963();

        void setScaleFactor();

        void read(int16_t *);
        void update();
        void init();
        void calibrate();

        // Choose magnetometer resolution before call initAK8963()
        void setAdcSensitivity14Bit() { sensitivity = ADCSensitivity::MFS_14BITS; }
        void setAdcSensitivity16Bit() { sensitivity = ADCSensitivity::MFS_16BITS; }
        // Choose 8Hz or 100Hz data update rate before call initAK8963()
        void setMeasurementMode8hz() { measurementMode = MeasurementMode::M_8HZ; }
        void setMeasurementMode100hz() { measurementMode = MeasurementMode::M_100HZ; }
        // Enable/disable debug mode
        void enableDebugMode() { debugMode = true; }
        void disableDebugMode() { debugMode = false; }

        uint8_t writeByte(uint8_t registerAddress, uint8_t data) {
          return I2CTransport::writeByte(deviceAddr, registerAddress, data, I2C_PREVENT_FREEZING);
        }

        // Read a byte from the given register address from device using I2C
        uint8_t readByte(uint8_t registerAddress) {
          return I2CTransport::readByte(deviceAddr, registerAddress);
        }

        // Read 1 or more bytes from given register and device using I2C
        uint8_t readBytes(uint8_t registerAddress, uint8_t length, uint8_t * dest ) {
          return I2CTransport::readBytes(deviceAddr, registerAddress, length, dest);
        }

    protected:
        // Choose either 14-bit or 16-bit magnetometer resolution
        ADCSensitivity sensitivity = MFS_16BITS;

        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
        MeasurementMode measurementMode = M_8HZ;

        bool debugMode;

        void setPowerDownBit();
        void setFuseRomAccessBit();

        #define debugPrint(...)    if (debugMode == true) { Serial.print(__VA_ARGS__); }
        #define debugPrintln(...)  if (debugMode == true) { Serial.println(__VA_ARGS__); }
};

#endif // GEEGROW_AK8963_H
