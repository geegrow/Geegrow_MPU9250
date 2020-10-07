/*!
 * @file Geegrow_MPU9250.cpp
 *
 * This is a library for MPU9250 IMU sensor
 * https://www.geegrow.ru
 *
 * @section author Author
 * Written by Geegrow
 *
 * @section license License
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Geegrow_MPU9250.h"

/*=============================================================================*/
/*     Define functions to access acceleration, gyroscope temperature data.    */
/*=============================================================================*/

Geegrow_MPU9250::Geegrow_MPU9250() {
    setAScaleFactor();
    setGScaleFactor();
}

/**
 * Each gyroscope measurement is 16-bit value which has defined in GYRO_CONFIG
 * register, GYRO_FS_SEL[bit 4:3]. For each full scale setting is shown in the 
 * table below:
 * 
 * GYRO_FS_SEL | Full Scale Range   | LSB Sensitivity
 * ------------+--------------------+----------------
 * 00          | +/- 250 degrees/s  | 131 LSB/deg/s
 * 01          | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 10          | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 11          | +/- 2000 degrees/s | 16.4 LSB/deg/s
 *
 * Here's a how to calculate LSB/(deg/s):
 */
void Geegrow_MPU9250::setGScaleFactor() {
    switch (gyroScale) {
        case GFS_250DPS:
            gScaleFactor = 32768.0f / 250.0f;
            break;
        case GFS_500DPS:
            gScaleFactor = 32768.0f / 500.0f;
            break;
        case GFS_1000DPS:
            gScaleFactor = 32768.0f / 1000.0f;
            break;
        case GFS_2000DPS:
            gScaleFactor = 32768.0f / 2000.0f;
            break;
    }
}

/**
 * Each accelerometer measurement value has defined in ACCEL_CONFIG register,
 * ACCEL_FS_SEL[bit 4:3]. For each full scale setting is shown in the 
 * table below:
 * 
 * ACCEL_FS_SEL | Full Scale Range | LSB Sensitivity
 * -------------+------------------+----------------
 * 00           | +/- 2g           | 16384 LSB/g
 * 01           | +/- 4g           |  8192 LSB/g
 * 10           | +/- 8g           |  4096 LSB/g
 * 11           | +/- 16g          |  2048 LSB/g
 *
 * Here's a how to calculate LSB/(deg/g):
 */
void Geegrow_MPU9250::setAScaleFactor() {
    switch (accelScale) {
        case AFS_2G:
            aScaleFactor = 16384; // 32768/2 = 8192
            break;
        case AFS_4G:
            aScaleFactor = 8192; // 32768/4 = 4096
            break;
        case AFS_8G:
            aScaleFactor = 4096; // 32768/8 = 2048
            break;
        case AFS_16G:
            aScaleFactor = 2048; // 32768/16 = 1024
            break;
    }
}

/**
 * Get gyroscope Digital Low Pass Filter setting (ACCEL_FCHOICE & DLPF_CFG).
 *
 * +-----------+----------+----------------------------------------+----------------+------------+
 * |  FCHOICE  |          |                Gyroscope               |      Temperature Sensor     |
 * +-----------+ DLPF_CFG +----------------------------------------+-----------------------------+
 * | <1> | <0> |          | Bandwidth (Hz) | Delay (ms) | Fs (kHz) | Bandwidth (Hz) | Delay (ms) |
 * +-----+-----+----------+----------------+------------+----------+----------------+------------+
 * |  x  |  0  |    x     |      8800      |    0.064   |    32    |      4000      |    0.04    |
 * |  0  |  1  |    x     |      3600      |    0.11    |    32    |      4000      |    0.04    |
 * |  1  |  1  |    0     |       250      |    0.97    |     8    |      4000      |    0.04    |
 * |  1  |  1  |    1     |       184      |    2.90    |     1    |       188      |    1.90    |
 * |  1  |  1  |    2     |        92      |    3.90    |     1    |        98      |    2.80    |
 * |  1  |  1  |    3     |        41      |    5.90    |     1    |        42      |    4.80    |
 * |  1  |  1  |    4     |        20      |    9.90    |     1    |        20      |    8.30    |
 * |  1  |  1  |    5     |        10      |   17.85    |     1    |        10      |   13.40    |
 * |  1  |  1  |    6     |         5      |   33.48    |     1    |         5      |   18.60    |
 * |  1  |  1  |    7     |      3600      |    0.17    |     8    |      4000      |    0.04    |
 * +-----+-----+----------+----------------+------------+----------+----------------+------------+
 *
 */
void Geegrow_MPU9250::getGyroFilterConfiguration(uint8_t &fchoice, uint8_t &dlpf) {
    fchoice = 0x03; //0b00000011
    dlpf = 0;       //0b00000000

    switch (gyroSampleRate) {
        case GFS_32000HZ:
            switch (gyroBandwidth) {
                case GBW_8800HZ:
                    fchoice = 0x00;
                    break;

                default: //3600HZ
                    fchoice = 0x01;
                    break;
            }
            break;

        case GFS_8000HZ:
            switch (gyroBandwidth) {
                case GBW_3600HZ:
                    dlpf = 0x07;
                    break;

                default: //250HZ
                    dlpf = 0x00;
                    break;
            }
            break;

        case GFS_1000HZ:
            switch (gyroBandwidth) {
                case GBW_184HZ:
                    dlpf = 0x01;
                    break;
                case GBW_92HZ:
                    dlpf = 0x02;
                    break;
                case GBW_41HZ:
                    dlpf = 0x03;
                    break;
                case GBW_20HZ:
                    dlpf = 0x04;
                    break;
                case GBW_10HZ:
                    dlpf = 0x05;
                    break;
                default: //5HZ
                    dlpf = 0x06;
                    break;
            }
            break;
    }
}

/**
 * Get accelerometer Digital Low Pass Filter setting (FCHOICE & A_DLPF_CFG).
 * This function will work properly only in Normal Mode!
 * @TODO make similar Low-Power Mode funcion
 *
 * +-----------+----------+------------------------------------------+
 * |           |          |                   Output                 |
 * | AFCHOICE  | DLPF_CFG +------------------------------------------+
 * |           |          | Bandwidth (Hz) | Delay (ms) | Rate (kHz) |
 * +-----+-----+----------+----------------+------------+------------+
 * |     0     |    x     |      1130      |    0.75    |      4     |
 * |     1     |    0     |       460      |    1.94    |      1     |
 * |     1     |    1     |       184      |    5.80    |      1     |
 * |     1     |    2     |        92      |    7.80    |      1     |
 * |     1     |    3     |        41      |   11.80    |      1     |
 * |     1     |    4     |        20      |   19.80    |      1     |
 * |     1     |    5     |        10      |   35.70    |      1     |
 * |     1     |    6     |         5      |   66.96    |      1     |
 * |     1     |    7     |       460      |    1.94    |      1     | <---+
 * +-----------+----------+----------------+------------+------------+     |
 * Wу will never use this mode because seems it's just a copy of row №3 ---+
 */
void Geegrow_MPU9250::getAccelFilterConfiguration(uint8_t &fchoice, uint8_t &dlpf) {
    fchoice = 0x08; //0b00001000
    dlpf = 0;       //0b00000000

    switch (accelSampleRate) {
        case AFS_4000HZ:
            fchoice = 0x00;
            break;

        case AFS_1000HZ:
            switch (accelBandwidth) {
                case ABW_460HZ:
                    dlpf = 0x00;
                    break;
                case ABW_184HZ:
                    dlpf = 0x01;
                    break;
                case ABW_92HZ:
                    dlpf = 0x02;
                    break;
                case ABW_41HZ:
                    dlpf = 0x03;
                    break;
                case ABW_20HZ:
                    dlpf = 0x04;
                    break;
                case ABW_10HZ:
                    dlpf = 0x05;
                    break;
                default: //5HZ
                    dlpf = 0x06;
                    break;
            }
            break;
    }
}

/**
 * Set gyro config register - GYRO_CONFIG
 */
void Geegrow_MPU9250::configureGyro() {
    uint8_t fchoice, dlpf = 0;
    getGyroFilterConfiguration(fchoice, dlpf);

    // Wride Digital Low Pass Filter value
    writeBits(CONFIG, dlpf);

    // Set sample rate divider register - SMPLRT_DIV.
    // Equation for calculate sample rate:
    // SampleRate = gyroscope_output_rate/(1 + SMPLRT_DIV)
    writeByte(SMPLRT_DIV, sampleRateDivider);

    // Read current GYRO_CONFIG register value
    uint8_t cfg = readByte(GYRO_CONFIG);

    // Clear self-test bits [7:5]
    cfg &= ~0xE0;
    
    // Set Fchoice_b bits [1:0]
    // Fchoice_b is inverted value of Fchoice
    cfg |= 0x03;
    cfg &= ~fchoice;
    
    // Clear gyro scale bits [4:3]
    cfg &= ~0x18;

    // Set gyro scale
    cfg |= gyroScale << 3;

    // Write new GYRO_CONFIG value to register
    writeByte(GYRO_CONFIG, cfg);
}

/**
 * Set accel config register - ACCEL_CONFIG and ACCEL_CONFIG2
 */
void Geegrow_MPU9250::configureAccel() {
    uint8_t fchoice, dlpf = 0;
    getGyroFilterConfiguration(fchoice, dlpf);

    // Set accelerometer full-scale range configuration
    // Get current ACCEL_CONFIG register value
    uint8_t cfg = readByte(ACCEL_CONFIG);
    
    // Clear self-test bits [7:5]
    cfg &= ~0xE0;

    // Clear accel scale bits [4:3]
    cfg &= ~0x18;

    // Set accel scale bits
    cfg |= accelScale << 3;

    // Write new ACCEL_CONFIG register value
    writeByte(ACCEL_CONFIG, cfg);

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by
    // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is
    // 1.13 kHz

    // Get current ACCEL_CONFIG2 register value
    cfg = readByte(ACCEL_CONFIG2);

    // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    cfg &= ~0x0F;

    // Set Fchoice_b bit [3]
    // Fchoice_b is inverted value of Fchoice
    cfg |= 0x08;
    cfg &= ~fchoice;

    // Set A_DLPFCFG bit [2:0]
    cfg |= dlpf;

    // Write new ACCEL_CONFIG2 register value
    writeByte(ACCEL_CONFIG2, cfg);
}

/*
 * Setter for sampleRateDivider
 */
void Geegrow_MPU9250::setSampleRateDivider(uint8_t srd) {
    sampleRateDivider = srd;
}

/*
    Single function to init both MPU and compass with default parameters
*/
void Geegrow_MPU9250::init()
{
    // // Start by performing self test and reporting values
    // this->MPU9250SelfTest(this->selfTest);
    // Serial.print(F("x-axis self test: acceleration trim within : "));
    // Serial.print(this->selfTest[0], 1); Serial.println("% of factory value");
    // Serial.print(F("y-axis self test: acceleration trim within : "));
    // Serial.print(this->selfTest[1], 1); Serial.println("% of factory value");
    // Serial.print(F("z-axis self test: acceleration trim within : "));
    // Serial.print(this->selfTest[2], 1); Serial.println("% of factory value");
    // Serial.print(F("x-axis self test: gyration trim within : "));
    // Serial.print(this->selfTest[3], 1); Serial.println("% of factory value");
    // Serial.print(F("y-axis self test: gyration trim within : "));
    // Serial.print(this->selfTest[4], 1); Serial.println("% of factory value");
    // Serial.print(F("z-axis self test: gyration trim within : "));
    // Serial.print(this->selfTest[5], 1); Serial.println("% of factory value");

    this->setSampleRateDivider(8);
    this->setGyroBandwidth(this->GBW_41HZ);
    this->setGyroSampleRate(this->GFS_1000HZ);
    this->setGyroScale(this->GFS_250DPS);
    this->setAccelBandwidth(this->ABW_41HZ);
    this->setAccelSampleRate(this->AFS_1000HZ);
    this->setAccelScale(this->AFS_16G);
    // Calibrate gyro and accelerometers, must be done before running initMPU9250();
    this->calibrate();
    // Initialize device for active mode read of acclerometer, gyroscope and temperature
    this->initMPU9250();
    /* ! compass will not work if IMU is not initialized! */
    // this->compass.disableDebugMode();
    // this->compass.setAdcSensitivity16Bit();
    this->compass.setMeasurementMode8hz();
    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate magnetometer bias and scale.
    this->compass.init();
    this->calibrateCompass();
    delay(2000); // Add delay to see results before serial spew of data
}

/*
 * Setup 
 * 1. gyro, accel and termometer sample rate and low pass filter
 * 2. configure PLL
 * 3. configure interrupts
 */
void Geegrow_MPU9250::initMPU9250() {
    //Make sure tar Gyro and Accel scale factor is set
    setAScaleFactor();
    setGScaleFactor();

    // wake up device
    // Clear sleep mode bit (6), enable all sensors
    writeByte(PWR_MGMT_1, 0x00);

    // Wait for all registers to reset
    delay(100);

    // Get stable time source
    deviceSelectAutoPll();
    
    delay(200);

    // Configure Gyro and Thermometer
    configureGyro();
    
    // Configure Accelerometer
    configureAccel();

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
    // until interrupt cleared, clear on read of INT_STATUS, and enable
    // I2C_BYPASS_EN so additional chips can join the I2C bus and all can be
    // controlled by the Arduino as master.
    writeByte(INT_PIN_CFG, 0x22);
    
    // Enable data ready (bit 0) interrupt
    writeByte(INT_ENABLE, 0x01);
    delay(100);
}

/*
 * Setup compass.
 * Attention! Compass initialisation only possible after MPU9250 initialisation
 */
void Geegrow_MPU9250::initAK8963() {
    compass.init();
}

/**
 * Read raw accelerometer data into variable
 */
void Geegrow_MPU9250::readAccelRawData(int16_t * data) {
    uint8_t tmpData[6]; 
    // Read the six (XH, XL, YH, YL, ZH, ZL) ACCEL raw data registers into tmpData
    readBytes(ACCEL_XOUT_H, 6, tmpData);

    // Turn the MSB and LSB into a signed 16-bit value
    data[0] = ((uint16_t)tmpData[0] << 8) | tmpData[1];
    data[1] = ((uint16_t)tmpData[2] << 8) | tmpData[3];  
    data[2] = ((uint16_t)tmpData[4] << 8) | tmpData[5]; 
}

/**
 * Store last accelerometer data into ax, ay and az
 */
void Geegrow_MPU9250::updateAccel() {
    // Get raw value from accelerometer
    readAccelRawData(accelRawData);

    ax = (float) (accelRawData[0] / (float) aScaleFactor);
    ay = (float) (accelRawData[1] / (float) aScaleFactor);
    az = (float) (accelRawData[2] / (float) aScaleFactor);
}

/**
 * Read raw gyro data into variable
 */
void Geegrow_MPU9250::readGyroRawData(int16_t * data) {
    uint8_t tmpData[6]; 
    // Read the six (XH, XL, YH, YL, ZH, ZL) GYRO raw data registers into tmpData
    readBytes(GYRO_XOUT_H, 6, tmpData);
   
    // Turn the MSB and LSB into a signed 16-bit value
    data[0] = ((uint16_t)tmpData[0] << 8) | tmpData[1];
    data[1] = ((uint16_t)tmpData[2] << 8) | tmpData[3];  
    data[2] = ((uint16_t)tmpData[4] << 8) | tmpData[5];
}

/**
 * Store last gyro data into gx, gy and gz
 */
void Geegrow_MPU9250::updateGyro() {
    // Get raw value from gyro
    readGyroRawData(gyroRawData);

    gx = (float) gyroRawData[0] / (float) gScaleFactor;
    gy = (float) gyroRawData[1] / (float) gScaleFactor;
    gz = (float) gyroRawData[2] / (float) gScaleFactor;
}

/**
 * Store last compass (magnetometer) data into mx, my and mz
 */
void Geegrow_MPU9250::updateCompass() {
    compass.update();
    mx = compass.mx;
    my = compass.my;
    mz = compass.mz;
}

/**
 * Store last gyro, accelerometer and compass(magnetometer) data
 */
void Geegrow_MPU9250::update() {
    updateAccel();
    updateGyro();
    updateCompass();
}

int16_t Geegrow_MPU9250::readTempData() {
    uint8_t rawData[2]; // x/y/z gyro register data stored here
    // Read the two raw data registers sequentially into data array
    readBytes(TEMP_OUT_H, 2, rawData);
    // Turn the MSB and LSB into a 16-bit value
    return (int16_t) (((uint16_t)rawData[0] << 8) | rawData[1]);
}

// Calculate the time the last update took for use in the quaternion filters
// TODO: This doesn't really belong in this class.
// void Geegrow_MPU9250::updateTime() {
//     this->Now = micros();

//     // Set integration time by time elapsed since last filter update
//     this->deltat = ((this->Now - this->lastUpdate) / 1000000.0f);
//     this->lastUpdate = this->Now;

//     this->sum += this->deltat; // sum for averaging filter update rate
//     this->sumCount++;
// }

/**
 * Calibrate MPU9250 accelerometer and gyro.
 * Attention! Run this function before initMPU9250 obly, because it may 
 * change configuration registers value and not set it back.
 */
void Geegrow_MPU9250::calibrate() {
    uint8_t samplingLength = 100;
    int32_t accelBiasRegister[3] = {0, 0, 0};
    int32_t gyroBias[3]  = {0, 0, 0};
    int32_t accelBias[3] = {0, 0, 0};

    deviceReset();
    delay(100);

    deviceSelectAutoPll();

    // Enable all accel and gyro axis (X, Y, Z)
    writeByte(PWR_MGMT_2, 0x00);
    delay(200);

    // Configure device for bias calculation
    // Disable all interrupts
    writeByte(INT_ENABLE, 0x00);
    // Disable FIFO
    writeByte(FIFO_EN, 0x00);
    // Turn on internal clock source
    writeByte(PWR_MGMT_1, 0x00);
    // Disable I2C master
    writeByte(I2C_MST_CTRL, 0x00);
    
    // Configure MPU9250 gyro and accelerometer for bias calculation
    // Set low-pass filter to 188 Hz
    writeByte(CONFIG, 0x01);
    // Set sample rate to 1 kHz
    writeByte(SMPLRT_DIV, 0x00);
    // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    writeByte(GYRO_CONFIG, 0x00);
    // Set accelerometer full-scale to 2g, maximum sensitivity
    writeByte(ACCEL_CONFIG, 0x00);

    // Clear gyro bias register
    writeGyroBiasRegister(gyroBias);

    Serial.println(F("Gyro and accel calibration will start in 5 seconds!"));
    Serial.println(F("It will take about 1s. Don't touch device!"));
    delay(5000);

    for (uint8_t i = 0; i < samplingLength; i++) {
        // Read accel data for averaging
        readAccelRawData(accelRawData);
        
        accelBias[0] += (int32_t) accelRawData[0];
        accelBias[1] += (int32_t) accelRawData[1];
        accelBias[2] += (int32_t) accelRawData[2];

        // Read gyro data for averaging
        readGyroRawData(gyroRawData);

        gyroBias[0] += (int32_t) gyroRawData[0];
        gyroBias[1] += (int32_t) gyroRawData[1];
        gyroBias[2] += (int32_t) gyroRawData[2];

        delay(10);
    }

    accelBias[0] = (int32_t) accelBias[0]/samplingLength;
    accelBias[1] = (int32_t) accelBias[1]/samplingLength;
    accelBias[2] = (int32_t) accelBias[2]/samplingLength;

    gyroBias[0] = (int32_t) gyroBias[0]/samplingLength;
    gyroBias[1] = (int32_t) gyroBias[1]/samplingLength;
    gyroBias[2] = (int32_t) gyroBias[2]/samplingLength;

    // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    if (accelBias[2] > 0L) {
        accelBias[2] -= (int32_t)ACCEL_FULL_SCALE;
    } else {
        accelBias[2] += (int32_t)ACCEL_FULL_SCALE;
    }

    
    // Construct the gyro biases for push to the hardware gyro bias registers,
    // which are reset to zero upon device startup.
    // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input
    // format.
    gyroBias[0] = (int32_t) -(gyroBias[0]/4);
    gyroBias[1] = (int32_t) -(gyroBias[1]/4);
    gyroBias[2] = (int32_t) -(gyroBias[2]/4);

    // Store new gyro bias value to bias register
    writeGyroBiasRegister(gyroBias);

    // Read factory (or previous calibration) bias value from accel bias register
    readAccelBiasRegister(accelBiasRegister);

    accelBiasRegister[0] -= (accelBias[0]/8);
    accelBiasRegister[1] -= (accelBias[1]/8);
    accelBiasRegister[2] -= (accelBias[2]/8);

    // Store new bias value to accel bias register
    writeAccelBiasRegister(accelBiasRegister);

    Serial.println(F("Acccel abd gyro calibration done!"));
}

/**
 *  @brief Read biases to the accel bias 6050 registers.
 *  This function reads from the MPU6050 accel offset cancellations registers.
 *  The format are G in +-8G format. The register is initialized with OTP 
 *  factory trim values.
 *  @param[in]  accel_bias  returned structure with the accel bias
 *  @return     0 if successful.
 */
void Geegrow_MPU9250::readAccelBiasRegister(int32_t *bias) {
    // int32_t accel_bias_reg[3] = {0, 0, 0};
    uint8_t data[2];
    // Read factory accelerometer trim values
    readBytes(XA_OFFSET_H, 2, data);
    bias[0] = (int32_t) (((uint16_t)data[0] << 8) | data[1]);

    readBytes(YA_OFFSET_H, 2, data);
    bias[1] = (int32_t) (((uint16_t)data[0] << 8) | data[1]);

    readBytes(ZA_OFFSET_H, 2, data);
    bias[2] = (int32_t) (((uint16_t)data[0] << 8) | data[1]);
}

/**
 *  Push biases to the gyro bias registers.
 *  This function expects biases in +/-16G format, and
 *  place it to registers.
 *  @param bias  New biases array(x, y, z).
 */
void Geegrow_MPU9250::writeAccelBiasRegister(const int32_t *bias) {
    uint8_t data[6];

    data[0] = (bias[0] >> 8) & 0xFF;
    data[1] = (bias[0])      & 0xFE;
    data[2] = (bias[1] >> 8) & 0xFF;
    data[3] = (bias[1])      & 0xFE;
    data[4] = (bias[2] >> 8) & 0xFF;
    data[5] = (bias[2])      & 0xFE;

    writeByte(XA_OFFSET_H, data[0]);
    writeByte(XA_OFFSET_L, data[1]);
    writeByte(YA_OFFSET_H, data[2]);
    writeByte(YA_OFFSET_L, data[3]);
    writeByte(ZA_OFFSET_H, data[4]);
    writeByte(ZA_OFFSET_L, data[5]);
}

/**
 *  Push biases to the gyro bias registers.
 *  This function expects biases in 32.8 LSB/(deg/s) (+/-1000deg/s) format, and
 *  place it to registers.
 *  @param bias  New biases array(x, y, z).
 */
void Geegrow_MPU9250::writeGyroBiasRegister(const int32_t *bias) {
    uint8_t data[6];

    // Biases are additive, so change sign of gyro biases
    data[0] = (bias[0]  >> 8) & 0xFF;
    data[1] = (bias[0])       & 0xFF;
    data[2] = (bias[1]  >> 8) & 0xFF;
    data[3] = (bias[1])       & 0xFF;
    data[4] = (bias[2]  >> 8) & 0xFF;
    data[5] = (bias[2])       & 0xFF;

    // Push gyro biases to hardware registers
    writeByte(XG_OFFSET_H, data[0]);
    writeByte(XG_OFFSET_L, data[1]);
    writeByte(YG_OFFSET_H, data[2]);
    writeByte(YG_OFFSET_L, data[3]);
    writeByte(ZG_OFFSET_H, data[4]);
    writeByte(ZG_OFFSET_L, data[5]);
}

// Function which accumulates magnetometer data after device initialization.
// It calculates the bias and scale in the x, y, and z axes.
void Geegrow_MPU9250::calibrateCompass() {
    compass.calibrate();
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
// Should return percent deviation from factory trim values, +/- 14 or less
// deviation is a pass.
void Geegrow_MPU9250::MPU9250SelfTest(float * destination)
{
    uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
    uint8_t selfTest[6];
    int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
    float factoryTrim[6];
    uint8_t FS = 0;

    // Set gyro sample rate to 1 kHz
    writeByte(SMPLRT_DIV, 0x00);
    // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
    writeByte(CONFIG, 0x02);
    // Set full scale range for the gyro to 250 dps
    writeByte(GYRO_CONFIG, 1<<FS);
    // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
    writeByte(ACCEL_CONFIG2, 0x02);
    // Set full scale range for the accelerometer to 2 g
    writeByte(ACCEL_CONFIG, 1<<FS);

    // Get average current values of gyro and acclerometer
    for (int ii = 0; ii < 200; ii++)
    {
        // Read the six raw data registers into data array
        readBytes(ACCEL_XOUT_H, 6, rawData);
        // Turn the MSB and LSB into a signed 16-bit value
        aAvg[0] += (int16_t)(((uint16_t)rawData[0] << 8) | rawData[1]) ;
        aAvg[1] += (int16_t)(((uint16_t)rawData[2] << 8) | rawData[3]) ;
        aAvg[2] += (int16_t)(((uint16_t)rawData[4] << 8) | rawData[5]) ;

        // Read the six raw data registers sequentially into data array
        readBytes(GYRO_XOUT_H, 6, rawData);
        // Turn the MSB and LSB into a signed 16-bit value
        gAvg[0] += (int16_t)(((uint16_t)rawData[0] << 8) | rawData[1]) ;
        gAvg[1] += (int16_t)(((uint16_t)rawData[2] << 8) | rawData[3]) ;
        gAvg[2] += (int16_t)(((uint16_t)rawData[4] << 8) | rawData[5]) ;
    }

    // Get average of 200 values and store as average current readings
    for (int ii =0; ii < 3; ii++)
    {
        aAvg[ii] = (int32_t) aAvg[ii]/200;
        gAvg[ii] = (int32_t) gAvg[ii]/200;
    }

    // Configure the accelerometer for self-test
    // Enable self test on all three axes and set accelerometer range to +/- 2 g
    writeByte(ACCEL_CONFIG, 0xE0);
    // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    writeByte(GYRO_CONFIG,  0xE0);
    delay(25);  // Delay a while to let the device stabilize

    // Get average self-test values of gyro and acclerometer
    for (int ii = 0; ii < 200; ii++)
    {
        // Read the six raw data registers into data array
        readBytes(ACCEL_XOUT_H, 6, rawData);
        // Turn the MSB and LSB into a signed 16-bit value
        aSTAvg[0] += (int16_t)(((uint16_t)rawData[0] << 8) | rawData[1]) ;
        aSTAvg[1] += (int16_t)(((uint16_t)rawData[2] << 8) | rawData[3]) ;
        aSTAvg[2] += (int16_t)(((uint16_t)rawData[4] << 8) | rawData[5]) ;

        // Read the six raw data registers sequentially into data array
        readBytes(GYRO_XOUT_H, 6, rawData);
        // Turn the MSB and LSB into a signed 16-bit value
        gSTAvg[0] += (int16_t)(((uint16_t)rawData[0] << 8) | rawData[1]) ;
        gSTAvg[1] += (int16_t)(((uint16_t)rawData[2] << 8) | rawData[3]) ;
        gSTAvg[2] += (int16_t)(((uint16_t)rawData[4] << 8) | rawData[5]) ;
    }

    // Get average of 200 values and store as average self-test readings
    for (int ii =0; ii < 3; ii++)
    {
        aSTAvg[ii] = (int32_t) aSTAvg[ii]/200;
        gSTAvg[ii] = (int32_t) gSTAvg[ii]/200;
    }

    // Configure the gyro and accelerometer for normal operation
    writeByte(ACCEL_CONFIG, 0x00);
    writeByte(GYRO_CONFIG,  0x00);
    delay(25);  // Delay a while to let the device stabilize

    // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
    // X-axis accel self-test results
    selfTest[0] = readByte(SELF_TEST_X_ACCEL);
    // Y-axis accel self-test results
    selfTest[1] = readByte(SELF_TEST_Y_ACCEL);
    // Z-axis accel self-test results
    selfTest[2] = readByte(SELF_TEST_Z_ACCEL);
    // X-axis gyro self-test results
    selfTest[3] = readByte(SELF_TEST_X_GYRO);
    // Y-axis gyro self-test results
    selfTest[4] = readByte(SELF_TEST_Y_GYRO);
    // Z-axis gyro self-test results
    selfTest[5] = readByte(SELF_TEST_Z_GYRO);

    // Retrieve factory self-test value from self-test code reads
    // FT[Xa] factory trim calculation
    factoryTrim[0] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[0] - 1.0) ));
    // FT[Ya] factory trim calculation
    factoryTrim[1] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[1] - 1.0) ));
    // FT[Za] factory trim calculation
    factoryTrim[2] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[2] - 1.0) ));
    // FT[Xg] factory trim calculation
    factoryTrim[3] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[3] - 1.0) ));
    // FT[Yg] factory trim calculation
    factoryTrim[4] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[4] - 1.0) ));
    // FT[Zg] factory trim calculation
    factoryTrim[5] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[5] - 1.0) ));

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim
    // of the Self-Test Response
    // To get percent, must multiply by 100
    for (int i = 0; i < 3; i++)
    {
        // Report percent differences
        destination[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i]
            - 100.;
        // Report percent differences
        destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]
            - 100.;
    }
}
