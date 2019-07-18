/*!
 * @file Geegrow_MPU9250.h
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
#ifndef _MPU9250_H_
#define _MPU9250_H_
#include <Arduino.h>
#include <Wire.h>
#include "AK8963.h"


#define SERIAL_DEBUG true

// See also MPU-9250 Register Map and Descriptions, Revision 4.0,
// RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in above
// document; the MPU9250 and MPU9150 are virtually identical but the latter has
// a different register map
/*=============================================================================*/
/*               Register Map for Gyroscope and Accelerometer                  */
/*=============================================================================*/

#define SELF_TEST_X_GYRO  0x00  //[Read/Write]  XG_ST_DATA[7:0] Gyroscope Self-Test Registers
#define SELF_TEST_Y_GYRO  0x01  //[Read/Write]  YG_ST_DATA[7:0] Gyroscope Self-Test Registers
#define SELF_TEST_Z_GYRO  0x02  //[Read/Write]  ZG_ST_DATA[7:0] Gyroscope Self-Test Registers

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D  //[Read/Write]  XA_ST_DATA[7:0] Accelerometer Self-Test Registers
#define SELF_TEST_Y_ACCEL 0x0E  //[Read/Write]  YA_ST_DATA[7:0] Accelerometer Self-Test Registers
#define SELF_TEST_Z_ACCEL 0x0F  //[Read/Write]  ZA_ST_DATA[7:0] Accelerometer Self-Test Registers

#define SELF_TEST_A       0x10
#define XG_OFFSET_H       0x13  //[Read/Write]  X_OFFS_USR[15:8] High byte of Gyro Offset Register
#define XG_OFFSET_L       0x14  //[Read/Write]  X_OFFS_USR[7:0] Low byte of Gyro Offset Register
#define YG_OFFSET_H       0x15  //[Read/Write]  Y_OFFS_USR[15:8] High byte of Gyro Offset Register
#define YG_OFFSET_L       0x16  //[Read/Write]  Y_OFFS_USR[7:0] Low byte of Gyro Offset Register
#define ZG_OFFSET_H       0x17  //[Read/Write]  Z_OFFS_USR[15:8] High byte of Gyro Offset Register
#define ZG_OFFSET_L       0x18  //[Read/Write]  Z_OFFS_USR[7:0] Low byte of Gyro Offset Register
#define SMPLRT_DIV        0x19  //[Read/Write]  SMPLRT_DIV[7:0] Sample Rate Divider SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV)
#define CONFIG            0x1A  //[Read/Write]  Configuration
                                //              FIFO_MODE[bit 6] Fifo mode:
                                //                      0 - when the fifo is full, additional writes
                                //                          will be written to the fifo, replacing the oldest data.
                                //                      1 - when the fifo is full, additional writes
                                //                          will not be written to fifo
                                //              EXT_SYNC_SET[bit 5:3] Enables the FSYNC pin data to be sampled:
                                //                      000 - function disabled
                                //                      001 - TEMP_OUT_L[0]
                                //                      010 - GYRO_XOUT_L[0]
                                //                      011 - GYRO_YOUT_L[0]
                                //                      100 - GYRO_ZOUT_L[0]
                                //                      101 - ACCEL_XOUT_L[0]
                                //                      110 - ACCEL_YOUT_L[0]
                                //                      111 - ACCEL_ZOUT_L[0]
                                //              DLPF_CFG[bit 2:0] Filter configuration.. see datasheet
#define GYRO_CONFIG       0x1B  //[Read/Write]  Gyroscope Configuration
                                //              XGYRO_Cten[bit 7] X Gyro self-test
                                //              YGYRO_Cten[bit 6] Y Gyro self-test
                                //              ZGYRO_Cten[bit 5] Z Gyro self-test
                                //              GYRO_FS_SEL[bit 4:3] Gyro Full Scale Select:
                                //                      00 - +250dps
                                //                      01 - +500 dps
                                //                      10 - +1000 dps
                                //                      11 - +2000 dps
                                //              Reserved[bit 2] Not used
                                //              Fchoice_b[bit 1:0] Used to bypass DLPF.. see datasheet
#define ACCEL_CONFIG      0x1C  //[Read/Write]  Accelerometer Configuration
                                //              ax_st_en[bit 7] X Accel self-test
                                //              ay_st_en[bit 6] Y Accel self-test
                                //              az_st_en[bit 5] Z Accel self-test
                                //              ACCEL_FS_SEL[bit 4:3] Accel Full Scale Select:
                                //                      00 - ±2g
                                //                      01 - ±4g
                                //                      10 - ±8g
                                //                      11 - ±16g
                                //              Reserved[bit 2:0] Not used
#define ACCEL_CONFIG2     0x1D  //[Read/Write]  Accelerometer Configuration 2
                                //              Reserved[bit 7:6, 5:4] Not used
                                //              accel_fchoice_b[bit 3] Used to bypass DLPF.. see datasheet
                                //              A_DLPFCFG[bit 2:0] Accelerometer low pass filter setting.. see datasheet
#define LP_ACCEL_ODR      0x1E  //[Read/Write]  Low Power Accelerometer ODR Control
                                //              Reserved[bit 7:4] Not used
                                //              lposc_clksel[bit 3:0] Sets the frequency of waking up the chip
                                //              to take a sample of accel data – the low power accel Output Data Rate:
                                //                      0000 - Output Frequency: 0.24 Hz
                                //                      0001 - Output Frequency: 0.49 Hz
                                //                      0010 - Output Frequency: 0.98 Hz
                                //                      0011 - Output Frequency: 1.95 Hz
                                //                      0100 - Output Frequency: 3.91 Hz
                                //                      0101 - Output Frequency: 7.81 Hz
                                //                      0110 - Output Frequency: 15.63 Hz
                                //                      0111 - Output Frequency: 31.25 Hz
                                //                      1000 - Output Frequency: 62.50 Hz
                                //                      1001 - Output Frequency: 125 Hz
                                //                      1010 - Output Frequency: 250 Hz
                                //                      1011 - Output Frequency: 500 Hz
#define WOM_THR           0x1F  //[Read/Write]  Wake-on Motion Threshold.. see datasheet
#define MOT_DUR           0x20  //[Read/Write]  Duration counter threshold for motion interrupt
                                //              generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR          0x21  //[Read/Write]  Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR         0x22  //[Read/Write]  Duration counter threshold for zero motion
                                //              interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN           0x23  //[Read/Write]  FIFO Enable register
                                //              TEMP_OUT[bit 7]:
                                //                      1 - Write TEMP_OUT_H and TEMP_OUT_L to the FIFO at the sample rate;
                                //                      0 - function is disabled
                                //              GYRO_XOUT[bit 6]:
                                //                      1 - Write GYRO_XOUT_H and GYRO_XOUT_L to the FIFO at the sample rate;
                                //                      0 - function is disabled
                                //              GYRO_YOUT[bit 5]:
                                //                      1 - Write GYRO_YOUT_H and GYRO_YOUT_L to the FIFO at the sample rate;
                                //                      0 - function is disabled
                                //              GYRO_ZOUT[bit 4]:
                                //                      1 - Write GYRO_ZOUT_H and GYRO_ZOUT_L to the FIFO at the sample rate;
                                //                      0 - function is disabled
                                //              ACCEL[bit 3]:
                                //                      1 - write ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L,
                                //                          ACCEL_ZOUT_H, and ACCEL_ZOUT_L to the FIFO at the sample rate
                                //                      0 - function is disabled
                                //              SLV_2[bit 2]:
                                //                      1 - write EXT_SENS_DATA registers associated to SLV_2 (as determined by I2C_SLV0_CTRL,
                                //                          I2C_SLV1_CTRL, and I2C_SL20_CTRL) to the FIFO at the sample rate;
                                //                      0 - function is disabled
                                //              SLV_1[bit 1]:
                                //                      1 - write EXT_SENS_DATA registers associated to SLV_1 (as determined by
                                //                          I2C_SLV0_CTRL and I2C_SLV1_CTRL) to the FIFO at the sample rate;
                                //                      0 - function is disabled
                                //              SLV_0[bit 0]:
                                //                      1 - write EXT_SENS_DATA registers associated to SLV_0 (as determined by
                                //                          I2C_SLV0_CTRL) to the FIFO at the sample rate;
                                //                      0 - function is disabled
#define I2C_MST_CTRL      0x24  //[Read/Write]  I2C Master Control
                                //              MULT_MST_EN[bit 7]: Enables multi-master capability.
                                //              WAIT_FOR_ES[bit 6]: Delays the data ready interrupt until external sensor data is loaded.
                                //              SLV_3_FIFO_EN[bit 5]:
                                //                      1 - write EXT_SENS_DATA registers associated to SLV_3 (as determined by
                                //                          I2C_SLV0_CTRL and I2C_SLV1_CTRL and I2C_SLV2_CTRL) to the FIFO at the sample rate;
                                //                      0 - function is disabled
                                //              I2C_MST_P_NSR[bit 4]: This bit controls the I2C Master’s transition from one slave read to the next slave read.
                                //                          If 0, there is a restart between reads. If 1, there is a stop between reads.
                                //              I2C_MST_CLK[bit 3:0]: I2C_MST_CLK is a 4 bit unsigned value which configures a divider on the MPU- 9250
                                //                          internal 8MHz clock. It sets the I2C master clock speed. See datasheed...
#define I2C_SLV0_ADDR     0x25  //[Read/Write]  I2C Slave 0 Control (I2C_SLV0_ADDR) Register
                                //              I2C_SLV0_RNW[bit 7]: Enables multi-master capability.
                                //                      1 - Transfer is a read
                                //                      0 - Transfer is a write
                                //              I2C_ID_0[bit 6:0]: Physical address of I2C slave 0
#define I2C_SLV0_REG      0x26  //[Read/Write]  I2C Slave 0 Control (I2C_SLV0_REG) Register [bit 7:0]: I2C slave 0 register address from where to begin data transfer
#define I2C_SLV0_CTRL     0x27  //[Read/Write]  I2C Slave 0 Control (I2C_SLV0_CTRL) Register
                                //              I2C_SLV0_EN[bit 7]:
                                //                      1 - Enable reading data from this slave at the sample rate and storing data at the first
                                //                          available EXT_SENS_DATA register, which is always EXT_SENS_DATA_00 for I2C slave 0.
                                //                      0 - Disabled
                                //              I2C_SLV0_BYTE_SW[bit 6]:
                                //                      1 - Swap bytes when reading both the low and high byte of a word. Note there is nothing to
                                //                          swap after reading the first byte if I2C_SLV0_REG[0] = 1, or if the last byte read has
                                //                          a register address lsb = 0
                                //                      0 - no swapping occurs, bytes are written in order read.
                                //              I2C_SLV0_REG_DIS[bit 5]: When set, the transaction does not write a register value, it will only read/write data.
                                //              I2C_SLV0_GRP[bit 4]: See datasheed...
                                //              I2C_SLV0_LENG[bit 3:0]: Number of bytes to be read from I2C slave 0
#define I2C_SLV1_ADDR     0x28  //[Read/Write]  I2C Slave 1 Control (I2C_SLV1_ADDR) Register
                                //              I2C_SLV1_RNW[bit 7]:
                                //                      1 - Transfer is a read
                                //                      0 - Transfer is a write
                                //              I2C_ID_1[bit 6:0]: Physical address of I2C slave 1
#define I2C_SLV1_REG      0x29  //[Read/Write]  I2C Slave 1 Control (I2C_SLV1_REG) Register [bit 7:0]: I2C slave 1 register address from where to begin data transfer
#define I2C_SLV1_CTRL     0x2A  //[Read/Write]  I2C Slave 1 Control (I2C_SLV1_CTRL) Register
                                //              I2C_SLV1_EN[bit 7]:
                                //                      1 - Enable reading data from this slave at the sample rate and storing data at the first
                                //                          available EXT_SENS_DATA register as determined by I2C_SLV1_EN and I2C_SLV1_LENG.
                                //                      0 - Disabled
                                //              I2C_SLV1_BYTE_SW[bit 6]:
                                //                      1 - Swap bytes when reading both the low and high byte of a word. Note there is nothing to
                                //                          swap after reading the first byte if I2C_SLV1_REG[0] = 1, or if the last byte read has
                                //                          a register address lsb = 0
                                //                      0 - no swapping occurs, bytes are written in order read.
                                //              I2C_SLV1_REG_DIS[bit 5]: When set, the transaction does not write a register value, it will only read/write data.
                                //              I2C_SLV1_GRP[bit 4]: See datasheed...
                                //              I2C_SLV1_LENG[bit 3:0]: Number of bytes to be read from I2C slave 1
#define I2C_SLV2_ADDR     0x2B  //[Read/Write]  I2C Slave 2 Control (I2C_SLV2_ADDR) Register
                                //              I2C_SLV2_RNW[bit 7]:
                                //                      1 - Transfer is a read
                                //                      0 - Transfer is a write
                                //              I2C_ID_2[bit 6:0]: Physical address of I2C slave 2
#define I2C_SLV2_REG      0x2C  //[Read/Write]  I2C Slave 2 Control (I2C_SLV2_REG) Register [bit 7:0]: I2C slave 2 register address from where to begin data transfer
#define I2C_SLV2_CTRL     0x2D  //[Read/Write]  I2C Slave 2 Control (I2C_SLV2_CTRL) Register
                                //              I2C_SLV2_EN[bit 7]:
                                //                      1 - Enable reading data from this slave at the sample rate and storing data at the first
                                //                          available EXT_SENS_DATA register as determined by I2C_SLV2_EN and I2C_SLV2_LENG.
                                //                      0 - Disabled
                                //              I2C_SLV2_BYTE_SW[bit 6]:
                                //                      1 - Swap bytes when reading both the low and high byte of a word. Note there is nothing to
                                //                          swap after reading the first byte if I2C_SLV2_REG[0] = 1, or if the last byte read has
                                //                          a register address lsb = 0
                                //                      0 - no swapping occurs, bytes are written in order read.
                                //              I2C_SLV2_REG_DIS[bit 5]: When set, the transaction does not write a register value, it will only read/write data.
                                //              I2C_SLV2_GRP[bit 4]: See datasheed...
                                //              I2C_SLV2_LENG[bit 3:0]: Number of bytes to be read from I2C slave 2
#define I2C_SLV3_ADDR     0x2E  //[Read/Write]  I2C Slave 3 Control (I2C_SLV3_ADDR) Register
                                //              I2C_SLV3_RNW[bit 7]:
                                //                      1 - Transfer is a read
                                //                      0 - Transfer is a write
                                //              I2C_ID_3[bit 6:0]: Physical address of I2C slave 3
#define I2C_SLV3_REG      0x2F  //[Read/Write]  I2C Slave 3 Control (I2C_SLV3_REG) Register [bit 7:0]: I2C slave 3 register address from where to begin data transfer
#define I2C_SLV3_CTRL     0x30  //[Read/Write]  I2C Slave 3 Control (I2C_SLV3_CTRL) Register
                                //              I2C_SLV3_EN[bit 7]:
                                //                      1 - Enable reading data from this slave at the sample rate and storing data at the first
                                //                          available EXT_SENS_DATA register as determined by I2C_SLV3_EN and I2C_SLV3_LENG.
                                //                      0 - Disabled
                                //              I2C_SLV3_BYTE_SW[bit 6]:
                                //                      1 - Swap bytes when reading both the low and high byte of a word. Note there is nothing to
                                //                          swap after reading the first byte if I2C_SLV3_REG[0] = 1, or if the last byte read has
                                //                          a register address lsb = 0
                                //                      0 - no swapping occurs, bytes are written in order read.
                                //              I2C_SLV3_REG_DIS[bit 5]: When set, the transaction does not write a register value, it will only read/write data.
                                //              I2C_SLV3_GRP[bit 4]: See datasheed...
                                //              I2C_SLV3_LENG[bit 3:0]: Number of bytes to be read from I2C slave 3
#define I2C_SLV4_ADDR     0x31  //[Read/Write]  I2C Slave 4 Control (I2C_SLV4_ADDR) Register
                                //              I2C_SLV4_RNW[bit 7]:
                                //                      1 - Transfer is a read
                                //                      0 - Transfer is a write
                                //              I2C_ID_4[bit 6:0]: Physical address of I2C slave 4
#define I2C_SLV4_REG      0x32  //[Read/Write]  I2C Slave 4 Control (I2C_SLV4_REG) Register [bit 7:0]: I2C slave 4 register address from where to begin data transfer
#define I2C_SLV4_DO       0x33  //[Read/Write]  I2C Slave 4 Control (I2C_SLV4_DO) Register
                                //              I2C_SLV4_DO[bit 7:0]: Data to be written to I2C Slave 4 if enabled.
#define I2C_SLV4_CTRL     0x34  //[Read/Write]  I2C Slave 4 Control (I2C_SLV4_CTRL) Register
                                //              I2C_SLV4_EN[bit 7]:
                                //                      1 - Enable reading data from this slave at the sample rate and storing data at the first
                                //                          available EXT_SENS_DATA register as determined by I2C_SLV4_EN and I2C_SLV4_LENG.
                                //                      0 - Disabled
                                //              I2C_SLV4_BYTE_SW[bit 6]:
                                //                      1 - Swap bytes when reading both the low and high byte of a word. Note there is nothing to
                                //                          swap after reading the first byte if I2C_SLV4_REG[0] = 1, or if the last byte read has
                                //                          a register address lsb = 0
                                //                      0 - no swapping occurs, bytes are written in order read.
                                //              I2C_SLV4_REG_DIS[bit 5]: When set, the transaction does not write a register value, it will only read/write data.
                                //              I2C_SLV4_GRP[bit 4]: See datasheed...
                                //              I2C_SLV4_LENG[bit 3:0]: Number of bytes to be read from I2C slave 4
#define I2C_SLV4_DI       0x35  //[Read]        I2C Slave 4 Control (I2C_SLV4_DI) Register
                                //              I2C_SLV4_DI[bit 7:0]: Data read from I2C Slave 4.
#define I2C_MST_STATUS    0x36  //[Read/C]      I2C Master Status
                                //              PASS_THROUGH[bit 7]: Status of FSYNC interrupt – used as a way to pass an external interrupt through
                                //                  this chip to the host. If enabled in the INT_PIN_CFG register by asserting bit FSYNC_INT_EN and if the
                                //                  FSYNC signal transitions from low to high, this will cause an interrupt. A read of this register clears
                                //                  all status bits in this register.
                                //              I2C_SLV4_DONE[bit 6]: Asserted when I2C slave 4’s transfer is complete, will cause an interrupt if bit
                                //                  I2C_MST_INT_EN in the INT_ENABLE register is asserted, and if the SLV4_DONE_INT_EN bit is asserted in the
                                //                  I2C_SLV4_CTRL register.
                                //              I2C_LOST_ARB[bit 5]: Asserted when I2C slave looses arbitration of the I2C bus, will cause an interrupt if
                                //                  bit I2C_MST_INT_EN in the INT_ENABLE register is asserted.
                                //              I2C_SLV4_NACK[bit 4]: Asserted when slave 4 receives a nack, will cause an interrupt if bit I2C_MST_INT_EN
                                //                  in the INT_ENABLE register is asserted.
                                //              I2C_SLV3_NACK[bit 3]: Asserted when slave 3 receives a nack, will cause an interrupt if bit I2C_MST_INT_EN
                                //                  in the INT_ENABLE register is asserted.
                                //              I2C_SLV2_NACK[bit 2]: Asserted when slave 2 receives a nack, will cause an interrupt if bit I2C_MST_INT_EN
                                //                  in the INT_ENABLE register is asserted.
                                //              I2C_SLV1_NACK[bit 1]: Asserted when slave 1 receives a nack, will cause an interrupt if bit I2C_MST_INT_EN
                                //                  in the INT_ENABLE register is asserted.
                                //              I2C_SLV0_NACK[bit 0]: Asserted when slave 0 receives a nack, will cause an interrupt if bit I2C_MST_INT_EN
                                //                  in the INT_ENABLE register is asserted.
#define INT_PIN_CFG       0x37  //[Read/Write]  INT Pin / Bypass Enable Configuration
                                //              ACTL[bit 7]:
                                //                      1 - The logic level for INT pin is active low.
                                //                      0 - The logic level for INT pin is active high.
                                //              OPEN[bit 6]:
                                //                      1 - INT pin is configured as open drain.
                                //                      0 - INT pin is configured as push-pull.
                                //              LATCH_INT_EN[bit 5]:
                                //                      1 - INT pin level held until interrupt status is cleared.
                                //                      0 - INT pin indicates interrupt pulse’s is width 50us.
                                //              INT_ANYRD_2CLEAR[bit 4]:
                                //                      1 - Interrupt status is cleared if any read operation is performed.
                                //                      0 - Interrupt status is cleared only by reading INT_STATUS register
                                //              ACTL_FSYNC[bit 3]:
                                //                      1 - The logic level for the FSYNC pin as an interrupt is active low.
                                //                      0 - The logic level for the FSYNC pin as an interrupt is active high.
                                //              FSYNC_INT_MODE_EN[bit 2]:
                                //                      1 - This enables the FSYNC pin to be used as an interrupt.
                                //                      0 - This disables the FSYNC pin from causing an interrupt.
                                //              BYPASS_EN[bit 1]: When asserted, the i2c_master interface pins(ES_CL and ES_DA) will go into
                                //                      ‘bypass mode’ when the i2c master interface is disabled. The pins will float high
                                //                      due to the internal pull-up if not enabled and the i2c master interface is disabled.
                                //              RESERVED[bit 0]
#define INT_ENABLE        0x38  //[Read/Write]  Interrupt Enable
                                //              RESERVED[bit 7]
                                //              WOM_EN[bit 6]:
                                //                      1 - Enable interrupt for wake on motion to propagate to interrupt pin.
                                //                      0 - function is disabled.
                                //              RESERVED[bit 5]
                                //              FIFO_OVERFLOW_EN[bit 4]:
                                //                      1 - Enable interrupt for fifo overflow to propagate to interrupt pin.
                                //                      0 - function is disabled.
                                //              FSYNC_INT_EN[bit 3]:
                                //                      1 - Enable Fsync interrupt to propagate to interrupt pin.
                                //                      0 - function is disabled.
                                //              RESERVED[bit 2]
                                //              RESERVED[bit 1]
                                //              RAW_RDY_EN[bit 0]:
                                //                      1 - Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin. The timing of
                                //                          the interrupt can vary depending on the setting in register 36 I2C_MST_CTRL, bit [6] WAIT_FOR_ES.
                                //                      0 - function is disabled.
#define DMP_INT_STATUS    0x39  // Check DMP status
#define INT_STATUS        0x3A  //[Read/C]       Interrupt Status
                                //              RESERVED[bit 7]
                                //              WOM_EN[bit 6]: 1 – Wake on motion interrupt occurred.
                                //              RESERVED[bit 5]
                                //              FIFO_OVERFLOW_EN[bit 4]: 1 – Fifo Overflow interrupt occurred. Note that the oldest data is has been dropped
                                //                      from the fifo.
                                //              FSYNC_INT_EN[bit 3]: 1 – Fsync interrupt occurred.
                                //              RESERVED[bit 2]
                                //              RESERVED[bit 1]
                                //              RAW_RDY_EN[bit 0]: 1 – Sensor Register Raw Data sensors are updated and Ready to be read. The timing of the
                                //                      interrupt can vary depending on the setting in register 36 I2C_MST_CTRL, bit [6] WAIT_FOR_ES.
#define ACCEL_XOUT_H      0x3B  //[Read-only]   ACCEL_XOUT_H[7:0]: High byte of accelerometer x-axis data.
#define ACCEL_XOUT_L      0x3C  //[Read-only]   ACCEL_XOUT_L[7:0]: Low byte of accelerometer x-axis data.
#define ACCEL_YOUT_H      0x3D  //[Read-only]   ACCEL_YOUT_H[7:0]: High byte of accelerometer y-axis data.
#define ACCEL_YOUT_L      0x3E  //[Read-only]   ACCEL_YOUT_L[7:0]: Low byte of accelerometer y-axis data.
#define ACCEL_ZOUT_H      0x3F  //[Read-only]   ACCEL_ZOUT_H[7:0]: High byte of accelerometer z-axis data.
#define ACCEL_ZOUT_L      0x40  //[Read-only]   ACCEL_ZOUT_L[7:0]: Low byte of accelerometer z-axis data.
#define TEMP_OUT_H        0x41  //[Read-only]   TEMP_OUT_H[7:0]: High byte of the temperature sensor output
#define TEMP_OUT_L        0x42  //[Read-only]   TEMP_OUT_L[7:0]: Low byte of the temperature sensor output:
                                //                      TEMP_degC = ((TEMP_OUT – RoomTemp_, Where Temp_degC is the temperature in degrees C measured by the
                                //                      temperature sensor. TEMP_OUT is the actual output of the temperature sensor.
#define GYRO_XOUT_H       0x43  //[Read-only]   GYRO_XOUT_H[7:0]: High byte of the X-Axis gyroscope output
#define GYRO_XOUT_L       0x44  //[Read-only]   GYRO_XOUT_L[7:0]: Low byte of the X-Axis gyroscope output
#define GYRO_YOUT_H       0x45  //[Read-only]   GYRO_YOUT_H[7:0]: High byte of the Y-Axis gyroscope output
#define GYRO_YOUT_L       0x46  //[Read-only]   GYRO_YOUT_L[7:0]: Low byte of the Y-Axis gyroscope output
#define GYRO_ZOUT_H       0x47  //[Read-only]   GYRO_ZOUT_H[7:0]: High byte of the Z-Axis gyroscope output
#define GYRO_ZOUT_L       0x48  //[Read-only]   GYRO_ZOUT_L[7:0]: Low byte of the Z-Axis gyroscope output
// #define MAG               0x49
#define EXT_SENS_DATA_00  0x49
#define EXT_SENS_DATA_01  0x4A
#define EXT_SENS_DATA_02  0x4B
#define EXT_SENS_DATA_03  0x4C
#define EXT_SENS_DATA_04  0x4D
#define EXT_SENS_DATA_05  0x4E
#define EXT_SENS_DATA_06  0x4F
#define EXT_SENS_DATA_07  0x50
#define EXT_SENS_DATA_08  0x51
#define EXT_SENS_DATA_09  0x52
#define EXT_SENS_DATA_10  0x53
#define EXT_SENS_DATA_11  0x54
#define EXT_SENS_DATA_12  0x55
#define EXT_SENS_DATA_13  0x56
#define EXT_SENS_DATA_14  0x57
#define EXT_SENS_DATA_15  0x58
#define EXT_SENS_DATA_16  0x59
#define EXT_SENS_DATA_17  0x5A
#define EXT_SENS_DATA_18  0x5B
#define EXT_SENS_DATA_19  0x5C
#define EXT_SENS_DATA_20  0x5D
#define EXT_SENS_DATA_21  0x5E
#define EXT_SENS_DATA_22  0x5F
#define EXT_SENS_DATA_23  0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO       0x63  //[Read/Write]  I2C Slave 0 Data Out
                                //              I2C_SLV0_DO[bit 7:0]: Data out when slave 0 is set to write
#define I2C_SLV1_DO       0x64  //[Read/Write]  I2C Slave 1 Data Out
                                //              I2C_SLV0_D1[bit 7:0]: Data out when slave 1 is set to write
#define I2C_SLV2_DO       0x65  //[Read/Write]  I2C Slave 2 Data Out
                                //              I2C_SLV0_D2[bit 7:0]: Data out when slave 2 is set to write
#define I2C_SLV3_DO       0x66  //[Read/Write]  I2C Slave 3 Data Out
                                //              I2C_SLV0_D3[bit 7:0]: Data out when slave 3 is set to write
#define I2C_MST_DELAY_CTRL 0x67 //[Read/Write]  I2C Master Delay Control
                                //              DELAY_ES_SHADOW[bit 7]: Delays shadowing of external sensor data until all data is received
                                //              Reserved[bit 6:5]
                                //              I2C_SLV4_DLY_EN[bit 4]: When enabled, slave 4 will only be accessed (1+I2C_MST_DLY)
                                //                  samples as determined by SMPLRT_DIV and DLPF_CFG
                                //              I2C_SLV3_DLY_EN[bit 3]: When enabled, slave 3 will only be accessed (1+I2C_MST_DLY)
                                //                  samples as determined by SMPLRT_DIV and DLPF_CFG
                                //              I2C_SLV2_DLY_EN[bit 2]: When enabled, slave 2 will only be accessed 1+I2C_MST_DLY)
                                //                  samples as determined by SMPLRT_DIV and DLPF_CFG
                                //              I2C_SLV1_DLY_EN[bit 1]: When enabled, slave 1 will only be accessed 1+I2C_MST_DLY)
                                //                  samples as determined by SMPLRT_DIV and DLPF_CFG
                                //              I2C_SLV0_DLY_EN[bit 0]: When enabled, slave 0 will only be accessed 1+I2C_MST_DLY)
                                //                  samples as determined by SMPLRT_DIV and DLPF_CFG
#define SIGNAL_PATH_RESET 0x68  //[Read/Write]  Signal Path Reset
                                //              Reserved[bit 7:3]
                                //              GYRO_RST[bit 2]: Reset gyro digital signal path. Note: Sensor registers are not cleared.
                                //                  Use SIG_COND_RST to clear sensor registers.
                                //              ACCEL_RST[bit 1]: Reset accel digital signal path. Note: Sensor registers are not cleared.
                                //                  Use SIG_COND_RST to clear sensor registers.
                                //              TEMP_RST[bit 0]: Reset temp digital signal path. Note: Sensor registers are not cleared.
                                //                  Use SIG_COND_RST to clear sensor registers.
#define ACCEL_INTEL_CTRL  0x69  //[Read/Write]  Accelerometer Interrupt Control
                                //              ACCEL_INTEL_EN[bit 7]: This bit enables the Wake-on-Motion detection logic.
                                //              ACCEL_INTEL_MODE[bit 6]: This bit defines
                                //                      1 - Compare the current sample with the previous sample.
                                //                      0 - Not used.
                                //              Reserved[bit 5:0]
#define USER_CTRL         0x6A  //[Read/Write]  User Control (@Bit 7 enable DMP, bit 3 reset DMP)
                                //              Reserved[bit 7]
                                //              FIFO_EN[bit 6]:
                                //                      1 - Enable FIFO operation mode.
                                //                      0 - Disable FIFO access from serial interface. To disable FIFO writes by dma, use FIFO_EN
                                //                          register. To disable possible FIFO writes from DMP, disable the DMP.
                                //              I2C_MST_EN[bit 5]:
                                //                      1 - Enable the I2C Master I/F module; pins ES_DA and ES_SCL are isolated from pins SDA/SDI and SCL/ SCLK.
                                //                      0 - Disable I2C Master I/F module; pins ES_DA and ES_SCL are logically driven by pins SDA/SDI and SCL/ SCLK.
                                //              I2C_IF_DIS[bit 4]:
                                //                      1 - Reset I2C Slave module and put the serial interface in SPI mode only. This bit auto clears
                                //                          after one clock cycle.
                                //              Reserved[bit 3]
                                //              FIFO_RST[bit 2]:
                                //                      1 - Reset FIFO module. Reset is asynchronous. This bit auto clears after one clock cycle.
                                //              I2C_MST_RST[bit 1]:
                                //                      1 - Reset I2C Master module. Reset is asynchronous. This bit auto clears after one clock cycle.
                                //              SIG_COND_RST[bit 0]:
                                //                      1 - Reset all gyro digital signal path, accel digital signal path, and temp digital signal path.
                                //                          This bit also clears all the sensor registers. SIG_COND_RST is a pulse of one clk8M wide.
#define PWR_MGMT_1        0x6B  //[Read/Write]  Power Management 1 (Device defaults in the SLEEP mode)
                                //              Reset value: (Depends on PU_SLEEP_MODE bit, see below)
                                //              H_RESET[bit 7]:
                                //                      1 - Reset the internal registers and restores the default settings. Write a 1 to set the
                                //                          reset, the bit will auto clear.
                                //              SLEEP[bit 6]: When set, the chip is set to sleep mode (After OTP loads, the PU_SLEEP_MODE bit will be written here)
                                //              CYCLE[bit 5]: When set, and SLEEP and STANDBY are not set, the chip will cycle between sleep and taking
                                //                          a single sample at a rate determined by LP_ACCEL_ODR register
                                //              GYRO_STANDBY[bit 4]: When set, the gyro drive and pll circuitry are enabled, but the sense paths are
                                //                          disabled. This is a low power mode that allows quick enabling of the gyros.
                                //              PD_PTAT[bit 3]: Power down internal PTAT voltage generator and PTAT ADC
                                //              CLKSEL[bit 2:0]: Select Clock Source/PLL
                                //                      000 - Internal 20MHz oscillator
                                //                      001 - Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
                                //                      010 - Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
                                //                      011 - Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
                                //                      100 - Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
                                //                      101 - Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
                                //                      110 - Internal 20MHz oscillator
                                //                      111 - Stops the clock and keeps timing generator in reset
                                //                  (After OTP loads, the inverse of PU_SLEEP_MODE bit will be written to CLKSEL[0])
#define PWR_MGMT_2        0x6C  //[Read/Write]  Power Management 2
                                //              Reserved[bit 7:6]
                                //              DISABLE_XA[bit 5]:
                                //                      1 - X accelerometer is disabled
                                //                      0 - X accelerometer is on
                                //              DISABLE_YA[bit 4]:
                                //                      1 - Y accelerometer is disabled
                                //                      0 - Y accelerometer is on
                                //              DISABLE_ZA[bit 3]:
                                //                      1 - Y accelerometer is disabled
                                //                      0 - Y accelerometer is on
                                //              DISABLE_XG[bit 2]:
                                //                      1 - X gyro is disabled
                                //                      0 - X gyro is on
                                //              DISABLE_YG[bit 1]:
                                //                      1 - Y gyro is disabled
                                //                      0 - Y gyro is on
                                //              DISABLE_ZG[bit 0]:
                                //                      1 - Z gyro is disabled
                                //                      0 - Z gyro is on
#define DMP_BANK          0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT        0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG           0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1         0x70
#define DMP_REG_2         0x71
#define FIFO_COUNTH       0x72  //[Read Only]   FIFO Count Register
                                //              Reserved[bit 7:5]
                                //              FIFO_CNT[bit 4:0]: High Bits, count indicates the number of written bytes in the FIFO.
                                //                  Reading this byte latches the data for both FIFO_COUNTH, and FIFO_COUN
#define FIFO_COUNTL       0x73  //[Read Only]   FIFO Count Register
                                //              FIFO_CNT[bit 7:0]: Low Bits, count indicates the number of written bytes in the FIFO.
                                //                  NOTE: Must read FIFO_COUNTH to latch new data for both FIFO_COUNTH and FIFO_COUNTL.
#define FIFO_R_W          0x74  //[Read/Write]  FIFO Read Write
                                //              D[bit 7:0]: Read/Write command provides Read or Write operation for the FIFO.
#define WHO_AM_I_MPU9250  0x75  //[Read Only]   Who Am I Register (Should return 0x71)
                                //              WHOAMI[bit 7:0]: Register to indicate to user which device is being accessed
#define XA_OFFSET_H       0x77  //[Read/Write]  Accelerometer Offset Register
                                //              XA_OFFS[bit 7:0]: Upper bits of the X accelerometer offset cancellation. +/- 16g
                                //                  Offset cancellation in all Full Scale modes, 15 bit 0.98-mg steps
#define XA_OFFSET_L       0x78  //[Read/Write]  Accelerometer Offset Register
                                //              XA_OFFS[bit 7:1]: Lower bits of the X accelerometer offset cancellation. +/- 16g
                                //                  Offset cancellation in all Full Scale modes, 15 bit 0.98-mg steps
#define YA_OFFSET_H       0x7A  //[Read/Write]  Accelerometer Offset Register
                                //              YA_OFFS[bit 7:0]: Upper bits of the Y accelerometer offset cancellation. +/- 16g
                                //                  Offset cancellation in all Full Scale modes, 15 bit 0.98-mg steps
#define YA_OFFSET_L       0x7B  //[Read/Write]  Accelerometer Offset Register
                                //              YA_OFFS[bit 7:1]: Lower bits of the Y accelerometer offset cancellation. +/- 16g
                                //                  Offset cancellation in all Full Scale modes, 15 bit 0.98-mg steps
#define ZA_OFFSET_H       0x7D  //[Read/Write]  Accelerometer Offset Register
                                //              ZA_OFFS[bit 7:0]: Upper bits of the Z accelerometer offset cancellation. +/- 16g
                                //                  Offset cancellation in all Full Scale modes, 15 bit 0.98-mg steps
#define ZA_OFFSET_L       0x7E  //[Read/Write]  Accelerometer Offset Register
                                //              ZA_OFFS[bit 7:1]: Lower bits of the Z accelerometer offset cancellation. +/- 16g
                                //                  Offset cancellation in all Full Scale modes, 15 bit 0.98-mg steps

// Using the MPU-9250 breakout board, ADO is set to 0
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO 0
#if ADO
#define MPU9250_ADDRESS 0x69  // Device address when ADO = 1
#else
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
#endif // AD0

#define READ_FLAG 0x80

#define ACCEL_FULL_SCALE 16384 //16384 LSB/g in full scale +/- 16g mode

class Geegrow_MPU9250 {

  public:
    // Gyro bandwidth (Hz)
    enum GBandwidth {
        GBW_8800HZ  = -0x01,
        GBW_250HZ   = 0x00,
        GBW_184HZ   = 0x01,
        GBW_92HZ    = 0x02,
        GBW_41HZ    = 0x03,
        GBW_20HZ    = 0x04,
        GBW_10HZ    = 0x05,
        GBW_5HZ	    = 0x06,
        GBW_3600HZ  = 0x07,
    };

    // Gyro sample rate (Hz)
    enum GFsample {
        GFS_1000HZ  = 0,
        GFS_8000HZ,
        GFS_32000HZ
    };

    // Gyro scale
    enum GScale {
        GFS_250DPS	= 0,
        GFS_500DPS,
        GFS_1000DPS,
        GFS_2000DPS
    };

    // Accel bandwidth (Hz)
    enum ABandwidth {
        ABW_1130HZ  = -0x01,
        ABW_460HZ   = 0x00,
        ABW_184HZ   = 0x01,
        ABW_92HZ    = 0x02,
        ABW_41HZ    = 0x03,
        ABW_20HZ    = 0x04,
        ABW_10HZ    = 0x05,
        ABW_5HZ	    = 0x06,
        // Seems this one is just copy of 460HZ (0x00), so we will not use it at all !!!
        //ABW_460HZ_2	= 0x07,
    };

    // Accel sample rate (Hz)
    enum AFsample {
        AFS_1000HZ = 0,
        AFS_4000HZ,
    };

    // Accel scale
    enum AScale {
        AFS_2G = 0,
        AFS_4G,
        AFS_8G,
        AFS_16G
    };

    GeegrowAK8963 compass;

    int16_t accelRawData[3] = {0, 0, 0};
    int16_t gyroRawData[3] = {0, 0, 0};

    // float temperature;   // Stores the real internal chip temperature in Celsius
    // int16_t tempCount;   // Temperature raw count output
    // uint32_t delt_t = 0; // Used to control display output rate

    uint32_t count = 0, sumCount = 0; // used to control display output rate
    float deltat = 0.0f, sum = 0.0f;  // integration interval for both filter schemes
    uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
    uint32_t Now = 0;        // used to calculate integration interval

    uint16_t aScaleFactor;
    float gScaleFactor, mScaleFactor;

    // Variables for storing latest sensor data values
    float ax, ay, az, gx, gy, gz, mx, my, mz;

    float selfTest[6];

    // Public method declarations
    Geegrow_MPU9250();

    void setGyroBandwidth(GBandwidth gbw) {
        gyroBandwidth = gbw;
    }

    void setGyroSampleRate(GFsample gfs) {
        gyroSampleRate = gfs;
    }

    void setGyroScale(GScale gs) {
        gyroScale = gs;
    }

    void setAccelBandwidth(ABandwidth abw) {
        accelBandwidth = abw;
    }

    void setAccelSampleRate(AFsample afs) {
        accelSampleRate = afs;
    }

    void setAccelScale(AScale as) {
        accelScale = as;
    }

    void setSampleRateDivider(uint8_t);

    void setGScaleFactor();
    void setAScaleFactor();

    void updateAccel();
    void updateGyro();
    void updateCompass();
    void update();
    int16_t readTempData();
    void updateTime();
    /* Initialization */
    void initAK8963();
    void initMPU9250();
    /* Calibration */
    void calibrate();
    void calibrateCompass();
    void MPU9250SelfTest(float * destination);

    /* I2C_PREVENT_FREEZING is defined in AK8963.h */

    uint8_t writeBits(uint8_t registerAddress, uint8_t data) {
        return I2CTransport::writeBits(deviceAddr, registerAddress, data, I2C_PREVENT_FREEZING);
    }

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

    // Physical I2C address of MPU9250
    uint8_t deviceAddr = MPU9250_ADDRESS;

    // Gyro settings
    GBandwidth gyroBandwidth = GBW_41HZ;
    GFsample gyroSampleRate = GFS_1000HZ;
    GScale gyroScale = GFS_250DPS;

    // Acccel settings
    ABandwidth accelBandwidth = ABW_41HZ;
    AFsample accelSampleRate = AFS_1000HZ;
    AScale accelScale = AFS_2G;

    uint8_t sampleRateDivider = 0x04;

    void getGyroFilterConfiguration(uint8_t &fchoice, uint8_t &dlpf);
    void getAccelFilterConfiguration(uint8_t &fchoice, uint8_t &dlpf);
    void configureGyro();
    void configureAccel();

    void readAccelBiasRegister(int32_t * bias);
    void writeAccelBiasRegister(const long *bias);
    void writeGyroBiasRegister(const long *bias);

    void readAccelRawData(int16_t * data);
    void readGyroRawData(int16_t * data);

    /*
     * Reset the internal registers and restores the default settings.
     */
    void deviceReset() {
        writeBits(PWR_MGMT_1, 0x08);
    }

    /*
     * Auto selects the best available clock source – PLL if ready,
     * else use the Internal oscillator
     */
    void deviceSelectAutoPll() {
        writeBits(PWR_MGMT_1, 0x01);
    }
};  // class MPU9250

#endif // _MPU9250_H_
