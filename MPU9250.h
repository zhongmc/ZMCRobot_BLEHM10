#pragma once
#ifndef MPU9250_H
#define MPU9250_H

#include "MPU.h"



//Magnetometer Registers
// #define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L	 0x03  // data
#define AK8963_XOUT_H	 0x04
#define AK8963_YOUT_L	 0x05
#define AK8963_YOUT_H	 0x06
#define AK8963_ZOUT_L	 0x07
#define AK8963_ZOUT_H	 0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

// #define X_FINE_GAIN      0x03 // [7:0] fine gain
// #define Y_FINE_GAIN      0x04
// #define Z_FINE_GAIN      0x05
// #define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
// #define XA_OFFSET_L_TC   0x07
// #define YA_OFFSET_H      0x08
// #define YA_OFFSET_L_TC   0x09
// #define ZA_OFFSET_H      0x0A
// #define ZA_OFFSET_L_TC   0x0B

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define MPU_CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E


class MPU9250 : public MPU {


    const uint8_t MPU9250_ADDRESS {0x68};  // Device address when ADO = 0
    const uint8_t AK8963_ADDRESS {0x0C};   //  Address of magnetometer

    const uint8_t MPU9250_WHOAMI_DEFAULT_VALUE {0x71}; // 0x68????
    const uint8_t AK8963_WHOAMI_DEFAULT_VALUE {0x48};
    const uint8_t Mmode {0x06};        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read

public:
    MPU9250():MPU()
    {
    }

public:
    virtual bool init()
    {
        wire = &Wire;

        uint8_t m_whoami = 0x00;
        uint8_t a_whoami = 0x00;

        Serial.println("setup MPU9250...");
        m_whoami = isConnectedMPU9250();
        if (m_whoami)
        {
            Serial.println("MPU9250 is online...");
            initMPU9250();

            a_whoami = isConnectedAK8963();
            if (a_whoami)
            {
                initAK8963(magCalibration);
            }
            else
            {
                Serial.print("Could not connect to AK8963: 0x");
                Serial.println(a_whoami);
                return false;
            }
        }
        else
        {
            Serial.print("Could not connect to MPU9250: 0x");
            Serial.println(m_whoami);
            return false;
        }

        return true;
    }

    virtual void doCalibrate()
    {
        calibrateMPU9250(gyroBias, accelBias);
        // magcalMPU9250(magBias, magScale);

        delay(100);
        initMPU9250();
        delay(1000);

        update();

    }
    
    virtual void update()
    {
        if (available())
        {  // On interrupt, check if data ready interrupt
            updateAccelGyro();
            updateMag(); // TODO: set to 30fps?
        }

    }


    private:

    void initMPU9250()
    {
        // wake up device
        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
        delay(100); // Wait for all registers to reset

        // get stable time source
        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
        delay(200);

        // Configure Gyro and Thermometer
        // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
        // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
        // be higher than 1 / 0.0059 = 170 Hz
        // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
        // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
        writeByte(MPU9250_ADDRESS, MPU_CONFIG, 0x03);

        // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        // writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                            // determined inset in CONFIG above

        writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x07);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                            // determined inset in CONFIG above
        // Set gyroscope full scale range
        // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
        uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
        // c = c & ~0xE0; // Clear self-test bits [7:5]
        c = c & ~0x03; // Clear Fchoice bits [1:0]
        c = c & ~0x18; // Clear GFS bits [4:3]
        c = c | (uint8_t)GFSSEL << 3; // Set full scale range for the gyro
        // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
        writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

        // Set accelerometer full-scale range configuration
        c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
        // c = c & ~0xE0; // Clear self-test bits [7:5]
        c = c & ~0x18;  // Clear AFS bits [4:3]
        c = c | (uint8_t)AFSSEL << 3; // Set full scale range for the accelerometer
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

        // Set accelerometer sample rate configuration
        // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
        // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
        c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
        c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
        c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

        // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
        // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

        // Configure Interrupts and Bypass Enable
        // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
        // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
        // can join the I2C bus and all can be controlled by the Arduino as master
        writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
        writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
        delay(100);
    }

    void initAK8963(float * destination)
    {
        // First extract the factory calibration for each magnetometer axis
        uint8_t rawData[3];  // x/y/z gyro calibration data stored here
        writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
        delay(10);
        writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
        delay(10);
        readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
        destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
        destination[1] =  (float)(rawData[1] - 128)/256. + 1.;
        destination[2] =  (float)(rawData[2] - 128)/256. + 1.;
        writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
        delay(10);
        // Configure the magnetometer for continuous read and highest resolution
        // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
        // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
        writeByte(AK8963_ADDRESS, AK8963_CNTL, (uint8_t)MFSSEL << 4 | Mmode); // Set magnetometer data resolution and sample ODR
        delay(10);
/*
        Serial.println("Calibration values: ");
        Serial.print("X-Axis sensitivity adjustment value "); Serial.println(destination[0], 4);
        Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(destination[1], 4);
        Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(destination[2], 4);
        Serial.print("X-Axis sensitivity offset value "); Serial.println(magBias[0], 2);
        Serial.print("Y-Axis sensitivity offset value "); Serial.println(magBias[1], 2);
        Serial.print("Z-Axis sensitivity offset value "); Serial.println(magBias[2], 2);
  */  }

    bool isConnectedMPU9250()
    {
        byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
        Serial.print("MPU9250 WHO AM I = ");
        Serial.println(c, HEX);
        return (c == MPU9250_WHOAMI_DEFAULT_VALUE);
    }

    bool isConnectedAK8963()
    {
        byte c = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);
        Serial.print("AK8963  WHO AM I = ");
        Serial.println(c, HEX);
        return (c == AK8963_WHOAMI_DEFAULT_VALUE);
    }

    bool available()
    {
        return (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01);
    }



    void updateAccelGyro()
    {
        int16_t MPU9250Data[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
        readMPU9250Data(MPU9250Data); // INT cleared on any read

        // Now we'll calculate the accleration value into actual g's
        a[0] = (float)MPU9250Data[0] * aRes - accelBias[0];  // get actual g value, this depends on scale being set
        a[1] = (float)MPU9250Data[1] * aRes - accelBias[1];
        a[2] = (float)MPU9250Data[2] * aRes - accelBias[2];

        // Calculate the gyro value into actual degrees per second
        g[0] = (float)MPU9250Data[4] * gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
        g[1] = (float)MPU9250Data[5] * gRes - gyroBias[1];
        g[2] = (float)MPU9250Data[6] * gRes - gyroBias[2];

        // raw_data[0] = MPU9250Data[0];
        // raw_data[1] = MPU9250Data[1];
        // raw_data[2] = MPU9250Data[2];

        // raw_data[3] = MPU9250Data[4];
        // raw_data[4] = MPU9250Data[5];
        // raw_data[5] = MPU9250Data[6];


    }

    void readMPU9250Data(int16_t * destination)
    {
        uint8_t rawData[14];  // x/y/z accel register data stored here
        readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 14, &rawData[0]);  // Read the 14 raw data registers into data array
        destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
        destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
        destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
        destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
        destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
        destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;

        for( int i=0; i<6; i++)
        {
            rawdata[i] = rawData[i];
            rawdata[6+i] = rawData[8+i];
        }
    }

    void updateMag()
    {
        int16_t magCount[3] = {0, 0, 0};    // Stores the 16-bit signed magnetometer sensor output

        newMagData = false;
        readMagData(magCount);  // Read the x/y/z adc values
        // getMres();
        if( newMagData )
        {
            newMagData = false;
        // Calculate the magnetometer values in milliGauss
            // Include factory calibration per data sheet and user environmental corrections
            m[0] = (float)(magCount[0] * mRes * magCalibration[0] - magBias[0]) * magScale[0];  // get actual magnetometer value, this depends on scale being set
            m[1] = (float)(magCount[1] * mRes * magCalibration[1] - magBias[1]) * magScale[1];
            m[2] = (float)(magCount[2] * mRes * magCalibration[2] - magBias[2]) * magScale[2];

            // raw_data[6] = magCount[0];
            // raw_data[7] = magCount[1];
            // raw_data[8] = magCount[2];
        }
    }

    // void readMagData(int16_t * destination)
    // {
    //     uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
    //     if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
    //         readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
    //         uint8_t c = rawData[6]; // End data read by reading ST2 register
    //         if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
    //             destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
    //             destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];  // Data stored as little Endian
    //             destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
    //         }
    //     }
    // }


    void readMagData(int16_t * destination)
    {
        uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
        newMagData = (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01);
        if(newMagData == true) { // wait for magnetometer data ready bit to be set
            readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
            uint8_t c = rawData[6]; // End data read by reading ST2 register
                if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
                    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
                    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
                    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 

                    for( int i=0; i<6; i++)
                        rawdata[12+i] = rawData[i];
            }
            else
            {
                newMagData = false;
            }
            
        }
    }    


    // Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
    // of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
    void calibrateMPU9250(float * dest1, float * dest2)
    {
        uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
        uint16_t ii, packet_count, fifo_count;
        int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

        // reset device
        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
        delay(100);

        // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
        // else use the internal oscillator, bits 2:0 = 001
        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
        writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
        delay(200);

        // Configure device for bias calculation
        writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
        writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
        writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
        writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
        writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
        delay(15);

        // Configure MPU6050 gyro and accelerometer for bias calculation
        writeByte(MPU9250_ADDRESS, MPU_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
        writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
        writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

        uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
        uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

        // Configure FIFO to capture accelerometer and gyro data for bias calculation
        writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
        writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
        delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

        // At end of sample accumulation, turn off FIFO sensor read
        writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
        readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
        fifo_count = ((uint16_t)data[0] << 8) | data[1];
        packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging

        for (ii = 0; ii < packet_count; ii++)
        {
            int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
            readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
            accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
            accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
            accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
            gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
            gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
            gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

            accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
            accel_bias[1] += (int32_t) accel_temp[1];
            accel_bias[2] += (int32_t) accel_temp[2];
            gyro_bias[0]  += (int32_t) gyro_temp[0];
            gyro_bias[1]  += (int32_t) gyro_temp[1];
            gyro_bias[2]  += (int32_t) gyro_temp[2];
        }
        accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
        accel_bias[1] /= (int32_t) packet_count;
        accel_bias[2] /= (int32_t) packet_count;
        gyro_bias[0]  /= (int32_t) packet_count;
        gyro_bias[1]  /= (int32_t) packet_count;
        gyro_bias[2]  /= (int32_t) packet_count;

        if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
        else {accel_bias[2] += (int32_t) accelsensitivity;}

        // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
        data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
        data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
        data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
        data[3] = (-gyro_bias[1] / 4)       & 0xFF;
        data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
        data[5] = (-gyro_bias[2] / 4)       & 0xFF;

        // Push gyro biases to hardware registers
        writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
        writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
        writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
        writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
        writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
        writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

        // Output scaled gyro biases for display in the main program
        dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
        dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
        dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

        // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
        // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
        // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
        // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
        // the accelerometer biases calculated above must be divided by 8.

        // int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
        // readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
        // accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
        // readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
        // accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
        // readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
        // accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

        // uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
        // uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

        // for(ii = 0; ii < 3; ii++) {
        //     if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
        // }

        // // Construct total accelerometer bias, including calculated average accelerometer bias from above
        // accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
        // accel_bias_reg[1] -= (accel_bias[1] / 8);
        // accel_bias_reg[2] -= (accel_bias[2] / 8);

        // data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
        // data[1] = (accel_bias_reg[0])      & 0xFF;
        // data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
        // data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
        // data[3] = (accel_bias_reg[1])      & 0xFF;
        // data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
        // data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
        // data[5] = (accel_bias_reg[2])      & 0xFF;
        // data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

        // Apparently this is not working for the acceleration biases in the MPU-9250
        // Are we handling the temperature correction bit properly?
        // Push accelerometer biases to hardware registers
        // writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
        // writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
        // writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
        // writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
        // writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
        // writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

        // Output scaled accelerometer biases for display in the main program
        dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
        dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
        dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;

        Serial.println("- MPU9250 bias");
        Serial.println("- accel Bias*1000");
        Serial.print((int)(1000 * accelBias[0])); Serial.print(" ");
        Serial.print((int)(1000 * accelBias[1])); Serial.print(" ");
        Serial.print((int)(1000 * accelBias[2])); Serial.print(" ");
        Serial.println("mg");
        Serial.println("- gyro Bias:");
        Serial.print(gyroBias[0], 1); Serial.print(" ");
        Serial.print(gyroBias[1], 1); Serial.print(" ");
        Serial.print(gyroBias[2], 1); Serial.print(" ");
        Serial.println("- o/s");

     }

    void magcalMPU9250(float * dest1, float * dest2)
    {
        uint16_t ii = 0, sample_count = 0;
        int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
        int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

        Serial.println("Mag Calibration: Wave device in a figure eight until done!");
        delay(4000);
        Serial.println("Go...");

        // shoot for ~fifteen seconds of mag data
        if      (Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
        else if (Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms

        for(ii = 0; ii < sample_count; ii++)
        {
            readMagData(mag_temp);  // Read the mag data
            for (int jj = 0; jj < 3; jj++)
            {
                if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
                if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
            }
            if(Mmode == 0x02) delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
            if(Mmode == 0x06) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
        }

        Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
        Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
        Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

        // Get hard iron correction
        mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
        mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
        mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

        dest1[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
        dest1[1] = (float) mag_bias[1]*mRes*magCalibration[1];
        dest1[2] = (float) mag_bias[2]*mRes*magCalibration[2];

        // Get soft iron correction estimate
        mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
        mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
        mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

        float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
        avg_rad /= 3.0;

        dest2[0] = avg_rad/((float)mag_scale[0]);
        dest2[1] = avg_rad/((float)mag_scale[1]);
        dest2[2] = avg_rad/((float)mag_scale[2]);

        Serial.println("Mag Calibration done!");

        Serial.println("AK8963 mag biases (mG)");
        Serial.print(magBias[0]); Serial.print(", ");
        Serial.print(magBias[1]); Serial.print(", ");
        Serial.print(magBias[2]); Serial.println();
        Serial.println("AK8963 mag scale (mG)");
        Serial.print(magScale[0]); Serial.print(", ");
        Serial.print(magScale[1]); Serial.print(", ");
        Serial.print(magScale[2]); Serial.println();
    }

};


#endif // MPU9250_H
