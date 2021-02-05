/******************************************************************************

 @file  simple_peripheral.c

 @brief This file contains the Simple BLE Peripheral sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <xdc/runtime/System.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>

#include "hci_tl.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"
#include "peripheral.h"
#define __BSD_VISIBLE
#include "math.h"
#include "myi2c.h"
#include "mpu9250.h"


#include "imu_reader.h"

/*********************************************************************
 * CONSTANTS
 */
#define Kp                          2.0f         // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki                          0.005f       // integral gain governs rate of convergence of gyroscope biases
#define PI                          M_PI
uint32_t last, now;
float deltat;

#define DELAY_MS(i)      (Task_sleep(((i) * 1000) / Clock_tickPeriod))
#define DELAY_US(i)      (Task_sleep(((i) * 1) / Clock_tickPeriod))
/*********************************************************************
 * TYPEDEFS
 */
#define MPU9250_Number              6

typedef struct MPU9250
{
    uint8_t id;
    uint8_t visable;
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    float quaternion[4];
    float yaw, pitch, roll;
    float exInt, eyInt, ezInt;
    uint32_t last, now;
    float deltat;
    uint8_t data[IMUREADER_CHAR_LEN];
    uint8_t mag_data[IMUREADER_CHAR_LEN];
} MPU9250;


/*********************************************************************
 * GLOBAL VARIABLES
 */


/*********************************************************************
 * LOCAL VARIABLES
 */
struct MPU9250 mpu9250[MPU9250_Number];

volatile bool res = false;
static uint8_t val = 0x00;
static uint8_t counter = 0x00;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
void Read_Data(uint8_t sel);
void Read_RawData(uint8_t *des, uint8_t index);
void Read_DataAndRawData(uint8_t sel, uint8_t *des);
void assignTestData(uint8_t count, uint8_t *des);
void Get_DeltaT(uint8_t sel);
void AHRSupdate(int index, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void QtoEular(int index, float q0, float q1, float q2, float q3);
/*********************************************************************
 * EXTERN FUNCTIONS
 */
/*********************************************************************
 * PUBLIC FUNCTIONS
 */


/*********************************************************************
 * @fn      SimpleBLEPeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
void IMUReader_init(void)
{
    uint8_t i;
    //float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};

    System_printf("IMUReader init...\n");
    System_flush();

    if (SensorI2C_open()){
        System_printf("I2C...OK!\n");
    }else{
        System_printf("I2C...Fail!\n");
    }

    for (i = 0; i < MPU9250_Number; i++)
    {
        mpu9250[i].id = i;
        mpu9250[i].visable = 0;
        res = SensorI2C_Select(i);

        if (res)
        {
            res = SensorI2C_readReg(MPU9250_ADDRESS, WHO_AM_I, &val, 1);
            System_printf("[%d] : 0x%x, ", i, val);

            if (res && (val == 0x73 || val == 0x71))
            {
                //calibrateMPU9250(gyroBias, accelBias);
                //System_printf("gyroBias=%f, %f, %f \r\n", gyroBias[0], gyroBias[1], gyroBias[2]);
                //System_printf("accelBias=%f, %f, %f \r\n", accelBias[0], accelBias[1], accelBias[2]);
                initMPU9250();

                AK8963_read(MAG_WHO_AM_I, &val, 1);
                System_printf(", 0x%x ", val);

                if (val != 0x48){
                    System_printf("Init Failed\r\n");
                }
                else
                {
                    initAK8963();

                    mpu9250[i].visable = 1;
                    mpu9250[i].quaternion[0] = 1;
                    System_printf("OK!\r\n");
                }

            }
            else
            {
                System_printf("Init Failed!\r\n");
            }
        }
    }

    System_printf("DONE!\n");
    System_flush();
    DELAY_MS(100);
}


void IMU_Reader_processEvent_Loop(void)
{
    uint8_t i = 0;

    for(;;)
    {
        for (i = 0; i < MPU9250_Number; i++)
        {
            if (mpu9250[i].visable)
            {
                res = SensorI2C_Select(i);

                if (res){
                    Read_RawData(mpu9250[i].data, i);
                    Get_DeltaT(i);  //計算deltat，單位(s)

                    switch (i)
                    {
                        case 0:
                            SimpleProfile_SetParameter(IMUREADER_CHAR_HAND, IMUREADER_CHAR_LEN,
                                                       mpu9250[i].data);
                            break;
                        case 1:
                            SimpleProfile_SetParameter(IMUREADER_CHAR_THUMB, IMUREADER_CHAR_LEN,
                                                       mpu9250[i].data);
                            break;
                        case 2:
                            SimpleProfile_SetParameter(IMUREADER_CHAR_INDEX, IMUREADER_CHAR_LEN,
                                                       mpu9250[i].data);
                            break;
                        case 3:
                            SimpleProfile_SetParameter(IMUREADER_CHAR_MIDDLE, IMUREADER_CHAR_LEN,
                                                       mpu9250[i].data);
                            break;
                        case 4:
                            SimpleProfile_SetParameter(IMUREADER_CHAR_RING, IMUREADER_CHAR_LEN,
                                                       mpu9250[i].data);
                            break;
                        case 5:
                            SimpleProfile_SetParameter(IMUREADER_CHAR_LITTLE, IMUREADER_CHAR_LEN,
                                                       mpu9250[i].data);
                            break;
                    }
                }
            }
        }

        DELAY_MS(20);
    }

}


void IMU_Reader_processEvent(void)
{
    uint8_t i = 0;
    /*
     * this part for test.
     * transport 0-255 value.
     */
/*
    //uint16 notify_Handle;
    uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 65, 66, 67, 68, 69 };

    uint8_t imuReaderValue1[IMUREADER_CHAR_LEN];
    uint8_t imuReaderValue2[IMUREADER_CHAR_LEN];
    uint8_t imuReaderValue3[IMUREADER_CHAR_LEN];
    uint8_t imuReaderValue4[IMUREADER_CHAR_LEN];
    uint8_t imuReaderValue5[IMUREADER_CHAR_LEN];
    uint8_t imuReaderValue6[IMUREADER_CHAR_LEN];

    for (i = 0; i < IMUREADER_CHAR_LEN; i++)
    {
        imuReaderValue1[i] = count + i;
        imuReaderValue2[i] = count + i + 1;
        imuReaderValue3[i] = count + i + 2;
        imuReaderValue4[i] = count + i + 3;
        imuReaderValue5[i] = count + i + 4;
        imuReaderValue6[i] = count + i + 5;
    }

    count ++;
    Read_Data(0);

    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                                   &count);

    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
                                   charValue5);

    SimpleProfile_SetParameter(IMUREADER_CHAR_HAND, IMUREADER_CHAR_LEN,
                               imuReaderValue1);
    SimpleProfile_SetParameter(IMUREADER_CHAR_THUMB, IMUREADER_CHAR_LEN,
                               imuReaderValue2);
    SimpleProfile_SetParameter(IMUREADER_CHAR_INDEX, IMUREADER_CHAR_LEN,
                               imuReaderValue3);
    SimpleProfile_SetParameter(IMUREADER_CHAR_MIDDLE, IMUREADER_CHAR_LEN,
                               imuReaderValue4);
    SimpleProfile_SetParameter(IMUREADER_CHAR_RING, IMUREADER_CHAR_LEN,
                               imuReaderValue5);
    SimpleProfile_SetParameter(IMUREADER_CHAR_LITTLE, IMUREADER_CHAR_LEN,
                               imuReaderValue6);

*/
    /*
    count ++;
    Read_RawData(mpu9250[0].data);

    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                                   &count);

    SimpleProfile_SetParameter(IMUREADER_CHAR_HAND, IMUREADER_CHAR_LEN,
                               mpu9250[0].data);
    */

    for (i = 0; i < MPU9250_Number; i++)
    {

        res = SensorI2C_Select(i);

        if (res){
            Read_RawData(mpu9250[i].data, i);
            Get_DeltaT(i);

            switch (i)
            {
                case 0:
                    SimpleProfile_SetParameter(IMUREADER_CHAR_HAND, IMUREADER_CHAR_LEN,
                                               mpu9250[i].data);
                    break;
                case 1:
                    SimpleProfile_SetParameter(IMUREADER_CHAR_THUMB, IMUREADER_CHAR_LEN,
                                               mpu9250[i].data);
                    break;
                case 2:
                    SimpleProfile_SetParameter(IMUREADER_CHAR_INDEX, IMUREADER_CHAR_LEN,
                                               mpu9250[i].data);
                    break;
                case 3:
                    SimpleProfile_SetParameter(IMUREADER_CHAR_MIDDLE, IMUREADER_CHAR_LEN,
                                               mpu9250[i].data);
                    break;
                case 4:
                    SimpleProfile_SetParameter(IMUREADER_CHAR_RING, IMUREADER_CHAR_LEN,
                                               mpu9250[i].data);
                    break;
                case 5:
                    SimpleProfile_SetParameter(IMUREADER_CHAR_LITTLE, IMUREADER_CHAR_LEN,
                                               mpu9250[i].data);
                    break;
            }
        }


    }
#ifdef HELLO
    for (i = 0; i < MPU9250_Number; i++)
    {
        if (mpu9250[i].visable)
        {
            res = SensorI2C_Select(i);

            if (res){
                Read_RawData(mpu9250[i].data, i);

                //Read_Data(i);
                //Read_DataAndRawData(i, mpu9250[i].data);
                //計算deltat，單位(s)
                Get_DeltaT(i);

                //AHRSupdate(i, mpu9250[i].ax, mpu9250[i].ay, mpu9250[i].az, mpu9250[i].gx * PI / 180.0, mpu9250[i].gy * PI / 180.0, mpu9250[i].gz * PI / 180.0, mpu9250[i].mx, mpu9250[i].my, mpu9250[i].mz);
                //QtoEular(i, mpu9250[i].quaternion[0], mpu9250[i].quaternion[1], mpu9250[i].quaternion[2], mpu9250[i].quaternion[3]);
                //System_printf("%.1f, %.1f, %.1f\n", mpu9250[i].mx, mpu9250[i].my, mpu9250[i].mz);
                /*
                System_printf("mpu[%d]:", i);
                System_printf("%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f\t", mpu9250[i].ax, mpu9250[i].ay, mpu9250[i].az, mpu9250[i].gx, mpu9250[i].gy, mpu9250[i].gz, mpu9250[i].mx, mpu9250[i].my, mpu9250[i].mz);
                //System_printf("%.2f, %.2f, %.2f, %.2f", mpu9250[i].quaternion[0], mpu9250[i].quaternion[1], mpu9250[i].quaternion[2], mpu9250[i].quaternion[3]);
                System_printf("\t(%.2f, %.2f, %.2f)-%.3f\n", mpu9250[i].yaw, mpu9250[i].pitch, mpu9250[i].roll, mpu9250[i].deltat);
                System_flush();
                */
            }

            switch (i)
            {
                case 0:
                    //assignTestData(count++, mpu9250[0].data);
                    SimpleProfile_SetParameter(IMUREADER_CHAR_HAND, IMUREADER_CHAR_LEN,
                                               mpu9250[i].data);
                    /*
                    SimpleProfile_SetParameter(IMUREADER_CHAR_THUMB, IMUREADER_CHAR_LEN,
                                                                   mpu9250[i].data);
                    SimpleProfile_SetParameter(IMUREADER_CHAR_INDEX, IMUREADER_CHAR_LEN,
                                                                   mpu9250[i].data);
                    SimpleProfile_SetParameter(IMUREADER_CHAR_MIDDLE, IMUREADER_CHAR_LEN,
                                                                   mpu9250[i].data);
                    SimpleProfile_SetParameter(IMUREADER_CHAR_RING, IMUREADER_CHAR_LEN,
                                                                   mpu9250[i].data);
                    SimpleProfile_SetParameter(IMUREADER_CHAR_LITTLE, IMUREADER_CHAR_LEN,
                                                                   mpu9250[i].data);
                    */
                    break;
                case 1:
                    SimpleProfile_SetParameter(IMUREADER_CHAR_THUMB, IMUREADER_CHAR_LEN,
                                               mpu9250[i].data);
                    break;
                case 2:
                    SimpleProfile_SetParameter(IMUREADER_CHAR_INDEX, IMUREADER_CHAR_LEN,
                                               mpu9250[i].data);
                    break;
                case 3:
                    SimpleProfile_SetParameter(IMUREADER_CHAR_MIDDLE, IMUREADER_CHAR_LEN,
                                               mpu9250[i].data);
                    break;
                case 4:
                    SimpleProfile_SetParameter(IMUREADER_CHAR_RING, IMUREADER_CHAR_LEN,
                                               mpu9250[i].data);
                    break;
                case 5:
                    SimpleProfile_SetParameter(IMUREADER_CHAR_LITTLE, IMUREADER_CHAR_LEN,
                                               mpu9250[i].data);
                    break;
            }
        }
    }
#endif
}

void IMU_Show_processEvent(void)
{
    uint8_t i;
    for (i = 0; i < MPU9250_Number; i++)
    {
        if (mpu9250[i].visable)
        {
            System_printf("mpu[%d]:", i);
            System_printf("%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f\t", mpu9250[i].ax, mpu9250[i].ay, mpu9250[i].az, mpu9250[i].gx, mpu9250[i].gy, mpu9250[i].gz, mpu9250[i].mx, mpu9250[i].my, mpu9250[i].mz);
            System_printf("%.2f, %.2f, %.2f, %.2f\t", mpu9250[i].quaternion[0], mpu9250[i].quaternion[1], mpu9250[i].quaternion[2], mpu9250[i].quaternion[3]);
            System_printf("(%.2f, %.2f, %.2f)-%.3f\n", mpu9250[i].yaw, mpu9250[i].pitch, mpu9250[i].roll, mpu9250[i].deltat);
            System_flush();
        }
    }
}

void Read_DataAndRawData(uint8_t sel, uint8_t *des)
{
    uint8_t acc_rawData[6];
    uint8_t gyro_rawData[6];
    uint8_t mag_rawData[8];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition

    //1. Read mag val
    do{
        AK8963_read(MAG_ST1, &mag_rawData[0], 8);
        //SensorI2C_readReg(MPU9250_ADDRESS, EXT_SENS_DATA_00, &mag_rawData[0], 8);
    }while((!(mag_rawData[0] & 0x01)) || (mag_rawData[7] & 0x08));

    /*
    SensorI2C_readReg(MPU9250_ADDRESS, EXT_SENS_DATA_00, &mag_rawData[0], 8);
    if(mag_rawData[0] & 0x01) { // wait for magnetometer data ready bit to be set
        if(!(mag_rawData[7] & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
            des[12] = mag_rawData[1];
            des[13] = mag_rawData[2];
            des[14] = mag_rawData[3];
            des[15] = mag_rawData[4];
            des[16] = mag_rawData[5];
            des[17] = mag_rawData[6];
        }
    }
    */

    //2. Read accel val
    SensorI2C_readReg(MPU9250_ADDRESS, ACCEL_XOUT_H, &acc_rawData[0], 6);  // Read the six raw data registers into data array
    SensorI2C_readReg(MPU9250_ADDRESS, GYRO_XOUT_H, &gyro_rawData[0], 6);  // Read the six raw data registers sequentially into data array
    des[0] = acc_rawData[1];
    des[1] = acc_rawData[0];
    des[2] = acc_rawData[3];
    des[3] = acc_rawData[2];
    des[4] = acc_rawData[5];
    des[5] = acc_rawData[4];

    //3. Read gyro val
    des[6] = gyro_rawData[1];
    des[7] = gyro_rawData[0];
    des[8] = gyro_rawData[3];
    des[9] = gyro_rawData[2];
    des[10] = gyro_rawData[5];
    des[11] = gyro_rawData[4];

    //4. Read mag val
    des[12] = mag_rawData[1];
    des[13] = mag_rawData[2];
    des[14] = mag_rawData[3];
    des[15] = mag_rawData[4];
    des[16] = mag_rawData[5];
    des[17] = mag_rawData[6];

    mpu9250[sel].mx = getMres(((int16_t)mag_rawData[4] << 8) | mag_rawData[3]);
    mpu9250[sel].my = getMres(((int16_t)mag_rawData[2] << 8) | mag_rawData[1]);
    mpu9250[sel].mz = -getMres(((int16_t)mag_rawData[6] << 8) | mag_rawData[5]);

    mpu9250[sel].ax = getAres(((int16_t)acc_rawData[0] << 8) | acc_rawData[1]);
    mpu9250[sel].ay = getAres(((int16_t)acc_rawData[2] << 8) | acc_rawData[3]);
    mpu9250[sel].az = getAres(((int16_t)acc_rawData[4] << 8) | acc_rawData[5]);

    mpu9250[sel].gx = getGres(((int16_t)gyro_rawData[0] << 8) | gyro_rawData[1]);
    mpu9250[sel].gy = getGres(((int16_t)gyro_rawData[2] << 8) | gyro_rawData[3]);
    mpu9250[sel].gz = getGres(((int16_t)gyro_rawData[4] << 8) | gyro_rawData[5]);

    /*
    switch(sel){
        case 0:

            //imu_00
            //mpu9250[sel].mx -= -50;
            //mpu9250[sel].my -= 200;
            //mpu9250[sel].mz -= 400;

            //imu_01
            mpu9250[sel].mx -= 373;
            mpu9250[sel].my -= 123;
            mpu9250[sel].mz -= 260;

            break;
    }
     */
}

void Read_Data(uint8_t sel)
{
    //Read 9-axis raw data's buffer.
    int16_t accelCount[3], gyroCount[3], magCount[3];

    // 1. Read accel val
    readAccelData(accelCount);

    //2. Read gyro val
    readGyroData(gyroCount);

    //3. Read mag val
    if(readMagData(magCount))
    {
        mpu9250[sel].mx = getMres(magCount[1]);
        mpu9250[sel].my = getMres(magCount[0]);
        mpu9250[sel].mz = -getMres(magCount[2]);
    }

    mpu9250[sel].ax = getAres(accelCount[0]);
    mpu9250[sel].ay = getAres(accelCount[1]);
    mpu9250[sel].az = getAres(accelCount[2]);

    mpu9250[sel].gx = getGres(gyroCount[0]);
    mpu9250[sel].gy = getGres(gyroCount[1]);
    mpu9250[sel].gz = getGres(gyroCount[2]);

    switch(sel){
        case 0:
            /*
            //imu_00
            mpu9250[sel].mx -= -50;
            mpu9250[sel].my -= 200;
            mpu9250[sel].mz -= 400;
            */
            //imu_01
            mpu9250[sel].mx -= 373;
            mpu9250[sel].my -= 123;
            mpu9250[sel].mz -= 260;

            break;
    }
}

void Read_RawData(uint8_t *des, uint8_t index)
{
    uint8_t acc_rawData[6];
    uint8_t gyro_rawData[6];
    uint8_t mag_rawData[8];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition


    uint8_t mag_fail_count = 0;
    const uint8_t mag_fail_count_limit = 64;
    //1. Read mag val
    do{
        AK8963_read(MAG_ST1, &mag_rawData[0], 8);
        mag_fail_count++;
        //SensorI2C_readReg(MPU9250_ADDRESS, EXT_SENS_DATA_00, &mag_rawData[0], 8);
    }while(((!(mag_rawData[0] & 0x01)) || (mag_rawData[7] & 0x08)) && (mag_fail_count < mag_fail_count_limit));

    /*
    SensorI2C_readReg(MPU9250_ADDRESS, EXT_SENS_DATA_00, &mag_rawData[0], 8);
    if(mag_rawData[0] & 0x01) { // wait for magnetometer data ready bit to be set
        if(!(mag_rawData[7] & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
            des[12] = mag_rawData[1];
            des[13] = mag_rawData[2];
            des[14] = mag_rawData[3];
            des[15] = mag_rawData[4];
            des[16] = mag_rawData[5];
            des[17] = mag_rawData[6];
        }
    }
    */

    //2. Read accel val
    SensorI2C_readReg(MPU9250_ADDRESS, ACCEL_XOUT_H, &acc_rawData[0], 6);  // Read the six raw data registers into data array
    des[0] = acc_rawData[1];
    des[1] = acc_rawData[0];
    des[2] = acc_rawData[3];
    des[3] = acc_rawData[2];
    des[4] = acc_rawData[5];
    des[5] = acc_rawData[4];

    //3. Read gyro val
    SensorI2C_readReg(MPU9250_ADDRESS, GYRO_XOUT_H, &gyro_rawData[0], 6);  // Read the six raw data registers sequentially into data array
    des[6] = gyro_rawData[1];
    des[7] = gyro_rawData[0];
    des[8] = gyro_rawData[3];
    des[9] = gyro_rawData[2];
    des[10] = gyro_rawData[5];
    des[11] = gyro_rawData[4];

    //Status of Mag
    if (mag_fail_count < mag_fail_count_limit)
    {
        //4. Read mag val
        des[12] = mag_rawData[1];
        des[13] = mag_rawData[2];
        des[14] = mag_rawData[3];
        des[15] = mag_rawData[4];
        des[16] = mag_rawData[5];
        des[17] = mag_rawData[6];
    }

}

void assignTestData(uint8_t count, uint8_t *des)
{
    uint8_t i = 0;
    for (i = 0; i < 18; i++)
    {
        if (i % 2 == 1){
            des[i] = 0x00;
        }
        else{
            des[i] = count;
            count++;
        }
    }
}

void Get_DeltaT(uint8_t sel)
{
    uint16_t deltat_100us;
    //計算deltat，單位(s)
    mpu9250[sel].now = Clock_getTicks();
    mpu9250[sel].deltat = (mpu9250[sel].now - mpu9250[sel].last) / 100000.0f;
    mpu9250[sel].last = mpu9250[sel].now;
    deltat_100us = mpu9250[sel].deltat * 10000;
    mpu9250[sel].data[18] = (deltat_100us >> 8) & 0x00FF;
    mpu9250[sel].data[19] = deltat_100us & 0x00FF;

}

void AHRSupdate(int index, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    float q1 = mpu9250[index].quaternion[0], q2 = mpu9250[index].quaternion[1], q3 = mpu9250[index].quaternion[2], q4 = mpu9250[index].quaternion[3];   // short name local variable for readability
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float pa, pb, pc;

    // Auxiliary variables to avoid repeated arithmetic
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0) return; // handle NaN
    norm = 1 / norm;        // use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0) return; // handle NaN
    norm = 1 / norm;        // use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    hx = 2 * mx * (0.5f - q3q3 - q4q4) + 2 * my * (q2q3 - q1q4) + 2 * mz * (q2q4 + q1q3);
    hy = 2 * mx * (q2q3 + q1q4) + 2 * my * (0.5f - q2q2 - q4q4) + 2 * mz * (q3q4 - q1q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = 2 * mx * (q2q4 - q1q3) + 2 * my * (q3q4 + q1q2) + 2 * mz * (0.5f - q2q2 - q3q3);

    // Estimated direction of gravity and magnetic field
    vx = 2 * (q2q4 - q1q3);
    vy = 2 * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;
    wx = 2 * bx * (0.5f - q3q3 - q4q4) + 2 * bz * (q2q4 - q1q3);
    wy = 2 * bx * (q2q3 - q1q4) + 2 * bz * (q1q2 + q3q4);
    wz = 2 * bx * (q1q3 + q2q4) + 2 * bz * (0.5f - q2q2 - q3q3);

    // Error is cross product between estimated direction and measured direction of gravity
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    if (Ki > 0)
    {
        mpu9250[index].exInt += ex;      // accumulate integral error
        mpu9250[index].eyInt += ey;
        mpu9250[index].ezInt += ez;
    }
    else
    {
        mpu9250[index].exInt = 0.0f;     // prevent integral wind up
        mpu9250[index].eyInt = 0.0f;
        mpu9250[index].ezInt = 0.0f;
    }

    // Apply feedback terms
    gx = gx + Kp * ex + Ki * mpu9250[index].exInt;
    gy = gy + Kp * ey + Ki * mpu9250[index].eyInt;
    gz = gz + Kp * ez + Ki * mpu9250[index].ezInt;

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * mpu9250[index].deltat);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * mpu9250[index].deltat);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * mpu9250[index].deltat);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * mpu9250[index].deltat);

    // Normalise quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    mpu9250[index].quaternion[0] = q1 * norm;
    mpu9250[index].quaternion[1] = q2 * norm;
    mpu9250[index].quaternion[2] = q3 * norm;
    mpu9250[index].quaternion[3] = q4 * norm;
}

void QtoEular(int index, float q0, float q1, float q2, float q3) {
    mpu9250[index].yaw = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
    mpu9250[index].pitch = -asin(2.0f * (q1 * q3 - q0 * q2));
    mpu9250[index].roll = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
    mpu9250[index].yaw *= 180 / PI;
    mpu9250[index].pitch *= 180 / PI;
    mpu9250[index].roll *= 180 / PI;
}
