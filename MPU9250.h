#pragma once
#ifndef MPU9250_H
#define MPU9250_H

#include <Arduino.h>
#include <Wire.h> 
#include "MPU9250RegisterMap.h" //Register file
#include "array_structures.h" //Quaternion and vector classes
#include "debug_print.h"
#include <stdarg.h>

#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f
#define INNITIAL_DATA_DELAY 10

enum Ascale {AFS_2G = 0, AFS_4G, AFS_8G, AFS_16G };
enum Gscale { GFS_250DPS = 0, GFS_500DPS, GFS_1000DPS, GFS_2000DPS };
enum Mscale { MFS_14BITS = 0, MFS_16BITS };

class MPU9250
{
	// Specify sensor full scale
	uint8_t Gscale = GFS_250DPS;
	uint8_t Ascale = AFS_2G;
	uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
	uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
	float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

	// Calibration
    float magCalibration[3] = {0, 0, 0}; // factory mag calibration
    float magBias[3] = { 254.35, -148.14, -166.12 }; //Pre-determined
    float magScale[3]  = { 1.03, 1.02, 0.95 }; // Bias corrections for gyro and accelerometer
    float gyroBias[3] = { 0.77, 0.03, 0.09 }; // bias corrections 
    float accelBias[3] = { -2.72 / 1000.0, 13.91 / 1000.0, 21.87 / 1000.0 }; // bias corrections

    int16_t tempCount;      // temperature raw count output
    float temperature;    // Stores the real internal chip temperature in degrees Celsius
    float SelfTest[6];    // holds results of gyro and accelerometer self test

	float GyroMeasError = PI * (4.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
	float GyroMeasDrift = PI * (0.0f / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
	float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
	float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

	float a[3] = { 0.0f, 0.0f, 0.0f }, g[3] = { 0.0f, 0.0f, 0.0f }, m[3] = { 0.0f, 0.0f, 0.0f }; // variables to hold latest sensor data values 
	float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
	float eInt[3] = { 0.0f, 0.0f, 0.0f };       // vector to hold integral error for Mahony method
    float pitch, yaw, roll;
    float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
    
	uint32_t delt_t = 0, count = 0;  // used to control display output rate
	int data_delay = 10; //Delay for data output
	float deltat = 0.0f, sum_send = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
	uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
	uint32_t Now = 0;

	Quaternion Q; //Quaternion
	VectorFloat Acc; //Acc vector

    float magnetic_declination = 4.62; // Ljubljana

public:

	MPU9250(); //Constructor
	void setup(); //Setup function
	void calibrateAccelGyro(); //Public MPU9250 calibration function
	void calibrateMag(); //Public magnetometer calibration function
	bool isConnectedMPU9250(); //Check if MPU9250 is connected
	bool isConnectedAK8963(); //Check if AK8963 magnetometer is connected
	bool update(); //Update data
	void updateAccelGyro(); //Update accelometer and gyro data
	void updateMag(); //Update magnetometer readings

	int16_t getZacc(); //Get Z rotated acceleration
	float getDt(); //Get Z rotated acceleration
	void setDataDelay(int); //Re-set value of data delay

	void MPU9250sleep(); //Go to sleep



private:
	bool available(); //Is new data avaliable

	void getAres(); //Get accelometer scale
	void getGres(); //Get gyro scale
	void getMres(); //Get magnetometer scale

	void initAK8963(float * destination); //Initialise magnetometer
	void initMPU9250(); //Initialise accelometer and gyro

	void readMPU9250Data(int16_t * destination); //read MPU9250 data
	void readMagData(int16_t * destination); //read magnetometer data
	int16_t readTempData(); //read temperature data

	void calibrateMPU9250(float * dest1, float * dest2); //Calibrate accelometer and gyro
	void magcalMPU9250(float * dest1, float * dest2); //Calibrate magnetometer

	void updateRPY(); //Update roll, pitch and yaw
	void MPU9250SelfTest(float * destination); //Accelerometer and gyroscope self test; check calibration wrt factory settings

	void writeByte(uint8_t address, uint8_t subAddress, uint8_t data); //Write byte to register
	uint8_t readByte(uint8_t address, uint8_t subAddress); //Read byte from register
	void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest); //Read multiple bytes from register
	void pirntI2CError();

	void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
	void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

    bool b_ahrs {true};

    uint8_t i2c_err_;


};


#endif // MPU9250_H
