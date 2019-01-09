#pragma once
#ifndef _WAVE_ANALYSER_H
#define _WAVE_ANALYSER_H

#include <Arduino.h>
#include "array_structures.h" //Quaternion and vector classes
#include "MPU9250.h" //Sensor library
#include <stdarg.h>
//#include "SD.h" - add for ESP32 to use SD logging
//#include "FS.h" - add for ESP32 to use SD logging
#include <SPI.h>

#define CUTOFF_FREQ 0.4 //Cutoff frequency
#define SAMPLING_TIME 0.01 //Sampling time in seconds - approximation
#define INIT_ORDER 3 //Low pass filter order
#define N_DATA_ARRAY 3000 //Length of data array
#define N_GRAD 50  //Distance for gradient calculation
#define N_GRAD_COUNT 20 //Number of points with the same gradient to consider as new direction
#define N_WAVES_MAX 50 //Max number of waves to calculate - defines array length (increase if needed)
#define N_WAVES 5 //Initial number of waves to calculate - can be adjusted by the user
#define INNITAL_CALIBRATION_DELAY 120000 //Delay for quaternions calculations to calibrate

//#define SD_CARD //If using ESP32 and want to use SD card logging uncomment

class WaveAnalyser
{
public:
	
	//WaveAnalyser(); 
	WaveAnalyser(float cutoff_freq = CUTOFF_FREQ, float sampling_time = SAMPLING_TIME, int order = INIT_ORDER, int n_data_array = N_DATA_ARRAY,
		int n_grad = N_GRAD, int innitial_calibration_delay = INNITAL_CALIBRATION_DELAY, int n_w = N_WAVES); //Constructor with default parameters
	void init(); //Initialization
	void setup(); //Setup
	bool update(); //Update reading - call every time from the main loop

	//Set function
	void setCalibrationDelay(int); //Change calibration delay after initialization
	void setNumberOfWaves(int); //Change number of waves to be measured after initialization
	float getSignificantWave(); 
	float getAverageWave();
	float getAveragePeriod();

private:

	MPU9250 mpu; //MPU9250 sensor
	MotionArray *A; //Filtered acceleration data array

	bool full = false; //Denotes if data array is full
	int grad = 0; //Current motion gradient
	int current_grad = 0; //Current steady direction of movement
	int grad_count = 0; //Gradient counter - count number of same gradients in a row

	int max_idx[2 * N_WAVES_MAX]; //Indices of local maximums and minimums
	float height[2 * N_WAVES_MAX]; //Measured heights
	float half_period[2 * N_WAVES_MAX]; //Measured half-periods
	int wave_max_counter = 0; //Maximum counter
	int wave_counter = 0; //Wave counter

	float old_offset = 0.0; //Acceleration offset, old and new wave
	float new_offset = 0.0;

	int wait_time; 
	int print_wait_time = 0; 
	int calibration_delay; //Initial delay time for calculations calibration

	int n_waves; //Number of waves to measure 
	float wave_avg = 0.0; //Last average wave height
	float wave_significant = 0.0;
	float period_avg = 0.0;

	bool analyseData();
	void analyseGradient();
	bool analyseWaves();
	int16_t calculateOffset(int, int);
	void calculateWaves();
	void sort();

	//SD card logging
	//File logfile;
	char filename[30];
};


#endif 
