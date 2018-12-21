#pragma once
#ifndef _WAVE_ANALYSER_H
#define _WAVE_ANALYSER_H

#include <Arduino.h>
#include "array_structures.h" //Quaternion and vector classes
#include "MPU9250.h" //Sensor library
#include <stdarg.h>
#include "SD.h"
#include "FS.h"
#include <SPI.h>

#define CUTOFF_FREQ 0.4 //Cutoff frquency
#define SAMPLING_TIME 0.01 //Sampling time in seconds
#define INIT_ORDER 3
#define N_DATA_ARRAY 3000 //Length of data array
#define N_GRAD 50  //Distanec for gardient calculation
#define N_GRAD_COUNT 20 //Number of points with the same gradient to consider
#define N_WAVES_MAX 50 //Max number of waves to calculate - defines array length
#define N_WAVES 5 //Innitial number of waves to calculate - can be adjusted by the user
#define INNITAL_CALIBRATION_DELAY 120000 //Delay for quaternions calculations to calibrate

class WaveAnalyser
{
public:
	
	//WaveAnalyser(); 
	WaveAnalyser(float cutoff_freq = CUTOFF_FREQ, float sampling_time = SAMPLING_TIME, int order = INIT_ORDER, int n_data_array = N_DATA_ARRAY,
		int n_grad = N_GRAD, int innitial_calibration_delay = INNITAL_CALIBRATION_DELAY, int n_w = N_WAVES);
	void init();
	void setup();
	bool update();

	//Set function
	void setCalibrationDelay(int);
	void setNumberOfWaves(int);
	float getSignificantWave();
	float getAverageWave();
	float getAveragePeriod();

private:

	MPU9250 mpu; //MPU9250 sensor
	
	MotionArray *A; //Filtered acceleration data array

	bool full = false;
	int grad = 0; //Current motion gradient
	int current_grad = 0; //Current staedy direction of movement
	int grad_count = 0; //Gradient counter - count number of same gradients in a row

	int max_idx[2 * N_WAVES_MAX];
	float height[2 * N_WAVES_MAX];
	float half_period[2 * N_WAVES_MAX];
	int wave_max_counter = 0;
	int wave_counter = 0;

	float old_offset = 0.0; //Acceleration ofset, old and new wave
	float new_offset = 0.0;

	int wait_time;
	int print_wait_time = 0;
	int calibration_delay; //Initial delay time for calculations calibration

	int n_waves; //Number of waves to measure 
	float wave_avg = 0.0; //Last average wave heigth
	float wave_significant = 0.0;
	float period_avg = 0.0;

	bool analyseData();
	void analyseGradient();
	bool analyseWaves();
	int16_t calculateOffset(int, int);
	void calculateWaves();
	void sort();

	//SD card logging
	File logfile;
	char filename[30];
};


#endif 
