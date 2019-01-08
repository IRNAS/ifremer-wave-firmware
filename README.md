# ifremer-wave-firmware
Firmware for wave tracking and loraWan communication using STM32 board. 

Firmware for wave tracking - measures wave height and period. Communicate significant wave height and average period using LoraWan. 
Firmware can be used on the STM32 board - with LoraWan communication support and on the ESP32 board with the SD card logging support. 

# STM32
For usage with STM32 and LoraWan communication you will need to run ifremer_wave_lorawan.ino as the main file, while sensors.ino and comms.ino files are needed as well. Needed libraries:

[wave-analyser.h](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/wave_analyser.h) and [wave-analyser.cpp](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/wave_analyser.cpp) - main wave analyser library.

[MPU9250.h](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/MPU9250.h),
[MPU9250.cpp](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/MPU9250.cpp),
[MPU9250RegisterMap.h](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/MPU9250RegisterMap.h) - MPU9250 sensor library rewriten from kriswiner [MPU9250 library](https://github.com/kriswiner/MPU9250). See the link for more usage examples. 

[array_structures.h](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/array_structures.h) - header for defining calculation data arrays and Quaternions.

[debug_print.h](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/debug_print.h) and [debug_print.cpp](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/debug_print.cpp) - library for debug print. Specify printut level by seting #define DEBUG 1 to 1-5.

filters.h, filters.cpp and filters_defs.h - class with low pass filter, used for acceleration data filtering. 


The WaveAnalyser class is defined inside sensors.ino. Adjust settings be defining parameters in the WaveAnalyser constructor:
* **cutoff_freq** - cutoff frequency for the low pass filter
* **sampling_time** - sampling time in seconds
* **order - order** of low-pass filter
* **n_data_array** - length of data storage array
* **n_grad** - number of points for gradient calculation
* **initial_calibration_delay** - initial delay for calibration in micro-seconds
* **n_w** - number of waves to measure 
```
WaveAnalyser(float cutoff_freq = CUTOFF_FREQ, float sampling_time = SAMPLING_TIME, int order = INIT_ORDER, int n_data_array = N_DATA_ARRAY,
    int n_grad = N_GRAD, int innitial_calibration_delay = INNITAL_CALIBRATION_DELAY, int n_w = N_WAVES);
```    
Or you can use the default constructor:
```
WaveAnalyser waveAnalyser,
```
the pre set parameters are defined in the wave_analyser.h file. You can change the initial_calibration_delay and n_w later during the setup using 
```
waveAnalyser.setCalibrationDelay(1000); //Set new innitial calibration delay time in millis
waveAnalyser.setNumberOfWaves(5);
```
# Operation
The board will wake up from sleep after the pre-set time defined in ifremer_wave_lorawan.ino:
```
STM32L0.stop(5000); // Enter STOP mode and wait for an interrupt
```
Then setup of the wave_analyser and MPU9250 sensor is called with:
```
wave_setup();
```
Each loop ```update_wave()``` is called to update sensor data. For pre determied period **initial_calibration_delay** sensor is calibrating then **n_data_array** measurments are colected with sampling time **sampling_time**. When sufficient values are recorded and  **n_w** waves are detected average wave-height, significant wave-height and average period will be printed and send via LoraWan communication.

# ESP32

For usage with the ESP32 board un [ifremer_wave.ino](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/ifremer_wave.ino). Same libraries are needed, except from HDC2080.h and HDC2080.cpp, LIS2DH12.h and LIS2DH12.cpp, ensors.ino and comms.ino files. There is no support for the LoraWan communication. 

The folowing changes are needed:
* To enable SD card logging, add SD.h and FS.h libraries and uncoment ```#define SD_CARD``` inside wave_analyser.h file. 
* To enable debug printout, comment ```#define STM32_BOARD``` inside debug_print.h file. 

It will wakeup the device every TIME_TO_SLEEP seconds (default 300 s) and take measurments. Rotated Z-axis acceleration and final data are stored to SD card. 
