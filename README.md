# ifremer-wave-firmware
Firmware for wave tracking and loraWan communication using STM32L0 board. 

Firmware for wave tracking - measures wave height and period of ordinary gravity waves with the average period between 2 and 15 seconds. Significant wave height and average period are communicated using LoraWan. 
Firmware can be used on the STM32L0 board - with the LoraWan communication support and also on the ESP32 board with the SD card logging support. 

<img src="https://github.com/IRNAS/ifremer-wave-firmware/blob/master/ESP32_setup.jpg" title="ESP32 and MPU9250 sensor setup." width="600" />
ESP32 and MPU9250 sensor setup.

# Wave results
The system delivers the results in two versions, depending on the configuration used. ESP32 version logs all the measurements to the SD card which includes raw data and measurement results. The STM32 version with LoraWAN delivers only the results via a LoraWan packet for example:

```
  "Acceleration": 0,
  "AirPressure": 980.3,
  "Average_period": 64306,
  "Average_wave_height": 10107,
  "Battery": 3.29,
  "CPU_temperature": 19.35,
  "Humidity": 30.42,
  "Info": 1,
  "Significant_wave_height": 2641,
  "Temperature": 22.56
```

# Power consumption
The system operates a wave detection period and a sleep period. A typical wave detection with default settings is 180s long at an average power consumption of 12mA@3.75V, while the sleep period power consumption 100uA@3.75V. There are possible further power optimizations. Provided a 17min sleep duration is used, we have 3 detections per hour at an average consumption of 1.8mAh, thus  typical 18650 LiPo battery should deliver about 2 months of operation.

# STM32L0 - Murata ABZ LoraWAN module
For usage with STM32 and LoraWan communication you will need to run [ifremer_wave_lorawan.ino](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/ifremer_wave_lorawan.ino) as the main file, while [sensors.ino](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/sensors.ino) and [comms.ino](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/comms.ino) files are needed as well. Add libraries:

[wave-analyser.h](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/wave_analyser.h) and [wave-analyser.cpp](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/wave_analyser.cpp) - main wave analyser library.

[MPU9250.h](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/MPU9250.h),
[MPU9250.cpp](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/MPU9250.cpp),
[MPU9250RegisterMap.h](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/MPU9250RegisterMap.h) - MPU9250 sensor library rewriten from kriswiner [MPU9250 library](https://github.com/kriswiner/MPU9250). See the link for more usage examples. 

[array_structures.h](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/array_structures.h) - header for defining calculation data arrays and Quaternions.

[debug_print.h](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/debug_print.h) and [debug_print.cpp](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/debug_print.cpp) - library for debug print. Specify printut level by seting #define DEBUG 1 to 1-5.

[filters](https://github.com/MartinBloedorn/libFilter/tree/25a03b6cb83cfef17b9eee85eb34e807bd0ad135) - class with low pass filter, used for acceleration data filtering. 

[HDC2080.h](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/HDC2080.h) and [HDC2080.cpp](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/HDC2080.cpp)

[LIS2DH12.h](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/LIS2DH12.h) and [LIS2DH12.cpp](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/LIS2DH12.cpp)

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
the pre-set parameters are defined in the wave_analyser.h file. You can change the **initial_calibration_delay** and **n_w** later during the setup using 
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

For usage with the ESP32 board run [ifremer_wave.ino](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/ifremer_wave.ino) main. Same libraries are needed, except from HDC2080.h and HDC2080.cpp, LIS2DH12.h and LIS2DH12.cpp, sensors.ino and comms.ino files. There is no support for the LoraWan communication. 

The folowing changes are needed:
* To enable SD card logging, add SD.h and FS.h libraries and uncoment ```#define SD_CARD``` inside wave_analyser.h file. 
* To enable debug printout, comment ```#define STM32_BOARD``` inside debug_print.h file. 

It will wakeup the device every TIME_TO_SLEEP seconds (default 300 s) and take measurments. Rotated Z-axis acceleration and final data are stored to the SD card. 

# Calibration procecss
To calibrate the gyroscope, a few steps must be completed.

1. Connect the battery
1. The LED will start blinking
1. Tap the device against the desk until the LED turns on solidly
1. When the LED turns off, start moving the unit in figure 8 shape for 30s untill the LED turns back on
1. Put the device on the flat surface for another 30s.
1. LED will blink and then the calibration is completed

Upon correct calibration the device will return close to 0 wave height when rested at a flat surface.
