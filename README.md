# ifremer-wave-firmware
Firmware for wave tracking

Firmware for wave tracking - measures wave height and period.

Run [ifremer_wave.ino](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/ifremer_wave.ino).
It will wakeup the device every TIME_TO_SLEEP seconds (default 300 s) and take measurments. 

Adjust settings be defining parameters in the WaveAnalyser constructor:
* cutoff_freq - cutoff frequency for the low pass filter
* sampling_time - sampling time in seconds
* order - order of low-pass filter
* n_data_array - length of data storage array
* n_grad - number of points for gradient calculation
* initial_calibration_delay - initial delay for calibration in micro-seconds
* n_w - number of waves to measure 

When n_w waves are detected average wave-height, significant wave-height and average period will be printed. 

Files:

[ifremer_wave.ino](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/ifremer_wave.ino) - main sketch.

[wave-analyser.h](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/wave_analyser.h) and [wave-analyser.cpp](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/wave_analyser.cpp) - main wave analyser library.

[MPU9250.h](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/MPU9250.h)
[MPU9250.cpp](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/MPU9250.cpp)
[MPU9250RegisterMap.h](https://github.com/IRNAS/ifremer-wave-firmware/blob/master/MPU9250RegisterMap.h) - MPU9250 sensor library rewriten from kriswiner [MPU9250 library](https://github.com/kriswiner/MPU9250).
