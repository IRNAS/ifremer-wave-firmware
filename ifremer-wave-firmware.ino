/* Arduino based firmware for ifremer wave analysis system
 *
 *  Built for EU868, modify code for other bands if hardware is available
 *    
 *  Currently implemented:
 *  Reading sensors
 *  Sending data periodically via LoraWAN
 *  Sleep mode (about 10uA)
 *  
 *  TODO:
 *  Properly implement sensors for low-power operation, currently about 6uA
 *  Note HW modifications are required for power optimization.
 *  Cover corner-case of Join failures etc...
 *  
 *  INSTRUCTIONS:
 *  Install: https://github.com/MartinBloedorn/libFilter/tree/25a03b6cb83cfef17b9eee85eb34e807bd0ad135
 *  Edit Comms tab to enter LoraWAN details
 *  Upload
 *  
 * Copyright (C) 2018 Musti - Institute IRNAS - contact@irnas.eu
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Affero General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Affero General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <STM32L0.h>
#include "TimerMillis.h"
#include <Wire.h>
#include "wave_analyser.h"

#define sleep_period 1 // sleep duration in minutes

TimerMillis wdtTimer; //timer for transmission events

#define debug
#define serial_debug  Serial1

/* Define Wave analyser
 *  You can specify additional parameters in the inicialisation to change defoult values. Defoult values are defined in the wave_analyser.h file.
 *  float cutoff_freq - cutoff fequency for the low pass filter
 *  float sampling_time - sensor sampling time
 *  int order - low pass filter order
 *  int n_data_array - length of data array
 *  int n_grad - number of points for gradient calculation
 *  int innitial_calibration_delay - initial delay for sensor calibration in millis
 *  int n_w - number of waves to detect

WaveAnalyser(float cutoff_freq = CUTOFF_FREQ, float sampling_time = SAMPLING_TIME, int order = INIT_ORDER, int n_data_array = N_DATA_ARRAY,
    int n_grad = N_GRAD, int innitial_calibration_delay = INNITAL_CALIBRATION_DELAY, int n_w = N_WAVES);
*/
WaveAnalyser waveAnalyser; //Defoult constructor

void wave_setup( void ){
  waveAnalyser.setup();
  //waveAnalyser.setCalibrationDelay(1000); //Set new innitial calibration delay time in millis
  //waveAnalyser.setNumberOfWaves(5); //Set new number of waves to measure in each itteration, max wave number is 100
}

bool update_wave( void ){
  return waveAnalyser.update();
}

// watchdog timer ISR
void ISR_WDT() {
    STM32L0.wdtReset();
}

void setup( void )
{
    //Serial port setup
    #ifdef debug
      serial_debug.begin(115200);
    #endif

    //Functions setup
    comms_setup(); // LoraWAN communication
    sensors_setup(); // Sensor communication

    // Watchdog setup with kick every 15s and 18s timeout
    wdtTimer.start(ISR_WDT, 0, 15*1000);
    STM32L0.wdtEnable(18000);
}
void loop( void )
{
  // setup the wave measurement code
  wave_setup();
  
  // block while the wave function performs and call it periodically
  while(update_wave()==false);

  //read other sensors and then send data
  read_sensors();
    
  // send the measurements and acquire all other sensor data
  comms_transmit();

  //flush data before sleep, otherwise not sent correctly
  #ifdef debug
    serial_debug.println("Sleep");
    serial_debug.flush();
  #endif

  Wire.beginTransmission(0x68);
  Wire.write(0x6b);
  Wire.write(0x3f);
  Wire.endTransmission();
  delay(100);
  Wire.beginTransmission(0x68);
  Wire.write(0x6b);
  Wire.write(0x48);
  Wire.endTransmission();
  delay(100);
  Wire.beginTransmission(0x0C);
  Wire.write(0x0A);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x6b);
  Wire.write(0x40);
  Wire.endTransmission();

  delay(5000);

  // sleep for a defined time
  STM32L0.stop(sleep_period*60*1000); // Enter STOP mode and wait for an interrupt
}
