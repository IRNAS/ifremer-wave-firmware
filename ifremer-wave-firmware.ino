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

#define sleep_period 1 // sleep duration in minutes

TimerMillis wdtTimer; //timer for transmission events

#define debug
#define serial_debug  Serial1

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
    
  // send the measurements and acquire all other sensor data
  comms_transmit();

  //flush data before sleep, otherwise not sent correctly
  #ifdef debug
    serial_debug.println("Sleep");
    delay(100);
    serial_debug.flush();
  #endif

  // sleep for a defined time
  STM32L0.stop(sleep_period*60*1000); // Enter STOP mode and wait for an interrupt
}
