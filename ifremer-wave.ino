#include <SPI.h>
#include "wave_analyser.h"
//#include "wave_test.h"

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 300 /* Time ESP32 will go to sleep (in seconds) */

//WaveAnalyser(float cutoff_freq = CUTOFF_FREQ, float sampling_time = SAMPLING_TIME, int order = INIT_ORDER, int n_data_array = N_DATA_ARRAY,
//    int n_grad = N_GRAD, int innitial_calibration_delay = INNITAL_CALIBRATION_DELAY, int n_w = N_WAVES);
WaveAnalyser waveAnalyser;

bool upd = false;
float significantWH = 0.0, averageWH = 0.0, averagePeriod = 0.0;

void setup()
{
  Wire.begin();
  delay(1000);
  Serial.begin(38400);
  delay(1000);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  
  waveAnalyser.setup();
  
  //waveAnalyser.setCalibrationDelay(60000); //Set new innitial calibration delay time in millis
  //waveAnalyser.setNumberOfWaves(10); //Set new number of waves to measure in each itteration, max wave number is 100
  
  
}

void loop()
{  
  if(!upd){
    upd = waveAnalyser.update(); //Returns true when new wave is calculated
    if(upd){
      significantWH = waveAnalyser.getSignificantWave();
      averageWH = waveAnalyser.getAverageWave();
      averagePeriod = waveAnalyser.getAveragePeriod();
      Serial.print("Significant WH: ");
      Serial.println(significantWH);
      Serial.print("Average WH: ");
      Serial.println(averageWH);
      Serial.print("Average Period: ");
      Serial.println(averagePeriod);
      Serial.println("Go to sleep...");
      delay(1000);
      upd = false;
      esp_deep_sleep_start();
    }
  }
}

