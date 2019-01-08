// Handle sensors
#include <STM32L0.h>
#include <Wire.h> 
#include <Adafruit_Sensor.h> // Install through Manager
#include <Dps310.h>// Install through Manager
#include "HDC2080.h"
#include "LIS2DH12.h"
#include "wave_analyser.h"

// I2C
//Adafruit_LIS3DH lis = Adafruit_LIS3DH();
LIS2DH12 lis = LIS2DH12();

#define LIS_INT2  6 //PB2

// Dps310 object
Dps310 Dps310PressureSensor = Dps310();

// HDC2080 object
HDC2080 hdc2080 = HDC2080();

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

void ISR_LIS() {

    STM32L0.wakeup();

    #ifdef debug
        serial_debug.println("LIS (ISR_LIS) - interrupt on accel");
    #endif

}

void wave_setup( void ){
  waveAnalyser.setup();
  //waveAnalyser.setCalibrationDelay(1000); //Set new innitial calibration delay time in millis
  //waveAnalyser.setNumberOfWaves(5); //Set new number of waves to measure in each itteration, max wave number is 100
}

bool update_wave( void ){
  return waveAnalyser.update();
}

void sensors_setup( void )
{
    pinMode(LIS_INT2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LIS_INT2), ISR_LIS, FALLING);

    // Each sensor should have a check to determine if present on boot
    // Code in reading should handle if sensor becomes unavailable
    // All sensors need to be placed in the low power mode

    //LIS2DH sensor
    lis.begin();
    //lis.setRange(LIS3DH_RANGE_2_G);
    //lis.setClick(2, 40);
    
    //DSO130 sensor
    Dps310PressureSensor.begin(Wire);

    //hdc2080
    hdc2080.begin();
     #ifdef debug
        serial_debug.println("sensors_setup()");
    #endif  
}

void read_sensors( void )
{
    float stm32l0_vdd = STM32L0.getVDDA();
    float stm32l0_temp = STM32L0.getTemperature();
  
    //LIS
    lis.read();      // get X Y and Z data at once
    float lis_movement=lis.acc_x_value+lis.acc_y_value+lis.acc_z_value;
  
    //DPOS130 read
    int32_t dsp310_temp;
    int32_t dsp310_pres;
    
    int ret = Dps310PressureSensor.measureTempOnce(dsp310_temp, 7);
    ret += Dps310PressureSensor.measurePressureOnce(dsp310_pres, 7);
    dsp310_pres=dsp310_pres/10;
  
    hdc2080.read();
    float hdc2080_temp = hdc2080.getTemp();
    float hdc2080_hum = hdc2080.getHum();

    packet.sensor.stat=   0x01;
    packet.sensor.t1  =   (int8_t)hdc2080_temp;
    packet.sensor.t01 =   (uint8_t)((hdc2080_temp-packet.sensor.t1)*100);
    packet.sensor.h1 =    (int8_t)hdc2080_hum;
    packet.sensor.h01 =   (uint8_t)((hdc2080_hum-packet.sensor.h1)*100);
    packet.sensor.ap =    (uint16_t)dsp310_pres;
    packet.sensor.acc=    (int8_t)(lis_movement/10);//ACC
    packet.sensor.vdd =   (uint16_t)(stm32l0_vdd*100);
    packet.sensor.tc1 =   (int8_t)stm32l0_temp;
    packet.sensor.tc01 =  (uint8_t)((stm32l0_temp-packet.sensor.tc1)*100);
    packet.sensor.significant_wh = (uint16_t)(waveAnalyser.getSignificantWave() * 1000); //height in mm
    packet.sensor.average_wh = (uint16_t)(waveAnalyser.getAverageWave() * 1000); //height in mm
    packet.sensor.average_period = (uint16_t)(waveAnalyser.getAveragePeriod() * 1000); //period in us
    Serial1.println(packet.sensor.significant_wh);
    Serial1.println(packet.sensor.average_wh);
    Serial1.println(packet.sensor.average_period);
    
    #ifdef debug
      /*serial_debug.println(""); 

      serial_debug.print("hdc2080_temp: "); serial_debug.print(hdc2080_temp);
      serial_debug.print(" 0x"); serial_debug.print(packet.sensor.t1,HEX); serial_debug.print(" 0x"); serial_debug.println((uint8_t)((hdc2080_temp-packet.sensor.t1)*100),HEX);
       
      serial_debug.print("hdc2080_hum: "); serial_debug.print(hdc2080_hum);
      serial_debug.print(" 0x"); serial_debug.print((int8_t)hdc2080_hum,HEX); serial_debug.print(" 0x"); serial_debug.println((uint8_t)((hdc2080_hum-packet.sensor.h1)*100),HEX);

      serial_debug.print("dsp310_pres: "); serial_debug.print(dsp310_pres);
      serial_debug.print(" 0x"); serial_debug.println(dsp310_pres,HEX);

      serial_debug.print("lis_movement: "); serial_debug.print(lis_movement);
      serial_debug.print(" 0x"); serial_debug.println((int8_t)(lis_movement/10),HEX);
      
      serial_debug.print("stm32l0_vdd: "); serial_debug.print(stm32l0_vdd);
      serial_debug.print(" 0x"); serial_debug.println((uint16_t)(stm32l0_vdd*100),HEX);
        
      serial_debug.print("stm32l0_temp: "); serial_debug.print(stm32l0_temp);
      serial_debug.print(" 0x"); serial_debug.print((int8_t)stm32l0_temp,HEX);serial_debug.print(" 0x"); serial_debug.println((uint8_t)((stm32l0_temp-packet.sensor.tc1)*100),HEX);
               
      serial_debug.print("dsp310_temp: "); serial_debug.println(dsp310_temp);
      
      serial_debug.print("dsp310_pres: "); serial_debug.println(dsp310_pres); 

      serial_debug.print("packet.bytes[");
      serial_debug.print(sizeof(sensorData_t));
      serial_debug.print("] ");
      for(int i = 0; i < sizeof(sensorData_t); i++){
        serial_debug.print(" 0x");
        serial_debug.print(packet.bytes[i],HEX);
      }
      serial_debug.println(""); */
    #endif  
}
