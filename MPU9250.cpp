#include "MPU9250.h"

/* CONSTRUCTOR */
MPU9250::MPU9250() {};

/* PUBLIC METHODS */

#pragma region void MPU9250::setup()
/* SETUP FUNCTION
Input: /
Output: /
Description:

*/
void MPU9250::setup()
{
	data_delay = INNITIAL_DATA_DELAY;

	uint8_t m_whoami = 0x00;
	uint8_t a_whoami = 0x00;

	m_whoami = isConnectedMPU9250();
	if (m_whoami)
	{
		LOG(3,"MPU9250 is online...");
		MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values

		LOG(3, "x-axis self test: acceleration trim within : %.2f % of factory value", SelfTest[0]);
		LOG(3, "y-axis self test: acceleration trim within : %.2f % of factory value", SelfTest[1]);
		LOG(3, "z-axis self test: acceleration trim within : %.2f % of factory value", SelfTest[2]);
		LOG(3, "x-axis self test: gyration trim within : %.2f % of factory value", SelfTest[3]);
		LOG(3, "y-axis self test: gyration trim within : %.2f % of factory value", SelfTest[4]);
		LOG(3, "z-axis self test: gyration trim within : %.2f % of factory value", SelfTest[5]);

		getAres();
		getGres();
		getMres();

		LOG(3, "Calibrate gyro and accel");
		//accelgyrocalMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
		LOG(3, "Accel biases (mg): %.2f %.2f %.2f", 1000.*accelBias[0], 1000.*accelBias[1], 1000.*accelBias[2]);
		LOG(3, "Gyro biases (dps):  %.2f %.2f %.2f", gyroBias[0], gyroBias[1], gyroBias[2]);

		initMPU9250();
		LOG(3, "MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature


		a_whoami = isConnectedAK8963();
		if (a_whoami)
		{
			initAK8963(magCalibration);
			LOG(3, "AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
		}
		else
		{
			LOG(0, "Could not connect to AK8963: 0x %d", a_whoami);
			while (1);
		}
	}
	else
	{
		LOG(0, "Could not connect to MPU9250: 0x %d", m_whoami);
		while (1);
	}
}
#pragma endregion

#pragma region void MPU9250::calibrateAccelGyro()
/* Calibrate MPU9250 accelometer and gyro - public function
Input: /
Output: /
Description: calls private calibration function.
*/
void MPU9250::calibrateAccelGyro()
{
	calibrateMPU9250(gyroBias, accelBias);
}
#pragma endregion

#pragma region void MPU9250::calibrateMag()
/* Public magnetometer calibration function
Input: /
Output: /
Description: Calls private magnetometer calibration function
*/
void MPU9250::calibrateMag()
{
	magcalMPU9250(magBias, magScale);
}
#pragma endregion

#pragma region bool MPU9250::isConnectedMPU9250()
/* Check if MPU9250 is connected
Input: /
Output: bool connected/not connected
Description: check if sensor is connected
*/
bool MPU9250::isConnectedMPU9250()
{
	byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
	LOG(3, "MPU9250 WHO AM I = %02X", c);
	LOG(3, "Compare to  =  %02X", MPU9250_WHOAMI_DEFAULT_VALUE);
	return (c == MPU9250_WHOAMI_DEFAULT_VALUE);
}
#pragma endregion

#pragma region bool MPU9250::isConnectedAK8963()
/* Check if AK8963 magnetometer is connected
Input: /
Output: bool connected/ not connected
Description: check if magnetometer is connected
*/
bool MPU9250::isConnectedAK8963()
{
	byte c = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);
	LOG(3, "AK8963  WHO AM I = %02X", c);
	LOG(3, "Compare to  =  %02X", AK8963_WHOAMI_DEFAULT_VALUE);
	return (c == AK8963_WHOAMI_DEFAULT_VALUE);
}
#pragma endregion

#pragma region bool MPU9250::update()
/* Update data
Input: /
Output: bool - was the update done?
Description:
*/
bool MPU9250::update()
{
	if (available())
	{  // On interrupt, check if data ready interrupt
		updateAccelGyro();
		updateMag(); // TODO: set to 30fps?
	}
	
	Now = micros();
	deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
	//Serial.println((Now - lastUpdate));
	//Serial.print(" ");
	//Serial.println(deltat, 7);
	sum += deltat; // sum for averaging filter update rate
	lastUpdate = Now;

	MadgwickQuaternionUpdate(a[0], a[1], a[2], g[0]*PI / 180.0f, g[1] *PI / 180.0f, g[2] *PI / 180.0f, m[1], m[0], m[2]);
	//MahonyQuaternionUpdate(a[0], a[1], a[2], g[0]*PI / 180.0f, g[1]*PI / 180.0f, g[2]*PI / 180.0f, m[1], m[0], m[2]);

	delt_t = millis() - count;

	if (delt_t > 10) {

		updateRPY();

		Acc.x = a[0];
		Acc.y = a[1];
		Acc.z = a[2];

		Acc.rotate(&Q);

		LOG(3, "%d, %d, %d, %d", sum, (int)(Acc.x * 1000), (int)(Acc.y * 1000), (int)(Acc.z * 1000));

		count = millis();
		sum_send = sum;
		sum = 0;

		return true;
	}
	else {
		return false;
	}

}
#pragma endregion

#pragma region void MPU9250::updateAccelGyro()
/* Update accelometer and gyro data
Input: /
Output: /
Description: 
	* Read new MPU9250 raw data
	* Convert raw data to calibrated accelometer and gyro measurments
*/
void MPU9250::updateAccelGyro()
{
	int16_t MPU9250Data[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
	readMPU9250Data(MPU9250Data); // INT cleared on any read

								  // Now we'll calculate the accleration value into actual g's
	a[0] = (float)MPU9250Data[0] * aRes - accelBias[0];  // get actual g value, this depends on scale being set
	a[1] = (float)MPU9250Data[1] * aRes - accelBias[1];
	a[2] = (float)MPU9250Data[2] * aRes - accelBias[2];

	// Calculate the gyro value into actual degrees per second
	g[0] = (float)MPU9250Data[4] * gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
	g[1] = (float)MPU9250Data[5] * gRes - gyroBias[1];
	g[2] = (float)MPU9250Data[6] * gRes - gyroBias[2];
}
#pragma endregion

#pragma region void MPU9250::updateMag()
/* Update magnetometer data
Input: /
Output: /
Description: 
	* Read raw magnetometer data
	* Convert raw data to scaled magnetometer readings
*/
void MPU9250::updateMag()
{
	int16_t magCount[3] = { 0, 0, 0 };    // Stores the 16-bit signed magnetometer sensor output
	readMagData(magCount);  // Read the x/y/z adc values

							// Calculate the magnetometer values in milliGauss
							// Include factory calibration per data sheet and user environmental corrections
	m[0] = (float)(magCount[0] * mRes * magCalibration[0] - magBias[0]) * magScale[0];  // get actual magnetometer value, this depends on scale being set
	m[1] = (float)(magCount[1] * mRes * magCalibration[1] - magBias[1]) * magScale[1];
	m[2] = (float)(magCount[2] * mRes * magCalibration[2] - magBias[2]) * magScale[2];
}
#pragma endregion

int16_t MPU9250::getZacc() {

	return((int16_t)(1000 * Acc.z - 1000));
}

float MPU9250::getDt() {

	return(sum_send);
}

void MPU9250::MPU9250sleep() {

	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x3f); // Set sleep mode bit (6), disable all sensors
	delay(100);
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x48); // Set sleep mode bit (6), disable all sensors
	delay(100); // Wait for all registers to reset
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00);
	delay(100);
	//Serial1.print("Sleep!");
}

/* PRIVATE METHODS */

#pragma region bool MPU9250::available()
/* Check if new data is avaliable
Input: /
Output: bool - data avaliable or not
Description: Check if new data is avaliable
*/
bool MPU9250::available()
{
	return (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01);
}
#pragma endregion

#pragma region void MPU9250::getAres()
/* Get accelometer scale
Input: /
Output: /
Description: get accelometer scale and register bit setting
*/
void MPU9250::getAres() {
	switch (Ascale)
	{
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case AFS_2G:
		aRes = 2.0 / 32768.0;
		break;
	case AFS_4G:
		aRes = 4.0 / 32768.0;
		break;
	case AFS_8G:
		aRes = 8.0 / 32768.0;
		break;
	case AFS_16G:
		aRes = 16.0 / 32768.0;
		break;
	}
}
#pragma endregion

#pragma region void MPU9250::getGres()
/* Get gyro scale
Input: /
Output: /
Description: get gyro scale and register bit setting
*/
void MPU9250::getGres() {
	switch (Gscale)
	{
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case GFS_250DPS:
		gRes = 250.0 / 32768.0;
		break;
	case GFS_500DPS:
		gRes = 500.0 / 32768.0;
		break;
	case GFS_1000DPS:
		gRes = 1000.0 / 32768.0;
		break;
	case GFS_2000DPS:
		gRes = 2000.0 / 32768.0;
		break;
	}
}
#pragma endregion

#pragma region void MPU9250::getMres()
/* Get magnetometer scale
Input: /
Output: /
Description: get magnetometer scale and register bit setting
*/
void MPU9250::getMres() {
	switch (Mscale)
	{
		// Possible magnetometer scales (and their register bit settings) are:
		// 14 bit resolution (0) and 16 bit resolution (1)
	case MFS_14BITS:
		mRes = 10.*4912. / 8190.; // Proper scale to return milliGauss
		break;
	case MFS_16BITS:
		mRes = 10.*4912. / 32760.0; // Proper scale to return milliGauss
		break;
	}
}
#pragma endregion

#pragma region void MPU9250::initMPU9250()
/* Initialise MPU9250 accelometer and gyro sensors
Input: /
Output: /
Description: Initialise MPU9250 accelometer and gyro sensors
*/
void MPU9250::initMPU9250()
{
	// wake up device
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
	delay(100); // Wait for all registers to reset 

	// get stable time source
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
	delay(200);

	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	// be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
	writeByte(MPU9250_ADDRESS, MPU_CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
												   // determined inset in CONFIG above

												   // Set gyroscope full scale range
												   // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
														// c = c & ~0xE0; // Clear self-test bits [7:5] 
	c = c & ~0x03; // Clear Fchoice bits [1:0] 
	c = c & ~0x18; // Clear GFS bits [4:3]
	c = c | Gscale << 3; // Set full scale range for the gyro
						 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c); // Write new GYRO_CONFIG value to register

												// Set accelerometer full-scale range configuration
	c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
												 // c = c & ~0xE0; // Clear self-test bits [7:5] 
	c = c & ~0x18;  // Clear AFS bits [4:3]
	c = c | Ascale << 3; // Set full scale range for the accelerometer 
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

												 // Set accelerometer sample rate configuration
												 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
												 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
	c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
	c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

												  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
												  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

												  // Configure Interrupts and Bypass Enable
												  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
												  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
												  // can join the I2C bus and all can be controlled by the Arduino as master
												  //   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);    
	writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x12);  // INT is 50 microsecond pulse and any read to clear  
	writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
	delay(100);
}
#pragma endregion

#pragma region void MPU9250::initAK8963(float * destination)
/* Initialise magnetometer
Input: float[3] - to store magnetometer factory calibration data
Output: /
Description: read factory calibration data and configure magnetometer
*/
void MPU9250::initAK8963(float * destination)
{
	// First extract the factory calibration for each magnetometer axis
	uint8_t rawData[3];  // x/y/z gyro calibration data stored here
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
	delay(10);
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	delay(10);
	readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
	destination[0] = (float)(rawData[0] - 128) / 256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
	destination[1] = (float)(rawData[1] - 128) / 256. + 1.;
	destination[2] = (float)(rawData[2] - 128) / 256. + 1.;
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
	delay(10);
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
	delay(10);
}
#pragma endregion

#pragma region void MPU9250::readMPU9250Data(int16_t * destination)
/* Read raw MPU9250 data
Input: int16_t[7] array to store raw data
Output: /
Description: read 14 raw 8-bit data and convert them to signed 16-bit values. 
*/
void MPU9250::readMPU9250Data(int16_t * destination)
{
	uint8_t rawData[14];  // x/y/z accel register data stored here
	readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 14, &rawData[0]);  // Read the 14 raw data registers into data array
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];
	destination[3] = ((int16_t)rawData[6] << 8) | rawData[7];
	destination[4] = ((int16_t)rawData[8] << 8) | rawData[9];
	destination[5] = ((int16_t)rawData[10] << 8) | rawData[11];
	destination[6] = ((int16_t)rawData[12] << 8) | rawData[13];
}
#pragma endregion

#pragma region void MPU9250::readMagData(int16_t * destination)
/* Read raw magnetometer data
Input: int16_t[3] array to store data
Output: /
Description: read 7 raw 8/bit data and convert them to signed 16-bit value
*/
void MPU9250::readMagData(int16_t * destination)
{
	uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	bool newMagData = (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01);
	if (newMagData == true) { // wait for magnetometer data ready bit to be set
		readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
		uint8_t c = rawData[6]; // End data read by reading ST2 register
		if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
			destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
			destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];  // Data stored as little Endian
			destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
		}
	}
}
#pragma endregion

#pragma region int16_t MPU9250::readTempData()
/* Read temperature data
Input: /
Output: int16_t - temperature value
Description: read raw data from temperature snsor and convert it to 16-bit value.
*/
int16_t MPU9250::readTempData()
{
	uint8_t rawData[2];  // x/y/z gyro register data stored here
	readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
	return ((int16_t)rawData[0] << 8) | rawData[1];  // Turn the MSB and LSB into a 16-bit value
}
#pragma endregion

#pragma region void MPU9250::calibrateMPU9250(float * dest1, float * dest2)
/* Calibrate accelometer and gyro
Input: dest1 - float[3] gyroBias to store gyro bias calibration values
	   dest2 - float[3] accBias to store scc bias calibration values
Description:
	* Accumulate 40 readings of raw data
	* Calculate averages and remove gravity from Z component - sensor must lie flat during calibration!
	* Store values into registers
*/
void MPU9250::calibrateMPU9250(float * dest1, float * dest2)
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	// reset device
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	delay(100);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
	// else use the internal oscillator, bits 2:0 = 001
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
	writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
	delay(200);

	// Configure device for bias calculation
	writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
	writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
	writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
	delay(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	writeByte(MPU9250_ADDRESS, MPU_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyrosensitivity = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

										 // Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO  
	writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

			   // At end of sample accumulation, turn off FIFO sensor read
	writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
		readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
		accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
		accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
		gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
		gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
		gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

		accel_bias[0] += (int32_t)accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t)accel_temp[1];
		accel_bias[2] += (int32_t)accel_temp[2];
		gyro_bias[0] += (int32_t)gyro_temp[0];
		gyro_bias[1] += (int32_t)gyro_temp[1];
		gyro_bias[2] += (int32_t)gyro_temp[2];

	}
	accel_bias[0] /= (int32_t)packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t)packet_count;
	accel_bias[2] /= (int32_t)packet_count;
	gyro_bias[0] /= (int32_t)packet_count;
	gyro_bias[1] /= (int32_t)packet_count;
	gyro_bias[2] /= (int32_t)packet_count;

	if (accel_bias[2] > 0L) { accel_bias[2] -= (int32_t)accelsensitivity; }  // Remove gravity from the z-axis accelerometer bias calculation
	else { accel_bias[2] += (int32_t)accelsensitivity; }

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0] / 4) & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
	data[3] = (-gyro_bias[1] / 4) & 0xFF;
	data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
	data[5] = (-gyro_bias[2] / 4) & 0xFF;

	// Push gyro biases to hardware registers
	writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
	writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
	writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
	writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
	writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
	writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

	// Output scaled gyro biases for display in the main program
	dest1[0] = (float)gyro_bias[0] / (float)gyrosensitivity;
	dest1[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
	dest1[2] = (float)gyro_bias[2] / (float)gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
	readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
	readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
	readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int32_t)(((int16_t)data[0] << 8) | data[1]);

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = { 0, 0, 0 }; // Define array to hold mask bit for each accelerometer bias axis

	for (ii = 0; ii < 3; ii++) {
		if ((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0]) & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1]) & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2]) & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

									 // Apparently this is not working for the acceleration biases in the MPU-9250
									 // Are we handling the temperature correction bit properly?
									 // Push accelerometer biases to hardware registers
									 /*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
									 writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
									 writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
									 writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
									 writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
									 writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
									 */
									 // Output scaled accelerometer biases for display in the main program
	dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
	dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
	dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
}
#pragma endregion

#pragma region void MPU9250::magcalMPU9250(float * dest1, float * dest2)
/* Calibrate magnetometer
Input: dest1 - float[3] magnetometer bias values
	   dest2 - float[3] magnetometer scale values
Output: /
Description: 
	* Print calibration instruction to user
	* Shot ca. 15 s of calibration data
	* Record min and max magnetometer data for all coordinates
	* Preform hard iron correction and store bias
	* Preform soft iron correction, and store scale
*/
void MPU9250::magcalMPU9250(float * dest1, float * dest2)
{
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = { 0, 0, 0 }, mag_scale[3] = { 0, 0, 0 };
	int16_t mag_max[3] = { -32767, -32767, -32767 }, mag_min[3] = { 32767, 32767, 32767 }, mag_temp[3] = { 0, 0, 0 };

	LOG(0, "Mag Calibration: Wave device in a figure eight until done!");
	delay(4000);

	// shoot for ~fifteen seconds of mag data
	if (Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
	if (Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
	for (ii = 0; ii < sample_count; ii++) {
		readMagData(mag_temp);  // Read the mag data   
		for (int jj = 0; jj < 3; jj++) {
			if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}
		if (Mmode == 0x02) delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
		if (Mmode == 0x06) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
	}

	// Get hard iron correction
	mag_bias[0] = (mag_max[0] + mag_min[0]) / 2;  // get average x mag bias in counts
	mag_bias[1] = (mag_max[1] + mag_min[1]) / 2;  // get average y mag bias in counts
	mag_bias[2] = (mag_max[2] + mag_min[2]) / 2;  // get average z mag bias in counts

	dest1[0] = (float)mag_bias[0] * mRes*magCalibration[0];  // save mag biases in G for main program
	dest1[1] = (float)mag_bias[1] * mRes*magCalibration[1];
	dest1[2] = (float)mag_bias[2] * mRes*magCalibration[2];

	// Get soft iron correction estimate
	mag_scale[0] = (mag_max[0] - mag_min[0]) / 2;  // get average x axis max chord length in counts
	mag_scale[1] = (mag_max[1] - mag_min[1]) / 2;  // get average y axis max chord length in counts
	mag_scale[2] = (mag_max[2] - mag_min[2]) / 2;  // get average z axis max chord length in counts

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	dest2[0] = avg_rad / ((float)mag_scale[0]);
	dest2[1] = avg_rad / ((float)mag_scale[1]);
	dest2[2] = avg_rad / ((float)mag_scale[2]);

	LOG(3,"Mag Calibration done!");
}

#pragma endregion

#pragma region void MPU9250::updateRPY()
/* Updare Roll, Pitch and Yaw
Input: /
Output: /
Description: 
	* Calculate rotation coefficients from quaternions
	* Calculate roll, pitch and yaw
	* calculate linear acceleration components (gravitation subtracted)
*/
void MPU9250::updateRPY()
{
	a12 = 2.0f * (Q.x * Q.y + Q.w * Q.z);
	a22 = Q.w * Q.w + Q.x * Q.x - Q.y * Q.y - Q.z * Q.z;
	a31 = 2.0f * (Q.w * Q.x + Q.y * Q.z);
	a32 = 2.0f * (Q.x * Q.z - Q.w * Q.y);
	a33 = Q.w * Q.w - Q.x * Q.x - Q.y * Q.y + Q.z * Q.z;
	pitch = -asinf(a32);
	roll = atan2f(a31, a33);
	yaw = atan2f(a12, a22);
	pitch *= 180.0f / PI;
	yaw *= 180.0f / PI;
	yaw += magnetic_declination; // Declination set for Ljubljana
	if (yaw < 0) yaw += 360.0f; // Ensure yaw stays between 0 and 360
	roll *= 180.0f / PI;

	lin_ax = a[0] + a32;
	lin_ay = a[1] - a31;
	lin_az = a[2] - a33;

}
#pragma endregion

#pragma region void MPU9250::MPU9250SelfTest(float * destination)
/* Self test calibration data
Input: float[6] - self test results -return percent deviation from factory trim values, +/- 14 or less deviation is a pass
Output: /
Description: *Accelerometer and gyroscope self test; check calibration wrt factory settings
*/
void MPU9250::MPU9250SelfTest(float * destination)
{
	uint8_t rawData[6] = { 0, 0, 0, 0, 0, 0 };
	uint8_t selfTest[6];
	int32_t gAvg[3] = { 0 }, aAvg[3] = { 0 }, aSTAvg[3] = { 0 }, gSTAvg[3] = { 0 };
	float factoryTrim[6];
	uint8_t FS = 0;

	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
	writeByte(MPU9250_ADDRESS, MPU_CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, FS << 3);  // Set full scale range for the gyro to 250 dps
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, FS << 3); // Set full scale range for the accelerometer to 2 g

	for (int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer

		readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
		aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

		readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
		gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
	}

	for (int ii = 0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	// Configure the accelerometer for self-test
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	delay(25);  // Delay a while to let the device stabilize

	for (int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer

		readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
		aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

		readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
	}

	for (int ii = 0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);
	delay(25);  // Delay a while to let the device stabilize

				// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
	selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
	selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
	selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
	selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
	selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

																// Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[0] - 1.0))); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[1] - 1.0))); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[2] - 1.0))); // FT[Za] factory trim calculation
	factoryTrim[3] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[5] - 1.0))); // FT[Zg] factory trim calculation

																					  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
																					  // To get percent, must multiply by 100
	for (int i = 0; i < 3; i++) {
		destination[i] = 100.0*((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.;   // Report percent differences
		destination[i + 3] = 100.0*((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.; // Report percent differences
	}

}
#pragma endregion

/* QUATERNION FILTERS */

#pragma region void MPU9250::MadgwickQuaternionUpdate(float ax, float ay, float az, float g[0], float gy, float gz, float mx, float my, float mz)
/* Madgwick Quaternion Update
Input:
* float ax - x acceleration
* float ay - y acceleration
* float az - z acceleration
* float gx - x gyro
* float gy - y gyro
* float gz - z gyro
* float mx - x mag
* float my - y mag
* float mz - z mag
Output: /
Description: update Quaternions.
*/
void MPU9250::MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	float q1 = Q.w, q2 = Q.x, q3 = Q.y, q4 = Q.z;   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrtf(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrtf(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrtf(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f / norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * deltat;
	q2 += qDot2 * deltat;
	q3 += qDot3 * deltat;
	q4 += qDot4 * deltat;
	norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f / norm;
	Q.w = q1 * norm;
	Q.x = q2 * norm;
	Q.y = q3 * norm;
	Q.z = q4 * norm;

}
#pragma endregion

#pragma region void MPU9250::MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
/* Mahony Quaternion Update
Input:
* float ax - x acceleration
* float ay - y acceleration
* float az - z acceleration
* float gx - x gyro
* float gy - y gyro
* float gz - z gyro
* float mx - x mag
* float my - y mag
* float mz - z mag
Output: /
Description: update Quaternions.
*/
void MPU9250::MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	float q1 = Q.w, q2 = Q.x, q3 = Q.y, q4 = Q.z;   // short name local variable for readability
	float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float pa, pb, pc;

	// Auxiliary variables to avoid repeated arithmetic
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrtf(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrtf(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f / norm;        // use reciprocal for division
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
	hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
	bx = sqrtf((hx * hx) + (hy * hy));
	bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

	// Estimated direction of gravity and magnetic field
	vx = 2.0f * (q2q4 - q1q3);
	vy = 2.0f * (q1q2 + q3q4);
	vz = q1q1 - q2q2 - q3q3 + q4q4;
	wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
	wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
	wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

	// Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	if (Ki > 0.0f)
	{
		eInt[0] += ex;      // accumulate integral error
		eInt[1] += ey;
		eInt[2] += ez;
	}
	else
	{
		eInt[0] = 0.0f;     // prevent integral wind up
		eInt[1] = 0.0f;
		eInt[2] = 0.0f;
	}

	// Apply feedback terms
	gx = gx + Kp * ex + Ki * eInt[0];
	gy = gy + Kp * ey + Ki * eInt[1];
	gz = gz + Kp * ez + Ki * eInt[2];

	// Integrate rate of change of quaternion
	pa = q2;
	pb = q3;
	pc = q4;
	q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
	q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
	q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
	q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

	// Normalise quaternion
	norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	norm = 1.0f / norm;
	Q.w = q1 * norm;
	Q.x = q2 * norm;
	Q.y = q3 * norm;
	Q.z = q4 * norm;

}

#pragma endregion

/* WIRE FUNCTIONS */

#pragma region void MPU9250::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
/* Write byte to register
Input: uint8_t address - register adress
	   uint8_t subAddress - slave register address
	   uint8_t data - byte data to write
Output: / 
Description: write data to register. Report commuication errors.
*/
void MPU9250::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{

	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	i2c_err_ = Wire.endTransmission();           // Send the Tx buffer
	if (i2c_err_)
	{
		pirntI2CError();
	}
}
#pragma endregion

#pragma region uint8_t MPU9250::readByte(uint8_t address, uint8_t subAddress)
/* Read byte from register
Input: uint8_t address - register adress
	   uint8_t subAddress - slave register address
Output: uint8_t data - byte data to read
Description: read data from register. Report commuication errors.
*/
uint8_t MPU9250::readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data = 0; // `data` will store the register data
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	i2c_err_ = Wire.endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
	if (i2c_err_)
	{
		pirntI2CError();
	}
	Wire.requestFrom(address, (size_t)1);  // Read one byte from slave register address
	if (Wire.available()) data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}
#pragma endregion

#pragma region void MPU9250::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
/* Read multiple bytes from register
Input: uint8_t address - register adress
	   uint8_t subAddress - slave register address
	   uint8_t cont - number of bytes to read
	   uint8_t * dest - uint8_t[count] array to store read data
Output: /
Description: read data from register. Report commuication errors.
*/
void MPU9250::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	i2c_err_ = Wire.endTransmission(false);  // Send the Tx buffer, but send a restart to keep connection alive
	if (i2c_err_)
	{
		pirntI2CError();
	}
	uint8_t i = 0;
	Wire.requestFrom(address, count);  // Read bytes from slave register address
	while (Wire.available())
	{
		dest[i++] = Wire.read();
	} // Put read results in the Rx buffer
}
#pragma endregion

#pragma region void MPU9250::pirntI2CError()
/* Print I2C communication error
Input: /
Output: /
Description: print i2c error type
*/
void MPU9250::pirntI2CError()
{
	if (i2c_err_ == 7) return; // to avoid stickbreaker-i2c branch's error code
	LOG(0, "I2C ERROR CODE : %d", i2c_err_);
}
#pragma endregion