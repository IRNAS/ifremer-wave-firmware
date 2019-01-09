#include "wave_analyser.h"

#pragma region WaveAnalyser::WaveAnalyser()
/* WaveAnalyser constructor
Input: All default value parameters
* float cutoff_freq - cutoff frequency for the low-pass filter
* float sampling_time - predicted sampling time of the IMU
* int order - order of the low-pass filter, between 1 and 4
* int n_data_array - Length of data array, defines max number of stored values
* int n_grad - distance for the gradient calculation
* int innitial_calibration_delay - milliseconds of initial calculation delay
* int n_w - number of waves to be recorded in single iteration
*/
WaveAnalyser::WaveAnalyser(float cutoff_freq, float sampling_time, int order, int n_data_array, int n_grad, int innitial_calibration_delay, int n_w) {

	A = new MotionArray(n_data_array, n_grad, cutoff_freq, sampling_time, order); //Construct motion array for storing acceleration data
	
	calibration_delay = innitial_calibration_delay; //Set calibration delay
	n_waves = n_w; //Set number of waves to be calculated

#ifdef SD_CARD
	sprintf(filename, "/Log.txt"); //Set filename
#endif // SD_CARD
}
#pragma endregion

#pragma region void WaveAnalyser::init()
/* Initialization
Input: /
Output: /
Description:
* Initialize variables 
*/
void WaveAnalyser::init() {
	
	grad = 0;
	current_grad = 0;
	grad_count = 0;
	wave_max_counter = 0;
	old_offset = 0.0;
	new_offset = 0.0;
	wave_avg = 0.0;
	wave_significant = 0.0;
	period_avg = 0.0;

	//Initialize array classes
	A->Init();

	wait_time = millis(); //Reset wait time
	print_wait_time = 0; //Reset print time
}
#pragma endregion

#pragma region void WaveAnalyser::setup()
/* Setup the system
Input: /
Output: /
Description: initialize class and setup MPU sensor
*/
void WaveAnalyser::setup() {

	mpu.setup(); //Setup MPU sensor
	init(); //Initialize analyser
	
	//Initialize arrays
	for (int i = 0; i < 2 * N_WAVES_MAX; i++) {
		max_idx[i] = 0;
		height[i] = 0.0;
		half_period[i] = 0.0;
	}
	wave_counter = 0;

	//Start SD card and log file
#ifdef SD_CARD
	if (!SD.begin(33)) {
		LOG(0, "SD card Mount Failed!");
	}
	if (!SD.exists(filename)) {
		LOG(1, "Log file does not exist, create new one.");
		logfile = SD.open(filename, FILE_WRITE);
		if (!logfile) {
			LOG(0, "Couldn't create data log file.");
		}
		logfile.println("Log file created.");
	}
	logfile.close();
	logfile = SD.open(filename, FILE_APPEND); //Open for appending
#endif
}
#pragma endregion

#pragma region bool WaveAnalyser::update()
/* Update - get called every loop
Input: /
Output: bool - return true when analysis is completed
Description:
* Update MPU measurement - if new value, true is returned -> proceed
* Check if the initial wait time has passed. During the wait time display seconds left.
* Add new rotated z-acceleration value and time interval to the calculation array
* If calculation array is full, send MPU9250 sensor to sleep and proceed with data analysis
* Check if we have desired number of crests and troughs, if yes analyse size and period. Initialize the class. 
*/
bool WaveAnalyser::update() {

	if (mpu.update()) {

		//Check if waiting period is done
		if (millis() - wait_time > calibration_delay)
		{
			bool full = A->AddElement(mpu.getZacc(), mpu.getDt()); //Add new acceleration value and time interval

			//LOG(1, "%d, %d, %d, %d, %d, %d", mpu.getDt(), mpu.getZacc(), A_raw->GetTimeInterval(), A_raw->UpdateAverage(), A->GetTimeInterval(), grad);
		
			if (full) {
				//mpu.MPU9250sleep();
				LOG(1, "MPU9250 to sleep.");
				bool done = analyseData();
				if (done) {
					mpu.MPU9250sleep();
				}
				return done;
			}
		}
		//Display waiting time in seconds
		else {
			if ((millis() - wait_time) > print_wait_time * 1000)
			{
				print_wait_time++;
				LOG(1, "Wait for: %d", calibration_delay / 1000 - (millis() - wait_time) / 1000);
				if (print_wait_time == calibration_delay/1000 )
				{
					LOG(1, "Log data for ca. 30 s.");
#ifdef SD_CARD
					logfile = SD.open(filename, FILE_APPEND);
					logfile.println("DATA LOG:");
					logfile.close();
#endif
				}
			}
		}
	}

	return false;
}
#pragma endregion

#pragma region bool WaveAnalyser::analyseData()
/* Data analysis
Input: / 
Output: bool - return true if sufficient waves are detected
Description:
* Apply low pass filter to the data
* Analyse gradient and determine min/max points
* Calculate wave heights
* Analyse height data
*/
bool WaveAnalyser::analyseData() {

	LOG(1, "Filtering data...");
#ifdef SD_CARD
	logfile = SD.open(filename, FILE_APPEND);
	for (int i = 0; i < A->N; i++) {
		logfile.println(A->x[i]);
#endif

	A->FilterData(); //Apply low-pass filter to data

#ifdef SD_CARD
	logfile.print("Average dt: ");
	logfile.println(A->dt, 6);
	logfile.close();
#endif // SD_CARD

	LOG(1, "Identifying waves...");
	analyseGradient(); //Analyse gradient and determine min/max points
	calculateWaves(); //Calculate new waves

	return(analyseWaves()); //Analyse wave data
}
#pragma endregion

#pragma region void WaveAnalyser::analyseGradient()
/* Analyse gradients and determine min/max points
Input: /
Output: /
Description: 
* Loop over all data points
* Compute new gradient
* If gradient has changed, update current gradient and reset gradient counter. Store position of the first point with new direction.
* If gradient stayed the same increase gradient counter.
* Check if we have new direction for sufficient number of consecutive points, if yes add new bottom or top.
*/
void WaveAnalyser::analyseGradient() {
	
	//Loop over acceleration points
	for (int i = 0; i < A->N; i++) {

		int new_grad = A->GetGradient(i); //Get new gradient
		
		//Analyse new gradient
		if (grad != new_grad) 
		{
			grad = new_grad; //Update gradient
			grad_count = 0; //Reset gradient counter

			//Store starting idx
			if (grad == 1 || grad == -1) {
				max_idx[wave_max_counter] = i;
			}
		}
		else
		{
			grad_count++; //Increase gradient count
		}

		//Check if new direction can be determined 
		if (grad_count == N_GRAD_COUNT && current_grad != grad) {

			//New bottom
			if (current_grad == -1 || current_grad == 1) {
				//Check if not first local extreme
				if (wave_max_counter > 0) {
					//calculate down wave from last top to new bottom
				}
				//This is first extreme - just store starting point
				else {
					//just update start idx
				}
				LOG(2, "Max point: %d", max_idx[wave_max_counter]);
				wave_max_counter++;
			}
			current_grad = grad;
		}
	}
}
#pragma endregion

#pragma region int16_t WaveAnalyser::calculateOffset(int idx1, int idx2)
/* Calculate offset - i.e. average between two heights
Input: int idx1, int idx2 - indices of two points
Output: int16_t - calculated average, offset
*/
int16_t WaveAnalyser::calculateOffset(int idx1, int idx2) {
	int16_t offset = (A->getElement(idx1) + A->getElement(idx2)) / 2;
	return(offset);
}
#pragma endregion

#pragma region void WaveAnalyser::calculateWaves()
/* Calculate wave heights
Input: /
Output: /
Description: 
* Check if we have at least two local extremes
* Loop over maximas.
* Until sufficient number of waves are analysed, calculate offset of previous and next wave-half.
* Calculate new displacement of half-wave. 
* Calculate new half-period. 
* Increase wave counter.
*/
void WaveAnalyser::calculateWaves() {

	//Check we have some waves to analyse
	if (wave_max_counter <= 1) {
		// no waves detected
	}
	else {
		//At least one whole wave, loop over maximums
		for (int i = 1; i < wave_max_counter; i++) {
			
			//Check if we need more waves
			if (wave_counter < 2 * n_waves) {
				
				//Determine offset
				old_offset = calculateOffset(max_idx[i - 1], max_idx[i]); //Fist offset
				if (i < wave_max_counter - 1) {
					new_offset = calculateOffset(max_idx[i], max_idx[i + 1]);
				}
				else {
					new_offset = old_offset;
				}

				height[wave_counter] = A->CalculateDisplacement(max_idx[i - 1], max_idx[i], old_offset, new_offset); //Calculate new height
				half_period[wave_counter] = A->getHalfPeriod(); //Calculate new period
				wave_counter++; //Increase wave counter
			}
			else {
				break; //stop calculation
			}
		}
	}
}
#pragma endregion

#pragma region bool WaveAnalyser::analyseWaves()
/* Analyse wave heights
Input: /
Output: bool - return true if sufficient number of waves were analysed, or number of max/min points in one round is less than 2 - no waves. 
Description: 
* If sufficient number of waves were detected proceed with analysis.
* Sort heights by size. 
* Calculate average wave height and period. 
* Determine 2/3 of measurements
* Calculate average height of waves in correct range - significant height
*/
bool WaveAnalyser::analyseWaves() {

	//Check if sufficient waves were scanned
	if (wave_counter == 2 * n_waves) {

		sort(); //Sort height
		LOG(1, "Heights: ");
#ifdef SD_CARD
		logfile = SD.open(filename, FILE_APPEND);
		logfile.println("Heights:");
#endif
		for (int i = 0; i < n_waves; i++) {
				wave_avg += height[i];
				period_avg += 2 * half_period[i];
				LOG(1, "%d", (int)(height[i]*100) );
#ifdef SD_CARD
				logfile.println(height[i], 2);
#endif
		}

		wave_avg /= (float)(n_waves);
		period_avg /= (float)n_waves;

		//Significant wave height
		int tmp_count = 0;
		int tmp_end = (int) ((2.0 / 3.0) * (float) n_waves + 1.0);
		for (int i = 0; i < tmp_end; i++) {
			if (height[i] < 10.0 && height[i] < 1.5 * wave_avg) {
				wave_significant += height[i];
				tmp_count++;
			}
		}
		wave_significant /= tmp_count;

		LOG(1, "AVERAGE WAVE H: %d", (int)(wave_avg*100));
		LOG(1, "SIGNIFICANT WAVE H: %d", (int)(wave_significant*100));
		LOG(1, "AVERAGE PERIOD: %d", (int)(period_avg*100));
#ifdef SD_CARD
		logfile.print("AVERAGE WAVE H: ");
		logfile.println(wave_avg, 2);
		logfile.print("SIGNIFICANT WAVE H: ");
		logfile.println(wave_significant, 2);
		logfile.print("AVERAGE PERIOD: ");
		logfile.println(period_avg, 2);
		logfile.close();
#endif
		return true;
	}
	//Else repeat scanning
	else {
		if (wave_max_counter <= 2) {
			//End declare no specific waves
			LOG(1, "Array full, no waves.");
#ifdef SD_CARD
			logfile = SD.open(filename, FILE_APPEND);
			logfile.println("Array full, no waves.");
			logfile.close();
#endif
			return true;
		}
		else {
			//Repeat scanning
			init(); //Initialize
			LOG(1, "Array not full.");
#ifdef SD_CARD
			logfile = SD.open(filename, FILE_APPEND);
			logfile.print("Array not full, number of waves: ");
			logfile.println(wave_counter);
			logfile.close();
#endif // SD_CARD

			return false;
		}

	}
}
#pragma endregion

#pragma region void WaveAnalyser::sort()
/* Sort wave height array by size
Input: /
Output: /
Description: sort from largest to smallest
*/
void WaveAnalyser::sort() {

	float tmp_height[2 * n_waves]; //Construct tmp array
	tmp_height[0] = height[0]; //Set first height in first place
	//Loop over heights
	for (int i = 1; i < 2 * n_waves; i++) {
		tmp_height[i] = 0.0;
		for (int j = 0; j <= i; j++) {
			if (height[i] > tmp_height[j]) {
				//Move elements
				for (int k = i - 1; k >= j; k--) {
					tmp_height[k + 1] = tmp_height[k];
				}
				tmp_height[j] = height[i];
				break;
			}
		}
	}
	//Copy back
	for (int i = 0; i < 2 * n_waves; i++) {
		height[i] = tmp_height[i];
	}
}
#pragma endregion

// SET FUNCTIONS

#pragma region void WaveAnalyser::setCalibrationDelay(int newDelay)
/* Re-set initial calibration delay in millis
* Input: int newDelay - in millis
* Output: /
*/
void WaveAnalyser::setCalibrationDelay(int newDelay) {
	if (newDelay > 0 && newDelay < 200000) {
		calibration_delay = newDelay;
	}
}
#pragma endregion

#pragma region void WaveAnalyser::setNumberOfWaves(int newNumber)
/* Re-set number of waves to calculate
Input: int newNumber
Output: /
*/
void WaveAnalyser::setNumberOfWaves(int newNumber) {
	if (newNumber > 0 && newNumber < N_WAVES_MAX) {
		n_waves = newNumber;
	}
}
#pragma endregion

// GET FUNCTIONS

float WaveAnalyser::getSignificantWave() {
	return wave_significant;
};

float WaveAnalyser::getAverageWave() {
	return wave_avg;
};

float WaveAnalyser::getAveragePeriod() {
	return period_avg;
};
