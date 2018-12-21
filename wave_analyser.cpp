#include "wave_analyser.h"

#pragma region WaveAnalyser::WaveAnalyser()
/* WaveAnalyser cosntructor
Input: All defoult value parameters
* float cutoff_freq - cutoff frequency for the low-pass filter
* float sampling_time - predicted sampling time of the IMU
* int order - order of the low-pass filter, between 1 and 4
* int n_data_array - Length of data array, defines max number of stored values
* int n_grad - distance for the gradient calculation
* int innitial_calibration_delay - milliseconds of innitial calculation delay
* int n_w - number of waves to be recorded in single itteration
*/
WaveAnalyser::WaveAnalyser(float cutoff_freq, float sampling_time, int order, int n_data_array, int n_grad, int innitial_calibration_delay, int n_w) {

	A = new MotionArray(n_data_array, n_grad, cutoff_freq, sampling_time, order); //Construct motion array for storing acceleration data
	
	calibration_delay = innitial_calibration_delay; //Set calibration delay
	n_waves = n_w; //Set number of waves to be calculated
	sprintf(filename, "/Log.txt");
}
#pragma endregion

/* Innitialisation
Input: /
Output: /
Description:
* Innitialise variables 
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

	//Initialise array classes
	A->Init();

	wait_time = millis(); //Reset wait time
	print_wait_time = 0; //Reset print time
}

#pragma region void WaveAnalyser::setup()
/* Setup the system
Input: /
Output: /
Description: initialise class and setup MPU sensor
*/
void WaveAnalyser::setup() {

	mpu.setup(); //Setup MPU sensor
	init(); //Initialise analyser
	
	//Initialise arrays
	for (int i = 0; i < 2 * N_WAVES_MAX; i++) {
		max_idx[i] = 0;
		height[i] = 0.0;
		half_period[i] = 0.0;
	}
	wave_counter = 0;

	//Start SD card and log file
	if (!SD.begin(33)) {
		LOG(0, "SD card Mount Failed!");
	}
	if (!SD.exists(filename)) {
		LOG(1, "Log file does not exist, create new one.");
		logfile = SD.open(filename, FILE_WRITE);
		if (!logfile) {
			LOG(0, "Couldnt create data log file.");
		}
		logfile.println("Log file created.");
	}
	logfile.close();
	logfile = SD.open(filename, FILE_APPEND); //Open for appending
}
#pragma endregion

#pragma region
/* Update - get called every loop
Input: /
Output: bool - 
Description:
* Update MPU measurment - if new value, tru is returned -> proceed
* Check if the innitial wait time has passed
* Add new rotated z-acceleration value and time interval to the calculation array
* If calculation array is full, add newxt moving average to moton array, get new gradient and analyse it
* Check if we have desired number of crests and troughs, if yes analyse size and period. Initialise the class. 
*/
bool WaveAnalyser::update() {

	if (mpu.update()) {

		//Check if waiting period is done
		if (millis() - wait_time > calibration_delay)
		{
			bool full = A->AddElement(mpu.getZacc(), mpu.getDt()); //Add new acceleration value and time interval

			//LOG(1, "%d, %d, %d, %d, %d, %d", mpu.getDt(), mpu.getZacc(), A_raw->GetTimeInterval(), A_raw->UpdateAverage(), A->GetTimeInterval(), grad);
		
			if (full) {
				return analyseData();
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
					logfile = SD.open(filename, FILE_APPEND);
					logfile.println("DATA LOG:");
					logfile.close();
				}
			}
		}
	}

	return false;
}
#pragma endregion

bool WaveAnalyser::analyseData() {

	LOG(1, "Filtering data...");
	logfile = SD.open(filename, FILE_APPEND);
	for (int i = 0; i < A->N; i++) {
		logfile.println(A->x[i]);
	}
	A->FilterData(); //Apply low-pass filter to data
	logfile.print("Average dt: ");
	logfile.println(A->dt, 6);
	logfile.close();
	LOG(1, "Identifying waves...");
	analyseGradient(); //Analyse gradient and determine min/max points
	calculateWaves(); //Calculate new waves

	return(analyseWaves()); //Analyse wave data
}

#pragma region void WaveAnalyser::analyseGradient()
/* Analyse gradients and determin min/max points
Input: /
Output: /
Description: 
* Loop over all data points
* Compute new gradient
* If gradient has changed, update current gradient and reset gardient counter. Store position of the first point with new direction
* If gradient stayed the same increase gradient counter.
* Check if we have new direction for sufficient number of consecitive points, if yes add new bottom or top.
*/
void WaveAnalyser::analyseGradient() {
	
	//Loop over acceleration points
	for (int i = 0; i < A->N; i++) {
		int new_grad = A->GetGradient(i); //Get new gradient
		//Analyse new gradient
		if (grad != new_grad) 
		{
			grad = new_grad;
			grad_count = 0;

			//Store starting idx
			if (grad == 1 || grad == -1) {
				max_idx[wave_max_counter] = i;
			}
		}
		else
		{
			grad_count++;
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

int16_t WaveAnalyser::calculateOffset(int idx1, int idx2) {
	int16_t offset = (A->getElement(idx1) + A->getElement(idx2)) / 2;
	return(offset);
}

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

				//Calculate new height
				height[wave_counter] = A->CalculateDisplacement(max_idx[i - 1], max_idx[i], old_offset, new_offset);
				half_period[wave_counter] = A->getHalfPeriod();
				wave_counter++;
			}
			else {
				break; //stop calculation
			}
		}
	}
}

bool WaveAnalyser::analyseWaves() {

	//Check if sufficient waves were scanned
	if (wave_counter == 2 * n_waves) {

		logfile = SD.open(filename, FILE_APPEND);
		sort(); //Sort height
		LOG(1, "Heights: ");
		logfile.println("Heights:");
		for (int i = 0; i < n_waves; i++) {
				wave_avg += height[i];
				period_avg += 2 * half_period[i];
				LOG(1, "%.2f", height[i]);
				logfile.println(height[i], 2);
		}

		wave_avg /= (float)(n_waves);
		period_avg /= (float)n_waves;

		int tmp_count = 0;
		int tmp_end = (int) ((2.0 / 3.0) * (float) n_waves + 1.0);
		for (int i = 0; i < tmp_end; i++) {
			if (height[i] < 10.0 && height[i] < 1.5 * wave_avg) {
				wave_significant += height[i];
				tmp_count++;
			}
		}
		wave_significant /= tmp_count;

		LOG(1, "AVERAGE WAVE H: %.2f", wave_avg);
		LOG(1, "SIGNIFICANT WAVE H: %.2f", wave_significant);
		LOG(1, "AVERAGE PERIOD: %.2f", period_avg);
		logfile.print("AVERAGE WAVE H: ");
		logfile.println(wave_avg, 2);
		logfile.print("SIGNIFICANT WAVE H: ");
		logfile.println(wave_significant, 2);
		logfile.print("AVERAGE PERIOD: ");
		logfile.println(period_avg, 2);
		logfile.close();

		return true;
	}
	//Else repeate scanning
	else {
		if (wave_max_counter <= 2) {
			//End declare no specific waves
			LOG(1, "Array full, no waves.");
			logfile = SD.open(filename, FILE_APPEND);
			logfile.println("Array full, no waves.");
			logfile.close();
			return true;
		}
		else {
			//Repeate scanning
			init();
			LOG(1, "Array not full.");
			logfile = SD.open(filename, FILE_APPEND);
			logfile.print("Array not full, number of waves: ");
			logfile.println(wave_counter);
			logfile.close();
			return false;
		}

	}
}

void WaveAnalyser::sort() {
	float tmp_height[2 * n_waves];
	tmp_height[0] = height[0];
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

	for (int i = 0; i < 2 * n_waves; i++) {
		height[i] = tmp_height[i];
	}
}

void WaveAnalyser::setCalibrationDelay(int newDelay) {
	if (newDelay > 0 && newDelay < 200000) {
		calibration_delay = newDelay;
	}
}

void WaveAnalyser::setNumberOfWaves(int newNumber) {
	if (newNumber > 0 && newNumber < N_WAVES_MAX) {
		n_waves = newNumber;
	}
}

float WaveAnalyser::getSignificantWave() {
	return wave_significant;
};
float WaveAnalyser::getAverageWave() {
	return wave_avg;
};
float WaveAnalyser::getAveragePeriod() {
	return period_avg;
};