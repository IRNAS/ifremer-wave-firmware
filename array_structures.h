/* ARRAY STRUCTURES classes - includes description of three array classes:
* MOTION ARRAY - used to store acceleration data and analyse it, used in the wave_analyser.h library
* QUATERNION - used for quarternion manipulation in the MPU9250.h library
* VECTOR FLOAT - used in quaternion class
*/

#ifndef _ARRAY_STRUCTURES_H_
#define _ARRAY_STRUCTURES_H_

#include <math.h>
#include "filters.h" //Library including low pass filter
#include "debug_print.h" //Additional library for debug logging

#define GRAV_CONSTANT 9.80665

class MotionArray {
public:

	int16_t *x; //Variable size array
	float dt; //Time interval
	float d_last = 0.0;

	Filter *filter; //Low pass filter

	int N; //Length of array
	int N_gradient; //Length of gradient calculation
	int n_elements = 0; //Number of elements in the array
	int pos = 0; //Position of next element to add

	float half_period = 0.0; //Current half wave period

#pragma region MotionArray(int n, int n_grad)
	/* Construct motion array
	Input: int n - length of motion array, int n_grad - number of points used in gradient calculation
	Output: /
	Description:
	* Check that gradient calculation points are not to big, relative to array size
	* Construct measurment (int16_t) and delta time (uint32_t) array of length n
	* Initialise arrays to 0
	*/
	MotionArray(int n, int n_grad, float cutoff_freq, float sampling_time, int order) {
		N = n;
		N_gradient = n_grad;
		if (N_gradient > (int)(N / 5)) { N_gradient = (int)(N / 5); } //Define the range of gradient


		x = (int16_t *)malloc((N)*sizeof(int16_t));
		dt = 0.0;
		for (int i = 1; i < N; i++)
		{
			x[i] = 0;
		}

		//Initialise filter
		IIR::ORDER  ord; //Low pass filter order
		if (order == 1) { ord = IIR::ORDER::OD1; }
		else if (order == 2) { ord = IIR::ORDER::OD2; }
		else if (order == 3) { ord = IIR::ORDER::OD3; }
		else if (order == 4) { ord = IIR::ORDER::OD4; }
		else { ord = IIR::ORDER::OD3; }

		filter = new Filter(cutoff_freq, sampling_time, ord); //Define low-pass filter
	}
#pragma endregion

#pragma region void Init()
	/* Initialisation
	Input: /
	Output: /
	Description: initialise arrays and counters
	*/
	void Init() {
		for (int i = 1; i < N; i++)
		{
			x[i] = 0;
		}
		d_last = 0.0;
		pos = 0;
		dt = 0.0;
		n_elements = 0;
		filter->init();
	}
#pragma endregion

#pragma region int AddElement(int16_t _x, uint32_t _dt)
	/* Add new element
	Input: int16_t _x - new acceleration, uint32_t _dt - new delta time interval
	Output: int - gradient of calculation 
	Description: 
	* Add acceleration and time interval
	* Update adding position, number of elements and calculation position
	* Call function for gradient calculation and return gradient
	*/
	//Add new element - return gradient of calculation elemnt
	bool AddElement(int16_t _x, float _dt) {
		
		x[pos] = _x; //Add element at the next position
		dt += _dt;
		pos = (pos + 1) % N;

		n_elements++; //Update number of elements
		if (n_elements == N)
		{
			return true;
		}
		else {
			return false;
		}
	}
#pragma endregion

#pragma region void FilterData()
	/* Low pass filter on data
	Input: /
	Output: /
	Description: 
	* Calculate average period
	* Apply filter. 
	*/
	void FilterData() {

		pos = 0;
		dt /= N; //Calculate average period
		LOG(1, "DT: %.6f", dt);
		//Adjust filter sampling time?
		for (int i = 0; i < N; i++) {
			int16_t tmp = x[i];
			x[i] = (int16_t) filter->filterIn((float) x[i]);
			LOG(2, ", %.6f, %d, %d", dt, tmp, x[i]);
		}
	}
#pragma endregion

#pragma region float CalculateDisplacement(int end)
	/* Calculate displacement
	Input: int end - end position for calculation
	Output: float - total displacement
	Description: 
	* Integrate twice from pos_first to given end to get total displacement
	* Update end position
	* Return absulute displacement in m
	*/
	float CalculateDisplacement(int start, int end, int16_t offset1, int16_t offset2) {

		float v = 0.0; //Zero velocity at top and bottom
		float d = 0.0; //Zero displacement at top and bottom

		//Loop til end idex
		for (int i = start; i <= end; i++) {

			float relative_pos = (float)(i - start) / (float)(end - start);
			float offset = ((1.0 - relative_pos) * (float)offset1 + relative_pos * (float)offset2);
			v += dt * (((float)x[i] - offset) * GRAV_CONSTANT / 1000.0f); //Update velocity
			d += dt * v; //Update displacement
			d_last += dt * v;
		}
		half_period = dt * (float)(end - start); //Update half period

		return(abs(d)); //Return absolute displacement
	}
#pragma endregion 

#pragma region float getHalfPeriod()
	float getHalfPeriod() {
		return(half_period);
	}
#pragma endregion

#pragma region int16_t getElement(int i)
	int16_t getElement(int i) {
		return x[i];
	}
#pragma endregion

#pragma region int GetGradient()
	/* Get gradient for current calculation point
	Input: int i - calculation position
	Output: int gradient - return negative, positive or 0 direction of the calculation position
	Description: 
	* Determine first and second point for calculationg gradient, optimally N_gradient positions from calculation point
	* Calculate non-normalised gradient
	* Return only direction (-1, 0, 1)
	*/
	int GetGradient(int i) {

		//Determine positions of gradient calculation
		int i_min = max(0, i - N_gradient); //Position of the first element
		int	i_max = min(N - 1, i + N_gradient); //position of the second element
		float grad = (float)(x[i_max] - x[i_min]); //Gradient calculation (non-scaled!)

		//Return gradient
		if (grad > 0.0) {
			return 1; //Positive
		}
		else if (grad < 0.0) {
			return -1; //Negative
		}
		else {
			return 0; //Zero
		}
	}
#pragma endregion

};

class Quaternion {
public:
	float w;
	float x;
	float y;
	float z;

	Quaternion() {
		w = 1.0f;
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
	}

	Quaternion(float nw, float nx, float ny, float nz) {
		w = nw;
		x = nx;
		y = ny;
		z = nz;
	}

	Quaternion getProduct(Quaternion q) {
		// Quaternion multiplication is defined by:
		//     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
		//     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
		//     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
		//     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
		return Quaternion(
			w*q.w - x*q.x - y*q.y - z*q.z,  // new w
			w*q.x + x*q.w + y*q.z - z*q.y,  // new x
			w*q.y - x*q.z + y*q.w + z*q.x,  // new y
			w*q.z + x*q.y - y*q.x + z*q.w); // new z
	}

	Quaternion getConjugate() {
		return Quaternion(w, -x, -y, -z);
	}

	float getMagnitude() {
		return sqrt(w*w + x*x + y*y + z*z);
	}

	void normalize() {
		float m = getMagnitude();
		w /= m;
		x /= m;
		y /= m;
		z /= m;
	}

	Quaternion getNormalized() {
		Quaternion r(w, x, y, z);
		r.normalize();
		return r;
	}
};

class VectorFloat {
public:
	float x;
	float y;
	float z;

	VectorFloat() {
		x = 0;
		y = 0;
		z = 0;
	}

	VectorFloat(float nx, float ny, float nz) {
		x = nx;
		y = ny;
		z = nz;
	}

	float getMagnitude() {
		return sqrt(x*x + y*y + z*z);
	}

	void normalize() {
		float m = getMagnitude();
		x /= m;
		y /= m;
		z /= m;
	}

	VectorFloat getNormalized() {
		VectorFloat r(x, y, z);
		r.normalize();
		return r;
	}

	void rotate(Quaternion *q) {
		Quaternion p(0, x, y, z);

		// quaternion multiplication: q * p, stored back in p
		p = q->getProduct(p);

		// quaternion multiplication: p * conj(q), stored back in p
		p = p.getProduct(q->getConjugate());

		// p quaternion is now [0, x', y', z']
		x = p.x;
		y = p.y;
		z = p.z;
	}

	VectorFloat getRotated(Quaternion *q) {
		VectorFloat r(x, y, z);
		r.rotate(q);
		return r;
	}
};

#endif 