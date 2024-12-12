/****************
 *
 * The majority of the math comes from: https://github.com/robcholz/Fitting
 *
 */

#include "Arduino.h"
#include <math.h>
#include "Interpolation.h"

Interpolation::Interpolation(Data data[], int array_size) {
	_array_size_ = array_size;

	ESP_LOGV("Interpolation", "Creating Interpolation for %d points.", _array_size_);
	for (int i = 0; i < _array_size_; i++) {
		_data_[i].expectedValue = data[i].expectedValue;
		_data_[i].recordedValue = data[i].recordedValue;
	}

	linear_availability = false;
}

Interpolation::~Interpolation() {
}

void Interpolation::sizeUpdate(int _array_size) {
	_array_size_ = _array_size;
}

void Interpolation::interpolationUpdate(Data _data[]) {
	for (int i = 0; i < _array_size_; i++) {
		_data_[i].expectedValue = _data[i].expectedValue;
		_data_[i].recordedValue = _data[i].recordedValue;
	}

	linear_availability = false;
}

// Lagrange's formula
double Interpolation::lagrange(double _target) {
	double result = 0;
	for (int i = 0; i < _array_size_; i++) {
		double term = _data_[i].recordedValue;
		for (int j = 0; j < _array_size_; j++) {
			if (j != i)
				term = term * (_target - _data_[j].expectedValue)
						/ (_data_[i].expectedValue - _data_[j].expectedValue);
		}
		result += term;
	}

	return result;
}

double Interpolation::linear(double _target) {
	double maxArrayVal, minArrayVal;
	double sorting_queue_x[_array_size_];
	for (int l = 0; l < _array_size_; l++) {
		sorting_queue_x[l] = _data_[l].expectedValue;
	}

	int key, j;
	for (int i = 1; i < _array_size_; i++) {
		key = sorting_queue_x[i];
		j = i - 1;
		while (j >= 0 && sorting_queue_x[j] > key) {
			sorting_queue_x[j + 1] = sorting_queue_x[j];
			j = j - 1;
		}
		sorting_queue_x[j + 1] = key;
	}

	maxArrayVal = sorting_queue_x[_array_size_ - 1];
	minArrayVal = sorting_queue_x[0];
	if ((_target > minArrayVal) && (_target < maxArrayVal)) {
		for (int k = 0; k < _array_size_ - 1; k++) {
			if ((_target > sorting_queue_x[k])
					&& (_target < sorting_queue_x[k + 1])) {
				indexStruct.x1 = k;
				indexStruct.x2 = k + 1;
			}
		}

		for (int m = 0; m < _array_size_; m++) {
			if (_data_[m].expectedValue == sorting_queue_x[indexStruct.x1]) {
				indexStruct.y1 = m;
			}
			if (_data_[m].expectedValue == sorting_queue_x[indexStruct.x2]) {
				indexStruct.y2 = m;
			}
		}

		linear_availability = DEFINED;
		return ((_data_[indexStruct.y2].recordedValue - _data_[indexStruct.y1].recordedValue)
				/ (sorting_queue_x[indexStruct.x2] - sorting_queue_x[indexStruct.x1]))
				* (_target - sorting_queue_x[indexStruct.x2]) + _data_[indexStruct.y2].recordedValue;
	} else {
		linear_availability = UNDEFINED;
		return NAN;
	}
}

double Interpolation::getLinearDerivative(double _point) {
	double maxArrayVal, minArrayVal;
	double sorting_queue_x[_array_size_];
	for (int l = 0; l < _array_size_; l++) {
		sorting_queue_x[l] = _data_[l].expectedValue;
	}

	int key, j;
	for (int i = 1; i < _array_size_; i++) {
		key = sorting_queue_x[i];
		j = i - 1;
		while (j >= 0 && sorting_queue_x[j] > key) {
			sorting_queue_x[j + 1] = sorting_queue_x[j];
			j = j - 1;
		}
		sorting_queue_x[j + 1] = key;
	}

	maxArrayVal = sorting_queue_x[_array_size_ - 1];
	minArrayVal = sorting_queue_x[0];
	if ((_point > minArrayVal) && (_point < maxArrayVal)) {
		for (int k = 0; k < _array_size_ - 1; k++) {
			if ((_point > sorting_queue_x[k])
					&& (_point < sorting_queue_x[k + 1])) {
				indexStruct.x1 = k;
				indexStruct.x2 = k + 1;
			}
		}

		for (int m = 0; m < _array_size_; m++) {
			if (_data_[m].expectedValue == sorting_queue_x[indexStruct.x1]) {
				indexStruct.y1 = m;
			}
			if (_data_[m].expectedValue == sorting_queue_x[indexStruct.x2]) {
				indexStruct.y2 = m;
			}
		}

		linear_derivative_availability = DEFINED;
		return (_data_[indexStruct.y2].recordedValue - _data_[indexStruct.y1].recordedValue)
				/ (sorting_queue_x[indexStruct.x2] - sorting_queue_x[indexStruct.x1]);
	} else {
		linear_derivative_availability = UNDEFINED;
		return NAN;
	}
}

bool Interpolation::linearAvailable() {
	return linear_availability;
}

bool Interpolation::linearDerivativeAvailable() {
	return linear_derivative_availability;
}

double Interpolation::linearRegression(double _point) {
	linearRegressionStruct.sum_x1 = 0;
	linearRegressionStruct.sum_x2 = 0;
	linearRegressionStruct.sum_x1 = 0;
	linearRegressionStruct.sum_y1 = 0;
	linearRegressionStruct.sum_xy = 0;
	for (int i = 0; i < _array_size_; i++) {
		linearRegressionStruct.sum_x1 = linearRegressionStruct.sum_x1 + _data_[i].expectedValue;
		linearRegressionStruct.sum_x2 = linearRegressionStruct.sum_x2
				+ _data_[i].expectedValue * _data_[i].expectedValue;
		linearRegressionStruct.sum_y1 = linearRegressionStruct.sum_y1 + _data_[i].recordedValue;
		linearRegressionStruct.sum_xy = linearRegressionStruct.sum_xy
				+ _data_[i].expectedValue * _data_[i].recordedValue;
	}

	linearRegressionStruct.coeff = (_array_size_ * linearRegressionStruct.sum_xy
			- linearRegressionStruct.sum_x1 * linearRegressionStruct.sum_y1)
			/ (_array_size_ * linearRegressionStruct.sum_x2
					- linearRegressionStruct.sum_x1 * linearRegressionStruct.sum_x1);
	linearRegressionStruct.constant = (linearRegressionStruct.sum_y1
			- linearRegressionStruct.coeff * linearRegressionStruct.sum_x1) / _array_size_;

	return _point * linearRegressionStruct.coeff + linearRegressionStruct.constant;
}

double Interpolation::getLinearRegressionDerivative() {
	linearRegressionStruct.sum_x1 = 0;
	linearRegressionStruct.sum_x2 = 0;
	linearRegressionStruct.sum_y1 = 0;
	linearRegressionStruct.sum_xy = 0;
	for (int i = 0; i < _array_size_; i++) {
		linearRegressionStruct.sum_x1 = linearRegressionStruct.sum_x1 + _data_[i].expectedValue;
		linearRegressionStruct.sum_x2 = linearRegressionStruct.sum_x2
				+ _data_[i].expectedValue * _data_[i].expectedValue;
		linearRegressionStruct.sum_y1 = linearRegressionStruct.sum_y1 + _data_[i].recordedValue;
		linearRegressionStruct.sum_xy = linearRegressionStruct.sum_xy
				+ _data_[i].expectedValue * _data_[i].recordedValue;
	}

	return (_array_size_ * linearRegressionStruct.sum_xy
			- linearRegressionStruct.sum_x1 * linearRegressionStruct.sum_y1)
			/ (_array_size_ * linearRegressionStruct.sum_x2
					- linearRegressionStruct.sum_x1 * linearRegressionStruct.sum_x1);
}

double Interpolation::exponentialFitting(double _point) {
	exponentialStruct.sum_x1 = 0;
	exponentialStruct.sum_x2 = 0;
	exponentialStruct.sum_y1 = 0;
	exponentialStruct.sum_xy = 0;

	for (int i = 0; i < _array_size_; i++) {
		exponentialStruct.sum_x1 = exponentialStruct.sum_x1 + _data_[i].expectedValue;
		exponentialStruct.sum_x2 = exponentialStruct.sum_x2 + _data_[i].expectedValue * _data_[i].expectedValue;
		exponentialStruct.sum_y1 = exponentialStruct.sum_y1 + log(_data_[i].recordedValue);
		exponentialStruct.sum_xy = exponentialStruct.sum_xy
				+ _data_[i].expectedValue * log(_data_[i].recordedValue);
	}

	exponentialStruct.B = (_array_size_ * exponentialStruct.sum_xy
			- exponentialStruct.sum_x1 * exponentialStruct.sum_y1)
			/ (_array_size_ * exponentialStruct.sum_x2
					- exponentialStruct.sum_x1 * exponentialStruct.sum_x1);
	exponentialStruct.A = (exponentialStruct.sum_y1 - exponentialStruct.B * exponentialStruct.sum_x1)
			/ _array_size_;

	exponentialStruct.coeff = exp(exponentialStruct.A);
	exponentialStruct.base = exp(exponentialStruct.B);

	return exp(exponentialStruct.A) * pow(exp(exponentialStruct.B), _point);
}

double Interpolation::getExponentialBase() {
	exponentialStruct.sum_x1 = 0;
	exponentialStruct.sum_x2 = 0;
	exponentialStruct.sum_y1 = 0;
	exponentialStruct.sum_xy = 0;

	for (int i = 0; i < _array_size_; i++) {
		exponentialStruct.sum_x1 = exponentialStruct.sum_x1 + _data_[i].expectedValue;
		exponentialStruct.sum_x2 = exponentialStruct.sum_x2 + _data_[i].expectedValue * _data_[i].expectedValue;
		exponentialStruct.sum_y1 = exponentialStruct.sum_y1 + log(_data_[i].recordedValue);
		exponentialStruct.sum_xy = exponentialStruct.sum_xy
				+ _data_[i].expectedValue * log(_data_[i].recordedValue);
	}

	exponentialStruct.B = (_array_size_ * exponentialStruct.sum_xy
			- exponentialStruct.sum_x1 * exponentialStruct.sum_y1)
			/ (_array_size_ * exponentialStruct.sum_x2
					- exponentialStruct.sum_x1 * exponentialStruct.sum_x1);

	return exp(exponentialStruct.B);
}

double Interpolation::getExponentialCoeff() {
	exponentialStruct.sum_x1 = 0;
	exponentialStruct.sum_x2 = 0;
	exponentialStruct.sum_y1 = 0;
	exponentialStruct.sum_xy = 0;

	for (int i = 0; i < _array_size_; i++) {
		exponentialStruct.sum_x1 = exponentialStruct.sum_x1 + _data_[i].expectedValue;
		exponentialStruct.sum_x2 = exponentialStruct.sum_x2 + _data_[i].expectedValue * _data_[i].expectedValue;
		exponentialStruct.sum_y1 = exponentialStruct.sum_y1 + log(_data_[i].recordedValue);
		exponentialStruct.sum_xy = exponentialStruct.sum_xy
				+ _data_[i].expectedValue * log(_data_[i].recordedValue);
	}

	exponentialStruct.B = (_array_size_ * exponentialStruct.sum_xy
			- exponentialStruct.sum_x1 * exponentialStruct.sum_y1)
			/ (_array_size_ * exponentialStruct.sum_x2
					- exponentialStruct.sum_x1 * exponentialStruct.sum_x1);
	exponentialStruct.A = (exponentialStruct.sum_y1 - exponentialStruct.B * exponentialStruct.sum_x1)
			/ _array_size_;

	return exp(exponentialStruct.A);
}

double Interpolation::powerFitting(double _point) {
	powerStruct.sum_x1 = 0;
	powerStruct.sum_x2 = 0;
	powerStruct.sum_y1 = 0;
	powerStruct.sum_xy = 0;

	for (int i = 0; i < _array_size_; i++) {
		powerStruct.sum_x1 = powerStruct.sum_x1 + log(_data_[i].expectedValue);
		powerStruct.sum_x2 = powerStruct.sum_x2 + log(_data_[i].expectedValue) * log(_data_[i].expectedValue);
		powerStruct.sum_y1 = powerStruct.sum_y1 + log(_data_[i].recordedValue);
		powerStruct.sum_xy = powerStruct.sum_xy + log(_data_[i].expectedValue) * log(_data_[i].recordedValue);
	}

	powerStruct.power = (_array_size_ * powerStruct.sum_xy - powerStruct.sum_x1 * powerStruct.sum_y1)
			/ (_array_size_ * powerStruct.sum_x2 - powerStruct.sum_x1 * powerStruct.sum_x1);
	powerStruct.A = (powerStruct.sum_y1 - powerStruct.power * powerStruct.sum_x1) / _array_size_;

	powerStruct.coeff = exp(powerStruct.A);

	return powerStruct.coeff * pow(_point, powerStruct.power);
}

double Interpolation::getPowerPower() {
	powerStruct.sum_x1 = 0;
	powerStruct.sum_x2 = 0;
	powerStruct.sum_y1 = 0;
	powerStruct.sum_xy = 0;

	for (int i = 0; i < _array_size_; i++) {
		powerStruct.sum_x1 = powerStruct.sum_x1 + log(_data_[i].expectedValue);
		powerStruct.sum_x2 = powerStruct.sum_x2 + log(_data_[i].expectedValue) * log(_data_[i].expectedValue);
		powerStruct.sum_y1 = powerStruct.sum_y1 + log(_data_[i].recordedValue);
		powerStruct.sum_xy = powerStruct.sum_xy + log(_data_[i].expectedValue) * log(_data_[i].recordedValue);
	}

	return (_array_size_ * powerStruct.sum_xy - powerStruct.sum_x1 * powerStruct.sum_y1)
			/ (_array_size_ * powerStruct.sum_x2 - powerStruct.sum_x1 * powerStruct.sum_x1);
}

double Interpolation::getPowerCoeff() {
	powerStruct.sum_x1 = 0;
	powerStruct.sum_x2 = 0;
	powerStruct.sum_y1 = 0;
	powerStruct.sum_xy = 0;

	for (int i = 0; i < _array_size_; i++) {
		powerStruct.sum_x1 = powerStruct.sum_x1 + log(_data_[i].expectedValue);
		powerStruct.sum_x2 = powerStruct.sum_x2 + log(_data_[i].expectedValue) * log(_data_[i].expectedValue);
		powerStruct.sum_y1 = powerStruct.sum_y1 + log(_data_[i].recordedValue);
		powerStruct.sum_xy = powerStruct.sum_xy + log(_data_[i].expectedValue) * log(_data_[i].recordedValue);
	}

	powerStruct.power = (_array_size_ * powerStruct.sum_xy - powerStruct.sum_x1 * powerStruct.sum_y1)
			/ (_array_size_ * powerStruct.sum_x2 - powerStruct.sum_x1 * powerStruct.sum_x1);

	return exp((powerStruct.sum_y1 - powerStruct.power * powerStruct.sum_x1) / _array_size_);
}
