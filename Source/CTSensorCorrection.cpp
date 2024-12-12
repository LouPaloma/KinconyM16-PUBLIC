/*
 * CTSensor.cpp
 *
 *  Created on: Nov 16, 2024
 */

#include "CTSensorCorrection.h"
#include <esp_task_wdt.h>

TickType_t mutexTicksToWait = 200;

CTSensorCorrection::CTSensorCorrection(short sensorIndex, std::vector<Data> newCorrections) {
	ESP_LOGV("CTSensorCorrection", "Creating new CTSensorCorrection for sensor %d.", sensorIndex);
	if (newCorrections.size() > 1) {
		//TODO check for upper limit?

		if (newCorrections.size() == 2) {
			ESP_LOGD("CTSensorCorrection", "For sensor %i, creating 2 point correction algorithm.", sensorIndex);
			interpolation = nullptr;
			//ensure first expected value is the lower value
			Data firstCorrection = newCorrections.at(0);
			Data secondCorrection = newCorrections.at(1);
			if (firstCorrection.expectedValue < secondCorrection.expectedValue) {
				corrections.push_back(firstCorrection);
				corrections.push_back(secondCorrection);
			} else {
				corrections.push_back(secondCorrection);
				corrections.push_back(firstCorrection);
			}
		} else {
			ESP_LOGD("CTSensorCorrection", "For sensor %i, creating interpolation correction algorithm with %d points.",
					sensorIndex, newCorrections.size());
			createInterpolation(newCorrections);
		}
	} else {
		ESP_LOGI("CTSensorCorrection", "No CTSensorCorrection set for sensor %d", sensorIndex);
	}
}

CTSensorCorrection::~CTSensorCorrection() {
	ESP_LOGD("kincony", "Deallocating child interpolation object...");
	interpolation->~Interpolation();
}

void CTSensorCorrection::updateCorrection(short sensorIndex, short arraySize, Data dataArray[]){
	//acquire the semaphore
	if (xSemaphoreTake(mutex, mutexTicksToWait) == pdTRUE ) {
		//tickle watchdog
		esp_task_wdt_reset();

		if (arraySize == 0) {
			ESP_LOGD("CTSensorCorrection", "For sensor %i, REMOVING all correction algorithms.", sensorIndex);
			corrections.clear();
			if (interpolation != nullptr) {
				interpolation->~Interpolation();
				interpolation = nullptr;
			}
		} else if (arraySize == 2) {
			//create linear, drop interpolation if exists
			ESP_LOGD("CTSensorCorrection", "For sensor %i, RE-creating as 2 point correction algorithm.", sensorIndex);
			corrections.clear();
			corrections.push_back(dataArray[0]);
			corrections.push_back(dataArray[1]);

			if (interpolation != nullptr) {
				interpolation->~Interpolation();
				interpolation = nullptr;
			}
		} else {
			corrections.clear();

			//reverse values
			Data interpolationData[arraySize];
			for (short index = 0 ; index < arraySize ; index++) {
				interpolationData[index] = { dataArray[index].recordedValue, dataArray[index].expectedValue };
			}

			if (interpolation == nullptr) {
				//create new interpolation
				ESP_LOGD("CTSensorCorrection", "For sensor %i, RE-creating as NEW interpolation correction algorithm.", sensorIndex);
				interpolation = new Interpolation(interpolationData, arraySize);
			} else {
				ESP_LOGD("CTSensorCorrection", "For sensor %i, RE-creating as updated interpolation correction algorithm.", sensorIndex);

				//update array size
				interpolation->sizeUpdate(arraySize);

				//copy from tmp array
				interpolation->interpolationUpdate(interpolationData);
			}
		}

		//release the mutex
		xSemaphoreGive(mutex);
	} else {
		ESP_LOGE("CTSensorCorrection", "Unable to acquire mutex for correction update after %d ticks, abandoning...",
				mutexTicksToWait);
	}
}

float CTSensorCorrection::applyCorrections(short sensorIndex, float reading) {
	float result = reading;

	//acquire the semaphore
	if (xSemaphoreTake(mutex, mutexTicksToWait) == pdTRUE ) {
		//tickle watchdog
		esp_task_wdt_reset();

		if ((corrections.empty()) && (interpolation == nullptr)) {
			ESP_LOGD("CTSensorCorrection", "No correction saved, using value as read");
		} else	if (corrections.size() == 2) {
			result = applyTwoPointCorrection(sensorIndex, reading);
		} else {
			result = applyInterpolation(sensorIndex, reading);
		}

		//release the mutex
		xSemaphoreGive(mutex);
	} else {
		ESP_LOGE("CTSensorCorrection", "Unable to acquire mutex for to apply correction after %d ticks, using read value...",
				mutexTicksToWait);
	}

	return result;
}

bool CTSensorCorrection::hasConfiguration() {
	return !corrections.empty() || interpolation != nullptr;
}

size_t CTSensorCorrection::addConfiguration(short correctionIndex, JsonArray* root) {
	if (!hasConfiguration()) {
		return 0;
	}

	JsonObject obj = root->add<JsonObject>();
	obj["sensor"] = correctionIndex;

	unsigned short correctionCount = 0;
	if (!corrections.empty()) {
		//2 point uses corrections
		JsonArray correctionArray = obj["correction"].to<JsonArray>();
		for (auto correction : corrections) {
			correctionCount++;
			JsonObject jsonCorrection = correctionArray.add<JsonObject>();
			jsonCorrection["expectedValue"] = correction.expectedValue;
			jsonCorrection["recordedValue"] = correction.recordedValue;
		}
	} else {
		//else get Interpolation _data_ array
		if (interpolation != nullptr) {
			JsonArray correctionArray = obj["correction"].to<JsonArray>();
			correctionCount = interpolation-> _array_size_;
			for (short arrayIndex = 0 ; arrayIndex < correctionCount ; arrayIndex++) {
				JsonObject jsonCorrection = correctionArray.add<JsonObject>();
				jsonCorrection["expectedValue"] = interpolation->_data_[arrayIndex].expectedValue;
				jsonCorrection["recordedValue"] = interpolation->_data_[arrayIndex].recordedValue;
			}
		}
	}

	return correctionCount;
}

bool CTSensorCorrection::readCorrectionPreference(short sensorIndex, Preferences preference) {
	//need preference begin/end within class, otherwise failures including INVALID_HANDLE
	//TODO would changing argument to Preferences* fix this?
	preference.begin(CORRECTION_PREF, PREFS_RO_MODE);

	String key = generateSensorSizeKey(sensorIndex);
	if (preference.isKey(key.c_str())) {
		//read size of array
		uint16_t arraySize = preference.getUShort(key.c_str());
		if (arraySize < 2) {
			ESP_LOGE("CTSensorCorrection", "Reading sensor correction error from NVM, invalid size saved, key: %s, retrieved size: %d, ignoring...",
					key, arraySize);
		} else {
			Data tmpArray[arraySize];
			String key = generateSensorKey(sensorIndex);
			size_t savedBytes = preference.getBytes(key.c_str(), tmpArray,
					arraySize * sizeof(Data));

			if (savedBytes <= 0) {
				//failure to retrieve records
				ESP_LOGE("CTSensorCorrection", "Unable to retrieve correction data, expecting %d records.",
						arraySize);
			} else {
				ESP_LOGD("CTSensorCorrection", "Read sensor correction size, key: %s for array size %d (%d bytes)",
						key, arraySize, savedBytes );

				for (short counter = 0 ; counter < arraySize ; counter++) {
					ESP_LOGD("CTSensorCorrection","For sensor %d, retrieved from NVM record %d: expected: %.3f, recorded: %.3f",
							sensorIndex, counter, tmpArray[counter].expectedValue,
							tmpArray[counter].recordedValue);
				}
				updateCorrection(sensorIndex, arraySize, tmpArray);
				preference.end();
				return true;
			}
		}
	} else {
		ESP_LOGD("CTSensorCorrection", "No sensor correction exists for key: %s", key);
	}

	preference.end();
	return false;
}

bool CTSensorCorrection::saveCorrectionPreference(short sensorIndex, Preferences preference) {
	if (!hasConfiguration()) {
		ESP_LOGD("CTSensorCorrection", "No correction configuration specified for sensor %d, removing from NVM",
				sensorIndex);

		//need preference begin/end within class, otherwise failures including INVALID_HANDLE
		//TODO would changing argument to Preferences* fix this?
		preference.begin(CORRECTION_PREF, PREFS_RW_MODE);

		//if no config, delete preference if exists
		String key = generateSensorKey(sensorIndex);
		preference.remove(key.c_str());

		key = generateSensorSizeKey(sensorIndex);
		preference.remove(key.c_str());
		preference.end();

		return true;
	}

	//need preference begin/end within class, otherwise failures including INVALID_HANDLE
	//TODO would changing argument to Preferences* fix this?
	preference.begin(CORRECTION_PREF, PREFS_RW_MODE);
	String key = generateSensorKey(sensorIndex);
	if (corrections.empty()) {
		//save interpolation array
		ESP_LOGI("CTSensorCorrection", "Saving new sensor correction, key: %s, for %d points",
				key, interpolation->_array_size_);
		size_t savedBytes = preference.putBytes(key.c_str(), (interpolation->_data_),
				interpolation->_array_size_ * sizeof(Data));

		if (savedBytes > 0) {
			//save size of array
			key = generateSensorSizeKey(sensorIndex);
			ESP_LOGI("CTSensorCorrection", "Saving new sensor correction size, key: %s of size %d (%d bytes)",
					key, interpolation->_array_size_, savedBytes );
			savedBytes = preference.putUShort(key.c_str(), interpolation->_array_size_);

			//if fail, remove data, log message
			if (savedBytes <= 0) {
				ESP_LOGE("CTSensorCorrection", "Sensor correction save to NVM failed, rolling back..");
				preference.remove(generateSensorKey(sensorIndex).c_str());
			} else {
				preference.end();
				return true;
			}
		} else {
			ESP_LOGE("CTSensorCorrection", "Sensor correction DATA save to NVM failed");
		}
	} else {
		//save corrections array
		ESP_LOGI("CTSensorCorrection", "Saving new sensor correction, key: %s, for %d points",
				key, corrections.size());

		//copy vector to array
		Data tmpArray[corrections.size()];
		for (size_t index = 0 ; index < corrections.size() ; index++) {
			tmpArray[index] = { corrections.at(index).recordedValue,
					corrections.at(index).expectedValue };
		}

		size_t savedBytes = preference.putBytes(key.c_str(), tmpArray,
				corrections.size() * sizeof(Data));

		if (savedBytes > 0) {
			//save size of array
			key = generateSensorSizeKey(sensorIndex);
			ESP_LOGI("CTSensorCorrection", "Saving new sensor correction size, key: %s of size %d",
					key, corrections.size());
			savedBytes = preference.putUShort(key.c_str(), corrections.size());

			//if fail, remove data, log message
			if (savedBytes <= 0) {
				ESP_LOGE("CTSensorCorrection", "Sensor correction save to NVM failed, rolling back..");
				preference.remove(generateSensorKey(sensorIndex).c_str());
			} else {
				preference.end();
				return true;
			}
		} else {
			ESP_LOGE("CTSensorCorrection", "Sensor correction DATA save to NVM failed");
		}
	}

	preference.end();
	return false;
}

/*************************************
 *
 *  PRIVATE
 *
 **********************************/

void CTSensorCorrection::createInterpolation(std::vector<Data> corrections) {
	//convert vector to array, reversing values
	Data interpolationData[corrections.size()];
	for (size_t index = 0 ; index < corrections.size() ; index++) {
		interpolationData[index] = { corrections.at(index).recordedValue, corrections.at(index).expectedValue };
	}

	//initialize Interpolation
	interpolation = new Interpolation(interpolationData, corrections.size());
}

float CTSensorCorrection::applyTwoPointCorrection(short sensorIndex, float inputValue) {
	ESP_LOGV("CTSensorCorrection", "Applying 2 point correction for sensor %d.", sensorIndex);
	ESP_LOGV("CTSensorCorrection", "Sensor %i using high reading %.3f, low reading %.3f", sensorIndex,
			corrections.at(1).recordedValue, corrections.at(0).recordedValue);
	ESP_LOGV("CTSensorCorrection", "Sensor %i using high reference %.3f, low reference %.3f", sensorIndex,
			corrections.at(1).expectedValue, corrections.at(0).expectedValue);
	float sensorRange = corrections.at(1).recordedValue - corrections.at(0).recordedValue;
	float referenceRange = corrections.at(1).expectedValue - corrections.at(0).expectedValue;
	ESP_LOGV("CTSensorCorrection", "Sensor %i sensorRange: %.3f, reference range: %.3f",
			sensorIndex, sensorRange, referenceRange);

	if ((sensorRange <= 0) || (referenceRange <= 0)) {
		return inputValue;
	}

	//CorrectedValue = (((RawValue - RawLow) * ReferenceRange) / RawRange) + ReferenceLow
	float correctedValue = (((inputValue - corrections.at(0).recordedValue) * referenceRange)
			/ sensorRange) + corrections.at(0).expectedValue;
	if (correctedValue < 0) {
		correctedValue = 0.0;
	}
	ESP_LOGI("CTSensorCorrection", "Adjusting via (2pt correction) read sensor (%d) value from %.3f to %.3f",
			sensorIndex, inputValue, correctedValue);
	return correctedValue;
}

float CTSensorCorrection::applyInterpolation(short sensorIndex, float inputValue) {
	//LaGrange gives good results with best time
	//	[546505][D][CTSensorCorrection.cpp:203] applyInterpolation(): [CTSensorCorrection] Exponential Fitting: -nan, time: 196 micros
	//	[546516][D][CTSensorCorrection.cpp:208] applyInterpolation(): [CTSensorCorrection] LaGrange Fitting: 13.39, time: 36 micros
	//	[546528][D][CTSensorCorrection.cpp:213] applyInterpolation(): [CTSensorCorrection] Linear Regression Fitting: 13.39, time: 46 micros
	//	[546540][D][CTSensorCorrection.cpp:218] applyInterpolation(): [CTSensorCorrection] Power Fitting: nan, time: 255 micros
	//	[546550][D][CTSensorCorrection.cpp:223] applyInterpolation(): [CTSensorCorrection] Exponential Base: -nan, time: 87 micros
	//	[546561][I][CTSensorCorrection.cpp:235] applyInterpolation(): [CTSensorCorrection] Adjusting via (exponential fitting) read sensor (2) value from 56.116 to 13.392//	unsigned long timerStart = micros();

//	unsigned long timerStart = micros();
//	double result = interpolation->exponentialFitting(inputValue);
//	unsigned long timerStop = micros() - timerStart;
//    ESP_LOGD("CTSensorCorrection", "Exponential Fitting: %.2lf, time: %d micros", result, timerStop);
//
//	timerStart = micros();
//	result = interpolation->lagrange(inputValue);
//	timerStop = micros() - timerStart;
//    ESP_LOGD("CTSensorCorrection", "LaGrange Fitting: %.2f, time: %d micros", result, timerStop);
//
//	timerStart = micros();
//	result = interpolation->linearRegression(inputValue);
//	timerStop = micros() - timerStart;
//    ESP_LOGD("CTSensorCorrection", "Linear Regression Fitting: %.2f, time: %d micros", result, timerStop);
//
//    timerStart = micros();
//	result = interpolation->powerFitting(inputValue);
//	timerStop = micros() - timerStart;
//    ESP_LOGD("CTSensorCorrection", "Power Fitting: %.2f, time: %d micros", result, timerStop);
//
//    timerStart = micros();
//	result = interpolation->getExponentialBase();
//	timerStop = micros() - timerStart;
//    ESP_LOGD("CTSensorCorrection", "Exponential Base: %.2f, time: %d micros", result, timerStop);
//
//	result = interpolation->lagrange(inputValue);
	double result = interpolation->lagrange(inputValue);

	if (result < 0) {
		result = 0;
	}
	ESP_LOGI("CTSensorCorrection", "Adjusting (via LaGrange fitting) read sensor (%d) value from %.3f to %.3f",
			sensorIndex, inputValue, result);
	return result;
}

String CTSensorCorrection::generateSensorKey(short sensorIndex) {
	return "s" + String(sensorIndex);
}

String CTSensorCorrection::generateSensorSizeKey(short sensorIndex) {
	return generateSensorKey(sensorIndex) + "_size";
}
