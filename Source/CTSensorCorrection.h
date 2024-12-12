/*
 * CTSensor.h
 *
 *  Created on: Nov 16, 2024
 */

#ifndef CTSENSORCORRECTION_H_
#define CTSENSORCORRECTION_H_

#include <vector>
#include <Arduino.h>
#include <Preferences.h>
#include "Interpolation.h"
#include <ArduinoJson.h>

#define CORRECTION_PREF "Corrections"
#define PREFS_RW_MODE false
#define PREFS_RO_MODE true

class CTSensorCorrection {
	private:

		std::vector<Data> corrections;
		Interpolation* interpolation;
		SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

		void createInterpolation(std::vector<Data> corrections);
		float applyTwoPointCorrection(short sensorIndex, float inputValue);
		float applyInterpolation(short sensorIndex, float inputValue);
		String generateSensorKey(short sensorIndex);
		String generateSensorSizeKey(short sensorIndex);

	public:
		CTSensorCorrection(short sensorIndex, std::vector<Data> newCorrections);
//		virtual ~CTSensorCorrection() = default;
		virtual ~CTSensorCorrection();
		void updateCorrection(short sensorIndex, short arraySize, Data dataArray[]);

		float applyCorrections(short sensorIndex, float reading);
		bool hasConfiguration();

		size_t addConfiguration(short correctionIndex, JsonArray* root);
		bool readCorrectionPreference(short sensorIndex, Preferences preference);
		bool saveCorrectionPreference(short sensorIndex, Preferences preference);
};

#endif /* CTSENSORCORRECTION_H_ */
