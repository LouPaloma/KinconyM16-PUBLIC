// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _Kincony_H_
#define _Kincony_H_

#include "Arduino.h"
//add your includes for the project Kincony here

//end of add your includes here


//add your function definitions for the project Kincony here

void setup() ;
char* getLocalTime() ;
void loop() ;
void page1();
String byteToHexString(uint8_t byte);
float measureCurrent(bool s0_state, bool s1_state, bool s2_state, bool s3_state);

void publishPowerReadings();
unsigned short setup_networking();
//void flickerLED(int times, int duration);
void setup_mqtt();
void publishStatus();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
char* getLocalTime();
void intializeSensorCorrections();
void publishConfiguration();
void calculateGridVoltage();
void calculatePower();
void initializeGridPreferences();
void initializeSensorVoltageMap();
String generateSensorKey(short sensorIndex);
String generateGridKey(short sensorIndex);
void saveGridMapping(short voltageSensorIndex, bool contributes);
void saveSensorVoltageMapping(short sensorIndex, short voltageSensor);
float calculateGridPower();
void saveGridSensors(short mapIndex, short sensorIndex);
void taskPowerReadings(void *pvParameters);
void taskStatus(void *pvParameters);
void taskUpdateDisplay(void *pvParameters);
void logSHT31BitStatus(uint16_t stat);

float calibrateACSensitivity(short voltageSensorId, float actualVoltage, float tolerance);
void calibrationDisplay(short sensorId, float maxSensitivity, float currentSensitivity,
		float maxFoundVoltage);


//Do not add code below this line
#endif /* _Kincony_H_ */
