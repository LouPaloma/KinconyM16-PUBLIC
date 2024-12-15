/****************
 *
 * The basis for this came from:
 *
 * Runtime: https://kincony.com/forum/showthread.php?tid=6148
 * Calibration: https://kincony.com/forum/showthread.php?tid=3089
 *
 */

#include "Kincony.h"

#include <ETH.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <EmonLib.h>
#include <ZMPT101B.h>
#include <U8g2lib.h>
#include <SHT31.h>

#include <ArduinoJson.h>
#include "InitializationException.h"
#include <esp_task_wdt.h>
#include <Preferences.h>

//local network info
#define USING_STATIC_IP
#include <HAConnectInfo.h>
#include "time.h"
#include <esp_bt.h>
#include "CTSensorCorrection.h"

//This can be used to output the date the code was compiled
const char BUILD_STAMP[] = __DATE__ " " __TIME__;

//define core for running tasks -- main() and loop() run on CORE 1
//https://community.hiveeyes.org/t/esp32-multicore-task-scheduling/1554
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif
#define ARDUINO_DEFAULT_TASK_CORE 0

/************* MQTT SETTINGS  **************************/
const char *HOST = "KC868_M16v2";
#define STATE_TOPIC "sensor/power/"
#define STATE_TOPIC_STATUS STATE_TOPIC "status"
#define STATE_TOPIC_POWER STATE_TOPIC "power"
#define STATE_TOPIC_SET STATE_TOPIC "set"
#define STATE_TOPIC_CONFIG STATE_TOPIC "config"

//int ledState = HIGH;
//#define LED_BUILTIN 2
//#define LED_BUILTIN 5

//This can be used to output the date the code was compiled
const char compile_date[] = __DATE__ " " __TIME__;

//NTP time buffer - 50 chars should be enough
char timeStringBuff[50];

/******************************************
 *
 * DEFINE USING WIFI to use WIFI vs ETH - can only use one
 *
 ******************************************/

#define USING_WIFI
#ifdef USING_WIFI

// WiFi configuration
char wifiMac[20];

#else

// Ethernet configuration
#define ETH_ADDR        0
#define ETH_POWER_PIN  -1
#define ETH_MDC_PIN    23
#define ETH_MDIO_PIN   18
#define ETH_TYPE       ETH_PHY_LAN8720
#define ETH_CLK_MODE   ETH_CLOCK_GPIO17_OUT

IPAddress local_ip(10, 0, 0, 149);
IPAddress gateway(10, 0, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(10, 0, 0, 1);

#endif

WiFiClient espClient;
PubSubClient client(espClient);

// Multiplexor pins
#define s0 32
#define s1 33
#define s2 13
#define s3 16
#define IN3 35

//Sensor Objects
//#define SHT31_ADDRESS_1 0x44
SHT31 sht;
unsigned long sht_failures = 0;
unsigned long sht_calls = 0;

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, 5, 4, U8X8_PIN_NONE);
EnergyMonitor emon1;
ZMPT101B voltageSensor1(34, 60.0);
ZMPT101B voltageSensor2(39, 60.0);
ZMPT101B voltageSensor3(36, 60.0);

//#define SENSITIVITY1 501.00f
#define SENSITIVITY1 1013.50f
//#define SENSITIVITY2 500.25f
#define SENSITIVITY2 512.25f
//#define SENSITIVITY3 501.25f
#define SENSITIVITY3 511.25f
float temperature, humidity;
float ctAmps[16];
float ctWatts[16];

//TODO what are these?  needed?
//float p_pv, p_grid, p_load, p_home;
//float l_pv, l_grid, l_load, l_home;
float p_grid;

float voltage_grid;
float voltages[3];
short voltageMap[16] = {2,2,256,2,2,2,3,256,256,3,256,2,256,2,3,256};

Preferences preferences;

//voltage sensor that contribute to the grid - only 2 can be TRUE
bool gridContributor[3] = {false, true, true};

//CT sensors that track grid current
short gridSensors[2] = {9, 11};
CTSensorCorrection* ctSensorCorrection[16];

//Calibrations only
#define CALIBRATION_ACTUAL_CHIP_VOLTAGE 2.5f 	// This shouldn't change
#define CALIBRATION_ACTUAL_LINE_VOLTAGE 122.1f 	// Change this based on actual voltage
#define CALIBRATION_START_VALUE 0.0f
//ROUND 2: for chip voltage calibration
//#define CALIBRATION_START_VALUE 1000.0f
//#define CALIBRATION_STOP_VALUE 1000.0f
#define CALIBRATION_STOP_VALUE 5000.0f
#define CALIBRATION_STEP_VALUE 0.25f
#define CALIBRATION_LINE_TOLERANCE 1.0f
#define CALIBRATION_CHIP_TOLERANCE 0.2f

#define CHECK_BIT(var,pos) (((var)>>(pos)) & 1)

void setup() {
	Serial.begin(115200);

	//TODO this has no effect, only compile time setting works
	esp_log_level_set("*", ESP_LOG_INFO);

	//turn off bluetooth radio
	esp_bt_controller_disable();
	esp_bt_controller_deinit();
	esp_bt_mem_release(ESP_BT_MODE_BTDM);

	pinMode(s0, OUTPUT);
	pinMode(s1, OUTPUT);
	pinMode(s2, OUTPUT);
	pinMode(s3, OUTPUT);
	pinMode(IN3, INPUT);

	u8g2.setI2CAddress(0x3C * 2);
	u8g2.begin();
	u8g2.enableUTF8Print();

	Wire.begin(4, 5);
//	Wire.setClock(100000);
	Wire.setClock(10000);
	Wire.setTimeOut(1000);
	sht.begin();
	sht.read();
	temperature = sht.getFahrenheit();
	humidity = sht.getHumidity();
	int error = sht.getError();
	if (error != 0) {
		uint16_t stat = sht.readStatus();
		ESP_LOGW("kincony", "Temperature sensor Error status: 0x%04x, read status: 0x%04x",
				error, stat);
		logSHT31BitStatus(stat);
	}

	emon1.current(IN3, 34.4820); // ADC_PIN is the pin where SCT013 is connected

	//TODO comment/move to mqtt callback operation -- WOULD REQUIRE SEMAPHORES around task
//	calibrateACSensitivity(1, CALIBRATION_ACTUAL_CHIP_VOLTAGE, CALIBRATION_CHIP_TOLERANCE);
//	calibrateACSensitivity(2, CALIBRATION_ACTUAL_LINE_VOLTAGE, CALIBRATION_LINE_TOLERANCE);
//	calibrateACSensitivity(3, CALIBRATION_ACTUAL_LINE_VOLTAGE, CALIBRATION_LINE_TOLERANCE);

	voltageSensor1.setSensitivity(SENSITIVITY1);
	voltageSensor2.setSensitivity(SENSITIVITY2);
	voltageSensor3.setSensitivity(SENSITIVITY3);

	//////////////////////////////////

	//initialize preference and sensor corrections
	try {
		//TODO try/catch unnecessary, no exception is ever thrown
		intializeSensorCorrections();
	} catch (const std::exception &e) {
		ESP_LOGE("kincony", "Unable to initialize sensor corrections, exception: %s", e.what());
	}

	//initialize grid voltage mapping
	try {
		//TODO try/catch unnecessary, no exception is ever thrown
		initializeGridPreferences();
	} catch (const std::exception &e) {
		ESP_LOGE("kincony", "Unable to initialize Grid NVM, exception: %s", e.what());
	}

	//initialize voltage mapping
	initializeSensorVoltageMap();

	//The chip ID is essentially its MAC address(length: 6 bytes).
	uint64_t chipid = ESP.getEfuseMac();
	ESP_LOGI("kincony", "ESP32 Chip ID = %04X%08X", (uint16_t )(chipid >> 32),
			(uint32_t )chipid); //print High 2 bytes
	//ESP_LOGI("%08X\n",(uint32_t)chipid);//print Low 4bytes.

	try {
		setup_networking();
	} catch (const InitializationException &e) {
		ESP_LOGE("kincony", "WiFi initialization failure after max attempts, rebooting...");

		//reboot
		ESP.restart();
	}

	//init and get the time
	configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
	char *localTime = getLocalTime();
	ESP_LOGI("kincony", "Current time: %s", localTime);

	try {
		setup_mqtt();
	} catch (const InitializationException &e) {
		ESP_LOGW("kincony", "MQTT initialization failure.");
		//not fatal
	}

	xTaskCreatePinnedToCore(
			taskPowerReadings				// Function to implement the task
			,  "Read Power Sensors"	  		// A name just for humans
			,  4096  						// This stack size can be checked & adjusted by reading the Stack Highwater
			,  NULL							// Task input parameter
			,  2  							// Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
			,  NULL							// Task handle.
			,  ARDUINO_DEFAULT_TASK_CORE);

	xTaskCreatePinnedToCore(
			taskStatus						// Function to implement the task
			,  "Read Environment Sensors"	// A name just for humans
			,  4096  						// This stack size can be checked & adjusted by reading the Stack Highwater
			,  NULL							// Task input parameter
			,  2  							// Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
			,  NULL							// Task handle.
			,  ARDUINO_DEFAULT_TASK_CORE);

	xTaskCreatePinnedToCore(
			taskUpdateDisplay				// Function to implement the task
			,  "Update LCD Display"	  		// A name just for humans
			,  2048  						// This stack size can be checked & adjusted by reading the Stack Highwater
			,  NULL							// Task input parameter
			,  2  							// Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
			,  NULL							// Task handle.
			,  ARDUINO_RUNNING_CORE);
}

//unsigned long lastMillis = 0L;
void loop() {
	if (!client.connected()) {
		setup_mqtt();
	}

	client.loop();
}

//Periodic Task
void taskPowerReadings(void *pvParameters) {
	for (;;) {
		voltages[0] = voltageSensor1.getRmsVoltage(); //board power
		voltages[1] = voltageSensor2.getRmsVoltage(); //220 leg
		voltages[2] = voltageSensor3.getRmsVoltage(); //220 leg;
		calculateGridVoltage();

		//tickle watchdog
		esp_task_wdt_reset();

		//read ct sensors
		float measuredCurrent = measureCurrent(LOW, LOW, LOW, LOW);
		ctAmps[0] = ctSensorCorrection[0]->applyCorrections(1, measuredCurrent);

		measuredCurrent = measureCurrent(HIGH, LOW, LOW, LOW);
		ctAmps[1] = ctSensorCorrection[1]->applyCorrections(2, measuredCurrent);

		measuredCurrent = measureCurrent(LOW, HIGH, LOW, LOW);
		ctAmps[2] = ctSensorCorrection[2]->applyCorrections(3, measuredCurrent);

		measuredCurrent = measureCurrent(HIGH, HIGH, LOW, LOW);
		ctAmps[3] = ctSensorCorrection[3]->applyCorrections(4, measuredCurrent);

		measuredCurrent = measureCurrent(LOW, LOW, HIGH, LOW);
		ctAmps[4] = ctSensorCorrection[4]->applyCorrections(5, measuredCurrent);

		measuredCurrent = measureCurrent(HIGH, LOW, HIGH, LOW);
		ctAmps[5] = ctSensorCorrection[5]->applyCorrections(6, measuredCurrent);

		measuredCurrent = measureCurrent(LOW, HIGH, HIGH, LOW);
		ctAmps[6] = ctSensorCorrection[6]->applyCorrections(7, measuredCurrent);

		measuredCurrent = measureCurrent(HIGH, HIGH, HIGH, LOW);
		ctAmps[7] = ctSensorCorrection[7]->applyCorrections(8, measuredCurrent);

		measuredCurrent = measureCurrent(LOW, LOW, LOW, HIGH);
		ctAmps[8] = ctSensorCorrection[8]->applyCorrections(9, measuredCurrent);

		measuredCurrent = measureCurrent(HIGH, LOW, LOW, HIGH);
		ctAmps[9] = ctSensorCorrection[9]->applyCorrections(10, measuredCurrent);

		measuredCurrent = measureCurrent(LOW, HIGH, LOW, HIGH);
		ctAmps[10] = ctSensorCorrection[10]->applyCorrections(11, measuredCurrent);

		measuredCurrent = measureCurrent(HIGH, HIGH, LOW, HIGH);
		ctAmps[11] = ctSensorCorrection[11]->applyCorrections(12, measuredCurrent);

		measuredCurrent = measureCurrent(LOW, LOW, HIGH, HIGH);
		ctAmps[12] = ctSensorCorrection[12]->applyCorrections(13, measuredCurrent);

		measuredCurrent = measureCurrent(HIGH, LOW, HIGH, HIGH);
		ctAmps[13] = ctSensorCorrection[13]->applyCorrections(14, measuredCurrent);

		measuredCurrent = measureCurrent(LOW, HIGH, HIGH, HIGH);
		ctAmps[14] = ctSensorCorrection[14]->applyCorrections(15, measuredCurrent);

		measuredCurrent = measureCurrent(HIGH, HIGH, HIGH, HIGH);
		ctAmps[15] = ctSensorCorrection[15]->applyCorrections(16, measuredCurrent);

		//tickle watchdog
		esp_task_wdt_reset();

		calculatePower();

		//calculate grid
		p_grid = calculateGridPower();

		//FIXME -- what are these?
//		p_pv = ctWatts[2] + ctWatts[3];
//		p_load = ctWatts[4] + ctWatts[5];
//		p_home = ctWatts[6] + ctWatts[7];

//		if (p_pv > p_load) {
//			p_grid = p_grid * -1;
//		}

		//publish readings
		publishPowerReadings();

		//testing only
		ESP_LOGD("kincony", "taskPowerReadings High Water Mark: %d", uxTaskGetStackHighWaterMark(NULL));

		// Delay 2+ second between loops.
		vTaskDelay(2047 / portTICK_PERIOD_MS);
	}
}

//Periodic Task
void taskStatus(void *pvParameters) {
	//Attempting to report status more frequently due to SHT31 read failures
	// caused by conflict with LCD display on I2C
	for (;;) {
		sht_calls++;
		if (sht_calls == 0) {
			//handle rollover
			sht_calls++;
			sht_failures = 0;
		}

		if (sht.isConnected()) {
			sht.read(false);
			temperature = sht.getFahrenheit();
			humidity = sht.getHumidity();


			int error = sht.getError();
			if (error != 0) {
				uint16_t stat = sht.readStatus();
				ESP_LOGW("kincony", "Temperature sensor Error status: 0x%04x, read status: 0x%04x",
						error, stat);
				logSHT31BitStatus(stat);
			}

			publishStatus();
		} else {
			sht_failures++;
			ESP_LOGE("kincony", "Temperature sensor connection failure.");
//			sht.reset(true);
		}

		//testing only
		ESP_LOGD("kincony", "taskStatus High Water Mark: %d", uxTaskGetStackHighWaterMark(NULL));

		// Delay 35+ second between loops.
//		vTaskDelay(600047 / portTICK_PERIOD_MS);
		vTaskDelay(35047 / portTICK_PERIOD_MS);
	}
}

//Periodic Task
void taskUpdateDisplay(void *pvParameters) {
	//Moved from loop() to own task to reduce conflict with SHT31 on I2C
	for (;;) {
		u8g2.firstPage();
		do {
			page1();
		} while (u8g2.nextPage());

		//testing only
		ESP_LOGD("kincony", "taskUpdateDisplay High Water Mark: %d", uxTaskGetStackHighWaterMark(NULL));

		// Delay 60+ second between loops.
		vTaskDelay(60047 / portTICK_PERIOD_MS);
	}
}

void calculateGridVoltage() {
	voltage_grid = 0;
	for (short voltageIndex = 0 ; voltageIndex < 3 ; voltageIndex++) {
		if (gridContributor[voltageIndex]) {
			voltage_grid += voltages[voltageIndex];
		}
	}
}

void calculatePower() {
	for (short sensorIndex = 0 ; sensorIndex < 16 ; sensorIndex++) {
		//lookup voltage mapping for sensor
		if (voltageMap[sensorIndex] == 256) {
			//indicates 220 power, use grid
			ctWatts[sensorIndex] = ctAmps[sensorIndex] * voltage_grid;
		} else {
			ctWatts[sensorIndex] = ctAmps[sensorIndex] * voltages[voltageMap[sensorIndex] - 1];
		}
	}
}

float calculateGridPower() {
	float gridPower = 0;
	for (short sensorId = 0 ; sensorId < 2 ; sensorId++) {
		//if a sensor is ID's, then add power value to grid
		if (gridSensors[sensorId] != -1) {
			gridPower += ctWatts[gridSensors[sensorId]];
		}
	}

	return gridPower;
}

void page1() {
	u8g2.setFont(u8g2_font_timR12_tf);

	u8g2.setCursor(0, 15);
	u8g2.print("Grid: ");
	u8g2.print(p_grid, 2);
	u8g2.print("W");

	u8g2.setCursor(0, 35);
	u8g2.print("Temp: ");
	u8g2.print(temperature);
	u8g2.print(" F");

	u8g2.setCursor(0, 55);
	u8g2.print("Humidity: ");
	u8g2.print(humidity);
	u8g2.print(" %");

//	short highSensor = 0;
//	float highWattage = 0.0;
//	for (short nCounter = 0 ; nCounter < 16 ; nCounter++) {
//		//ignore mains and sub sensors
//		if ((nCounter != 9) && (nCounter != 11) && (nCounter != 15)) {
//			if (ctWatts[nCounter] > highWattage) {
//				highWattage = ctWatts[nCounter];
//				highSensor = nCounter;
//			}
//		}
//	}
//	u8g2.setCursor(0, 35);
//	u8g2.print("High Sensor Read:");
//
//	u8g2.setCursor(0, 55);
//	u8g2.print(++highSensor);
//	u8g2.print(": ");
//	u8g2.print(highWattage, 2);
//	u8g2.print(" W");
}

float measureCurrent(bool s0_state, bool s1_state, bool s2_state, bool s3_state) {
	digitalWrite(s0, s0_state);
	digitalWrite(s1, s1_state);
	digitalWrite(s2, s2_state);
	digitalWrite(s3, s3_state);
	float ct = 0.0;
	if (analogRead(IN3) != 0) {
		ct = emon1.calcIrms(1480);
	}

	return ct;
}

String generateSensorKey(short sensorIndex) {
	return "s" + String(sensorIndex);
}

void intializeSensorCorrections() {

	//HARDCODE values here; must have 16 entries, altho 2nd level vector can be empty
	std::vector<std::vector<Data>> initializationVector = {
			{
					{0, 0.155344},
					{1.67, 7.054},
					{13.5, 56.522}
			},//20a/1V
			{
					{0, 0.125011666666667},
					{1.67, 7.148},
					{13.5, 56.57}
			},//20a/1V
			{
					{0, 0.148305},
					{1.67, 4.876},
					{13.5, 38.329}
			},//30a/1V
			{
					{0, 0.124826666666667},
					{1.67, 7.16933333333333},
					{13.5, 55.654}
			},//20a/1V
			{
					{0, 0.122601666666667},
					{1.67, 7.09566666666667},
					{13.5, 55.955}
			},//20a/1V
			{
					{0, 0.137444333333333},
					{1.67, 2.45},
					{13.5, 19.2126666666667}
			},//60a/1V
			{
					{0, 0.127844666666667},
					{1.67, 7.131},
					{13.5, 56.681}
			},//20a/1V
			{
					{0, 0.120418},
					{1.67, 2.46833333333333},
					{13.5, 19.215}
			},//60a/1V
			{
					{0, 0.159658},
					{1.67, 4.69433333333333},
					{13.5, 38.125}
			},//30a/1V
			{
					{0, 0.120111},
					{1.67, 0.766666666666667},
					{13.5, 6.342}
			},//200a/50ma
			{
					{0, 0.128135},
					{1.67, 4.83166666666667},
					{13.5, 38.277}
			},//30a/1V
			{
					{0, 0.0977443333333333},
					{1.67, 0.742},
					{13.5, 6.252}
			},//200a/50ma
			{
					{0, 0.11101},
					{1.67, 2.45633333333333},
					{13.5, 18.987}
			},//60a/1V
			{
					{0, 0.138905},
					{1.67, 7.204},
					{13.5, 57.127}
			},//20a/1V
			{
					{0, 0.110088},
					{1.67, 4.86266666666667},
					{13.5, 38.477}
			},//30a/1V
			{
					{0, 0.180905},
					{1.67, 2.45566666666667},
					{13.5, 19.436}
			}//60a/1V
	};

	//TODO should validate 16 entries exist
	for (short i = 0; i < 16; i++) {
		ctSensorCorrection[i] = new CTSensorCorrection(i, initializationVector.at(i));
		ctSensorCorrection[i]->readCorrectionPreference(i, preferences);
	}
}

String generateGridKey(short sensorIndex) {
	return "g" + String(sensorIndex);
}

void initializeGridPreferences() {
	if (preferences.begin("GridContribs", PREFS_RO_MODE)) {

		//retrieve voltage contributors from preferences
		for (short i = 0; i < 3 ; i++) {
			String keyStr = generateGridKey(i);
			const char * key = keyStr.c_str();
			ESP_LOGD("kincony", "GridContrib Key String: %s, Const char key: %s", keyStr, key);

			if (preferences.isKey(key)) {
				gridContributor[i] = preferences.getBool(key, false);
				ESP_LOGI("kincony", "Retrieved grid mapping saved for key: %s, mapping %d",
						keyStr, gridContributor[i]);
			} else {
				ESP_LOGD("kincony", "No grid mapping saved for key: %s", key);
			}
		}
		preferences.end();
	}

	//retrieve grid sensor id's from preferences
	if (preferences.begin("GridSensors", PREFS_RO_MODE)) {
		for (short i = 0; i < 2 ; i++) {
			String keyStr = generateGridKey(i);
			const char * key = keyStr.c_str();
			ESP_LOGD("kincony", "GridSensors Key String: %s, Const char key: %s", keyStr, key);

			if (preferences.isKey(key)) {
				gridSensors[i] = preferences.getShort(key);
				ESP_LOGI("kincony", "Retrieved grid sensor id saved for key: %s, sensor %d",
						keyStr, gridSensors[i]);
			} else {
				ESP_LOGD("kincony", "No grid sensor id saved for key: %s", key);
			}
		}
		preferences.end();
	}
}

void saveGridMapping(short voltageSensorIndex, bool contributes) {
	String keyStr = generateGridKey(voltageSensorIndex);
	ESP_LOGD("kincony", "Saving Voltage GridContrib with key String: %s as %d", keyStr, contributes);
	size_t size = preferences.putBool(keyStr.c_str(), contributes);
	ESP_LOGI("kincony", "Saved Voltage GridContrib with key String: %s as %d, wrote %d", keyStr, contributes, size);
}

void saveGridSensors(short mapIndex, short sensorIndex) {
	String keyStr = generateGridKey(mapIndex);
	const char * key = keyStr.c_str();
	ESP_LOGD("kincony", "GridSensors Key String: %s, Const char key: %s", keyStr, key);
	preferences.putShort(key, sensorIndex);
	ESP_LOGI("kincony", "Savdc GridSensors with key String: %s for sensor %d", keyStr, sensorIndex);
}


void initializeSensorVoltageMap() {
	if (preferences.begin("VoltMapping", PREFS_RO_MODE)) {
		//retrieve from preferences
		for (short i = 0; i < 16 ; i++) {
			String keyStr = generateSensorKey(i);
			const char * key = keyStr.c_str();

			if (preferences.isKey(key)) {
				voltageMap[i] = preferences.getShort(key);
				ESP_LOGI("kincony", "Retrieved voltage mapping saved for sensor: %s, mapping %d",
						voltageMap[i]);
			} else {
				ESP_LOGD("kincony", "No voltage mapping saved for key: %s, defaulting to FALSE", key);
			}
		}
		preferences.end();
	}
}

void saveSensorVoltageMapping(short sensorIndex, short voltageSensor) {
	String keyStr = generateSensorKey(sensorIndex);
	const char * key = keyStr.c_str();
	preferences.putShort(key, voltageSensor);
}

void publishPowerReadings() {
	if (!client.connected()) {
		setup_mqtt();
	}

	//Voltage and Current sensors
	/*
	 {
		  "sensor": {
			"id": "KC868_M16v2",
			"time": 10,
			"timeStamp": "2020-01-01T12:12:12-0700",
			"buildDate": "Dec 18 2020 18:49:53",
			"rssi": -58,
			"linkSpeed":1000
		  },
		  "voltages": {
			"ac1": 45.22,
			"ac2": 120.3,
			"ac3": 240,
			"acTotal": 244
		  },
		  "current": {
			"sensor1": 47.3,
			"sensor2": 47.3,
			"sensor3": 47.3,
			"sensor4": 47.3,
			"sensor5": 47.3,
			"sensor6": 47.3,
			"sensor7": 47.3,
			"sensor8": 47.3,
			"sensor9": 47.3,
			"sensor10": 47.3,
			"sensor11": 47.3,
			"sensor12": 47.3,
			"sensor13": 47.3,
			"sensor14": 47.3,
			"sensor15": 47.3,
			"sensor16": 47.3
		  },
		  "wattage": {
			"sensor1": 47.3,
			"sensor2": 47.3,
			"sensor3": 47.3,
			"sensor4": 47.3,
			"sensor5": 47.3,
			"sensor6": 47.3,
			"sensor7": 47.3,
			"sensor8": 47.3,
			"sensor9": 47.3,
			"sensor10": 47.3,
			"sensor11": 47.3,
			"sensor12": 47.3,
			"sensor13": 47.3,
			"sensor14": 47.3,
			"sensor15": 47.3,
			"sensor16": 47.3,
			"grid": 75.9
		  }
		}
	 */

	JsonDocument doc;

	JsonObject sensor = doc["sensor"].to<JsonObject>();
	sensor["id"] = String(HOST);
	sensor["time"] = millis();
	sensor["timeStamp"] = getLocalTime();
	sensor["buildDate"] = BUILD_STAMP;
#ifdef USING_WIFI
	sensor["rssi"] = WiFi.RSSI();
#else
	sensor["linkSpeed"] = ETH.linkSpeed();
#endif

	JsonObject voltagesJson = doc["voltages"].to<JsonObject>();
	voltagesJson["ac1"] = voltages[0];
	voltagesJson["ac2"] = voltages[1];
	voltagesJson["ac3"] = voltages[2];
	voltagesJson["acTotal"] = voltage_grid;

	JsonObject amperage = doc["current"].to<JsonObject>();
	amperage["sensor1"] = ctAmps[0];
	amperage["sensor2"] = ctAmps[1];
	amperage["sensor3"] = ctAmps[2];
	amperage["sensor4"] = ctAmps[3];
	amperage["sensor5"] = ctAmps[4];
	amperage["sensor6"] = ctAmps[5];
	amperage["sensor7"] = ctAmps[6];
	amperage["sensor8"] = ctAmps[7];
	amperage["sensor9"] = ctAmps[8];
	amperage["sensor10"] = ctAmps[9];
	amperage["sensor11"] = ctAmps[10];
	amperage["sensor12"] = ctAmps[11];
	amperage["sensor13"] = ctAmps[12];
	amperage["sensor14"] = ctAmps[13];
	amperage["sensor15"] = ctAmps[14];
	amperage["sensor16"] = ctAmps[15];

	JsonObject wattage = doc["wattage"].to<JsonObject>();
	wattage["sensor1"] = ctWatts[0];
	wattage["sensor2"] = ctWatts[1];
	wattage["sensor3"] = ctWatts[2];
	wattage["sensor4"] = ctWatts[3];
	wattage["sensor5"] = ctWatts[4];
	wattage["sensor6"] = ctWatts[5];
	wattage["sensor7"] = ctWatts[6];
	wattage["sensor8"] = ctWatts[7];
	wattage["sensor9"] = ctWatts[8];
	wattage["sensor10"] = ctWatts[9];
	wattage["sensor11"] = ctWatts[10];
	wattage["sensor12"] = ctWatts[11];
	wattage["sensor13"] = ctWatts[12];
	wattage["sensor14"] = ctWatts[13];
	wattage["sensor15"] = ctWatts[14];
	wattage["sensor16"] = ctWatts[15];

	wattage["grid"] = p_grid;

	doc.shrinkToFit();  // optional
	//build string output
	char buffer[measureJson(doc) + 1];
	serializeJson(doc, buffer, sizeof(buffer));

	ESP_LOGI("kincony", "Outgoing power info (%d bytes): %s", sizeof(buffer), buffer);
	boolean success = client.publish(STATE_TOPIC_POWER, buffer, true);
	if (!success) {
		ESP_LOGW("kincony",
				"Unsuccessful publish: %i; MQTT not connected, client state: %d",
				success, client.state());
		ESP_LOGW("kincony",
				"Unsuccessful publish: %i; MQTT not connected, client state: %d, write error: %d",
				success, client.state(), client.getWriteError());
		ESP_LOGW("kincony", "WiFI status: %d", WiFi.status());
	} else {
		ESP_LOGD("kincony", "Published power info to topic: %s",
				STATE_TOPIC_POWER);
	}
}

#ifdef USING_WIFI

/********************************** START SETUP WIFI*****************************************/
unsigned short setup_networking() {
	ESP_LOGD("kincony", "Initializing WIFI:  Connecting to: %s", ssid);

	WiFi.persistent(false);
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);

	unsigned short connectAttempts = 0;
	if (WiFi.status() != WL_CONNECTED) {
		ESP_LOGI("kincony", "Reconnecting WiFi...");

		while ((connectAttempts < 10) && (WiFi.status() != WL_CONNECTED)) {
			connectAttempts++;
			WiFi.reconnect();
			delay(3000);
			if (WiFi.status() != WL_CONNECTED) {
				ESP_LOGW("kincony", "WiFi initialization failure, attempt: %d",
						connectAttempts);
			}

			//tickle watchdog
			esp_task_wdt_reset();

			//TODO - use exponential backoff?
			//https://github.com/GoogleCloudPlatform/google-cloud-iot-arduino/pull/93/commits/d4462e42018b923ce720c0525bc9896082b6f57d
			//https://kyhsa93.github.io/algorithm/2019/10/15/exponential-backoff.html
		}
	}

	if (WiFi.status() != WL_CONNECTED) {
		ESP_LOGE("kincony", "WiFi initialization failure, status: %d",
				WiFi.status());

//		flickerLED(1, 500);

		throw(InitializationException("WiFi failure", __FUNCTION__));
	} else {
		ESP_LOGI("kincony", "Connected to WiFi: IP Address: %s",
				WiFi.localIP().toString().c_str());
		ESP_LOGI("kincony", "Signal strength: %d", WiFi.RSSI());

	    uint8_t derived_mac_addr[6] = {0};
	    ESP_ERROR_CHECK(esp_read_mac(derived_mac_addr, ESP_MAC_WIFI_STA));
	    char msg[70];
		snprintf(msg, 70, "WiFi MAC: %x:%x:%x:%x:%x:%x, IP Address: %s",
			             derived_mac_addr[0], derived_mac_addr[1], derived_mac_addr[2],
			             derived_mac_addr[3], derived_mac_addr[4], derived_mac_addr[5],
						 WiFi.localIP().toString().c_str());
		ESP_LOGD("kincony", "%s", msg);

		snprintf(wifiMac, 20, "%x:%x:%x:%x:%x:%x",
			             derived_mac_addr[0], derived_mac_addr[1], derived_mac_addr[2],
			             derived_mac_addr[3], derived_mac_addr[4], derived_mac_addr[5]);

		//Flicker the LED on (Note that LOW is the voltage level but actually the LED is on;
		//this is because it is active low on the ESP-01)
//		ledState = HIGH;
//		digitalWrite(LED_BUILTIN, ledState);
//		flickerLED(5, 250);

		//WiFi.setSleep(false);
		WiFi.setAutoReconnect(true);

		return connectAttempts;
	}
}

#else

/********************************** START SETUP WIRED ETH*****************************************/
unsigned short setup_networking() {
	//FIXME replace with DHCP: https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ethernet.html

	//  ETH.begin(ETH_TYPE, ETH_ADDR, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_POWER_PIN, ETH_CLK_MODE); // start with ETH
	ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE,
			ETH_CLK_MODE);
	if (ETH.config(local_ip, gateway, subnet, dns, dns) == false) {
		ESP_LOGE("kincony", "ETH initialization failure, status: %d",
				WiFi.status());

//		flickerLED(1, 500);

		throw(InitializationException("Ethernet failure", __FUNCTION__));
	} else {
		ESP_LOGI("kincony", "Connected to Ethernet: IP Address: %s for MAC: %s",
				ETH.localIP().toString().c_str(), ETH.macAddress());
		ESP_LOGD("kincony", "Ethernet link speed: %d", ETH.linkSpeed());

		//Flicker the LED on (Note that LOW is the voltage level but actually the LED is on;
		//this is because it is active low on the ESP-01)
//		ledState = HIGH;
//		digitalWrite(LED_BUILTIN, ledState);
//		flickerLED(5, 250);
	}

	return ETH.linkSpeed();
}

#endif

void setup_mqtt() {
#ifdef USING_WIFI
	if (WiFi.status() == WL_CONNECTED) {
#else
	if (ETH.linkUp()) {
#endif
		ESP_LOGD("kincony", "Attempting MQTT connection...");
		client.setServer(mqtt_server, mqtt_port);
		client.setBufferSize(4096);

		//Reconnect to Wifi and to MQTT. If Wifi is already connected, then autoconnect doesn't do anything.
		if (client.connect(HOST, mqtt_user, mqtt_pass, nullptr, 0, false,
				nullptr, true)) {
			ESP_LOGD("kincony", "MQTT connected, buffer size: %d", client.getBufferSize());

			//incoming topics
			client.setCallback(mqtt_callback);
			client.subscribe(STATE_TOPIC_SET);
			ESP_LOGI("kincony", "Listening on topic: %s", STATE_TOPIC_SET);

			client.loop();
		} else {
			ESP_LOGW("kincony", "MQTT failed, rc=%d", client.state());
			ESP_LOGD("kincony", " try again in 5 seconds.");

			if (client.state() != MQTT_CONNECTED) {
				ESP_LOGW("kincony", "Network failure: %d", client.state());

				//https://pubsubclient.knolleary.net/api.html#state
				switch (client.state()) {
				case MQTT_CONNECTION_TIMEOUT:
					ESP_LOGW("kincony",
							"MQTT_CONNECTION_TIMEOUT - the server didn't respond within the keepalive time");
					break;
				case MQTT_CONNECTION_LOST:
					ESP_LOGW("kincony",
							"MQTT_CONNECTION_LOST - the network connection was broken");
					break;
				case MQTT_CONNECT_FAILED:
					ESP_LOGW("kincony",
							"MQTT_CONNECT_FAILED - the network connection failed");
					break;
				case MQTT_DISCONNECTED:
					ESP_LOGW("kincony",
							"MQTT_DISCONNECTED - the client is disconnected cleanly");
					break;
				case MQTT_CONNECT_BAD_PROTOCOL:
					ESP_LOGW("kincony",
							"MQTT_CONNECT_BAD_PROTOCOL - the server doesn't support the requested version of MQTT");
					break;
				case MQTT_CONNECT_BAD_CLIENT_ID:
					ESP_LOGW("kincony",
							"MQTT_CONNECT_BAD_CLIENT_ID - the server rejected the client identifier");
					break;
				case MQTT_CONNECT_UNAVAILABLE:
					ESP_LOGW("kincony",
							"MQTT_CONNECT_UNAVAILABLE - the server was unable to accept the connection");
					break;
				case MQTT_CONNECT_BAD_CREDENTIALS:
					ESP_LOGW("kincony",
							"MQTT_CONNECT_BAD_CREDENTIALS - the username/password were rejected");
					break;
				case MQTT_CONNECT_UNAUTHORIZED:
					ESP_LOGW("kincony",
							"MQTT_CONNECT_UNAUTHORIZED - the client was not authorized to connect");
					break;
				}

				throw(InitializationException("MQTT initialization failure",
						__FUNCTION__));
			}

			//No delay - wait 5 seconds before retrying
			//delay(5000);
		}
	}
}

void publishStatus() {
	if (!client.connected()) {
		setup_mqtt();
	}

	//publish status
	/*
	 *
	 {
		 "sensor": {
			 "id": "KC868_M16v2",
			 "time" : 10,
			 "timeStamp": "2020-01-01T12:12:12-0700",
			 "buildDate": "Dec 18 2020 18:49:53",
			 "ipAddress": "192.168.1.1",
			 "macAddress": ""
			 "rssi":-58,
			 "linkSpeed":1000
		 },
		 "environment": {
			"temp": 45.1,
			"humidity": 60.3,
			"failureRate": 0.056
		 }
	 }
	 */

	//https://arduinojson.org/v7/assistant/
	JsonDocument doc;

	JsonObject sensor = doc["sensor"].to<JsonObject>();
	sensor["id"] = String(HOST);
	sensor["time"] = millis();
	sensor["timeStamp"] = getLocalTime();
	sensor["buildDate"] = BUILD_STAMP;

#ifdef USING_WIFI
	sensor["ipAddress"] = WiFi.localIP().toString();
	sensor["macAddress"] = 	 wifiMac;
	sensor["rssi"] = WiFi.RSSI();
#else
	sensor["ipAddress"] = ETH.localIP().toString();
	sensor["macAddress"] = ETH.macAddress();
	sensor["linkSpeed"] = ETH.linkSpeed();
#endif

	JsonObject environment = doc["environment"].to<JsonObject>();
	environment["temp"] = temperature;
	environment["humidity"] = humidity;

	//handle rollover
	if (sht_calls > 0) {
		environment["failureRate"] = sht_failures / (double)sht_calls;
	}

	doc.shrinkToFit();  // optional
	//build string output
	char buffer[measureJson(doc) + 1];
	serializeJson(doc, buffer, sizeof(buffer));

	ESP_LOGI("kincony", "Outgoing status info: %s", buffer);
	boolean success = client.publish(STATE_TOPIC_STATUS, buffer, true);
	if (!success) {
		ESP_LOGW("kincony",
				"Unsuccessful publish: %i; MQTT not connected, client state: %d",
				success, client.state());
		ESP_LOGW("kincony", "WiFI status: %d", WiFi.status());
	} else {
		ESP_LOGD("kincony", "Published startup info to topic: %s", STATE_TOPIC_STATUS);
	}

	doc.clear();
}

/********************************** MQTT CALLBACK*****************************************/
void mqtt_callback(char *topic, byte *payload, unsigned int length) {
	ESP_LOGW("kincony", "MQTT callback on topic %s", topic);

	payload[length] = '\0';
	String strTopic = String((char*) topic);
	if (strTopic == STATE_TOPIC_SET) {
		ESP_LOGI("kincony", "Received incoming configuration payload: %s",
				(char* ) payload);

		/* Deserialize the JSON document -- handle unknown number of sensors, using 2 as baseline
			{
			  "debugLog": true,
			  "publishStatus": true,
			  "reboot": true,
			  "publishConfiguration": true,
			  "gridMapping": [false,true,true],
			  "gridSensors":[6,7],
			  "voltageMapping": [2,3,2,3,2,2,3,2,2,3,3,3,3,3,3,256],
			  "corrections": {
				"clear": true,
				"set": [
				  {
					"sensor": 1,
					"correction": [
						{
							"expectedValue": 0,
							"recordedValue": 0.03,
						},
						{
							"expectedValue": 20,
							"recordedValue": 23.33,
						},
					]
				  },
				  {
					"sensor": 2,
					"correction": [
						{
							"expectedValue": 0,
							"recordedValue": 0.103,
						},
						{
							"expectedValue": 20,
							"recordedValue": 26.37,
						},
					]
				  }
				],
				"delete": [
				  1,
				  4,
				  13
				]
			  }
			}
		 mosquitto_pub -h [HOST] -u [USER] -P [PASSWORD] -t 'sensor/power/set' -m '{"reboot":true}'
		 */

		//https://arduinojson.org/v7/assistant/
		JsonDocument doc;
		try {
			DeserializationError error = deserializeJson(doc, (char*) payload);
			if (error) {
				// parsing failed
				ESP_LOGE("kincony","Incoming message deserializeJson() failed: %s",
						error.c_str());
				return;
			}

			if (doc.containsKey("debugLog")) {
				bool debugLevel = doc["debugLog"];
				//TODO these have no effect
				if (debugLevel) {
					esp_log_level_set("*", ESP_LOG_DEBUG);
				} else {
					esp_log_level_set("*", ESP_LOG_INFO);
				}
			}

			if (doc.containsKey("publishStatus")) {
				bool publishStatusRequest = doc["publishStatus"];
				if (publishStatusRequest) {
					publishStatus();
				}
			}

			if (doc.containsKey("publishConfiguration")) {
				bool isPublishConfiguration = doc["publishConfiguration"];
				if (isPublishConfiguration) {
					publishConfiguration();
				}
			}

			//handle grid voltage contributor definitions
			if (doc.containsKey("gridMapping")) {
				JsonArray gridMapping = doc["gridMapping"];
				if (gridMapping.size() != 3) {
					//log error
					ESP_LOGE("kincony", "Invalid grid mapping count, must be exactly THREE");
				} else {
					preferences.begin("GridContribs", PREFS_RW_MODE);
					for (short i = 0; i < 3 ; i++) {
						bool isContributor = gridMapping[i];
						gridContributor[i] = isContributor;
						saveGridMapping(i, gridContributor[i]);
					}
					preferences.end();
				}
			}

			//handle grid sensor contributor definitions
			if (doc.containsKey("gridSensors")) {
				JsonArray gridSensorArray = doc["gridSensors"];
				short arraySize = gridSensorArray.size();
				if (arraySize > 2) {
					//log error
					ESP_LOGE("kincony", "Invalid grid sensor count, must 2 MAXIMUM");
				} else {
					preferences.begin("GridSensors", PREFS_RW_MODE);
					arraySize--;
					for (short i = 0; i < 2 ; i++) {
						if (i <= arraySize) {
							//assuming 1 based sensor id, adjustment needed
							//allowing 0 to unset values
							if ((gridSensorArray[i] >= 0) && (gridSensorArray[i] < 17)) {
								short sensorIndex = gridSensorArray[i];
								gridSensors[i] = --sensorIndex;
								ESP_LOGI("kincony", "Setting grid sensor identifier: %d",
										gridSensors[i]);
								saveGridSensors(i, gridSensors[i]);
							} else {
								ESP_LOGE("kincony", "Invalid grid sensor identifier: %d",
										gridSensorArray[i]);
							}
						} else {
							//unset any prior setting
							if (gridSensors[i] != -1) {
								ESP_LOGI("kincony", "UNSetting grid sensor identifier #%d", i);
								gridSensors[i] = - 1;
								saveGridSensors(i, -1);
							}
						}
					}
					preferences.end();
				}
			}

			//handle sensor:voltage mappings
			if (doc.containsKey("voltageMappingPref")) {
				JsonArray voltageMapping = doc["voltageMappingPref"];
				if (voltageMapping.size() != 16) {
					//log error
					ESP_LOGE("kincony", "Invalid voltage mapping count, must be exactly SIXTEEN");
				} else {
					preferences.begin("VoltMapping", PREFS_RW_MODE);
					for (short i = 0; i < 16 ; i++) {
						if ((voltageMapping[i] >= 1) && (voltageMapping[i] <= 3)) {
							voltageMap[i] = voltageMapping[i];
							saveSensorVoltageMapping(i, voltageMap[i]);
						} else{
							//log error
							ESP_LOGE("kincony", "Voltage mapping must be between 1 and 3, sensor %d has invalid setting: %d, ignoring.",
									i, voltageMapping[i]);
						}
					}
					preferences.end();
				}
			}

			//extract and store any sensor correction settings
			if (doc.containsKey("corrections")) {
				JsonObject corrections = doc["corrections"];
				if (corrections.containsKey("clear") && (corrections["clear"] == true)) {
					//clear all corrections in NVM
					preferences.begin(CORRECTION_PREF, PREFS_RW_MODE);
					preferences.clear();
					preferences.end();

					//clear struct
					Data emptyCorrection[0];
					for (short i = 0; i < 16; i++) {
						ctSensorCorrection[i]->updateCorrection(i, 0, emptyCorrection);

						//no NVM update needed
					}
				} else if (corrections.containsKey("delete")) {
					//delete listed corrections, no need to delete if clear is called
					Data emptyCorrection[0];
					for (int correctionIndex : corrections["delete"].as<JsonArray>()) {
						if ((correctionIndex < 1) || (correctionIndex > 16)) {
							//log error
							ESP_LOGE("kincony", "Invalid sensor specified for deletion: %d, must be between 1 and 16",
									correctionIndex);
						} else {
							correctionIndex--;
							ctSensorCorrection[correctionIndex]->updateCorrection(
									correctionIndex, 0, emptyCorrection);

							//NVM update
							ctSensorCorrection[correctionIndex]->saveCorrectionPreference(
									correctionIndex, preferences);
						}
					}
				}

				//add settings last
				if (corrections.containsKey("set")) {
					//set new correction values
					for (JsonObject correctionField : corrections["set"].as<JsonArray>()) {
						//JSON id are 1->16
						int sensorId = correctionField["sensor"];
						sensorId--;
						if ((sensorId < 0) || (sensorId > 15)) {
							ESP_LOGE("kincony", "Invalid sensor id specified: %d, ignoring...",
									correctionField["sensor"]);
						} else {
							short arraySize = correctionField["correction"].as<JsonArray>().size();
							Data dataArray[arraySize];
							short correctionValueIndex = 0;
							for (JsonObject correctionValues : correctionField["correction"].as<JsonArray>()) {
								dataArray[correctionValueIndex] = { correctionValues["expectedValue"],
									correctionValues["recordedValue"] };
								correctionValueIndex++;
							}

							ctSensorCorrection[sensorId]->updateCorrection(sensorId, arraySize, dataArray);

							//NVM update
							ctSensorCorrection[sensorId]->saveCorrectionPreference(sensorId, preferences);
						}
					}
				}

				publishConfiguration();
			}

			if (doc.containsKey("reboot")) {
				bool reboot = doc["reboot"];
				if (reboot) {
					ESP_LOGE("kincony","Received REBOOT message, restarting....");

					//pause 2 sec before restarting
					vTaskDelay(2000 / portTICK_PERIOD_MS);

					ESP.restart();
				}
			}
		} catch (const std::exception &e) {
			ESP_LOGE("kincony",
					"Unable to process incoming MQTT message, exception: %s; message: %s",
					e.what(), (char* ) payload);
		}
	} else {
		ESP_LOGE("kincony",
				"Received message on unknown topic: %s, payload: %s... ignoring",
				strTopic, payload);
	}
}

void publishConfiguration() {
	/*
		{
		  "sensor": {
			"id": "KC868_M16v2",
			"time": 10,
			"timeStamp": "2020-01-01T12:12:12-0700",
			"buildDate": "Dec 18 2020 18:49:53",
			"rssi": -58,
			"linkSpeed": 1000
		  },
		  "availableEntries": 554,
		  "corrections": [
			{
			  "sensor": 1,
			  "correction": [
			  	  {
			  	  	  "expectedValue": 0,
			  	  	  "recordedValue": 0.01
			  	  },
			  	  {
			  	  	  "expectedValue": 30,
			  	  	  "recordedValue": 57.01
			  	  },
			  ]
			}
		  ],
		  "gridVoltageMapping": [
			false,
			true,
			true
		  ],
		  "gridSensorMapping": [
			10,
			11
		  ]
		}
	 */

	JsonDocument doc;

	JsonObject sensor = doc["sensor"].to<JsonObject>();
	sensor["id"] = String(HOST);
	sensor["time"] = millis();
	sensor["timeStamp"] = getLocalTime();
	sensor["buildDate"] = BUILD_STAMP;
#ifdef USING_WIFI
	sensor["rssi"] = WiFi.RSSI();
#else
	sensor["linkSpeed"] = ETH.linkSpeed();
#endif

	if (preferences.begin(CORRECTION_PREF, PREFS_RO_MODE)) {
		doc["availableEntries"] = preferences.freeEntries();
		preferences.end();
	}

	JsonArray corrections = doc["corrections"].to<JsonArray>();
	unsigned short correctionIndex = 1;
	for (auto sensorCorrection : ctSensorCorrection) {
		if (sensorCorrection->hasConfiguration()) {
			short count = sensorCorrection->addConfiguration(correctionIndex, &corrections);
			ESP_LOGI("kincony", "Sensor %d added %d configuration fields", correctionIndex,
					count);
			correctionIndex++;
		}
	}

	JsonArray gridVoltageMapping = doc["gridVoltageMapping"].to<JsonArray>();
	for (boolean voltMapEntry : gridContributor) {
		gridVoltageMapping.add(voltMapEntry);
	}

	JsonArray gridSensorMapping = doc["gridSensorMapping"].to<JsonArray>();
	for (short gridMappingEntry : gridSensors) {
		gridSensorMapping.add(gridMappingEntry);
	}

	doc.shrinkToFit();  // optional

	//build string output
	char buffer[measureJson(doc) + 1];
	serializeJson(doc, buffer, sizeof(buffer));
	doc.clear();

	ESP_LOGI("kincony", "Outgoing corrections info (%d bytes): %s", sizeof(buffer), buffer);
	boolean success = client.publish(STATE_TOPIC_CONFIG, buffer, true);
	if (!success) {
		ESP_LOGW("kincony",
				"Unsuccessful publish: %i; MQTT not connected, client state: %d, write error: %d",
				success, client.state(), client.getWriteError());
		ESP_LOGW("kincony", "WiFI status: %d", WiFi.status());
	} else {
		ESP_LOGD("kincony", "Published corrections settings to topic: %s",
				STATE_TOPIC_CONFIG);
	}
}

char* getLocalTime() {
	struct tm timeinfo;

	if (!getLocalTime(&timeinfo)) {
		ESP_LOGE("kincony", "Failed to obtain time from NTP server.");
		//return "unspecified";
		strcpy(timeStringBuff, "unspecified");
	} else {

		//formatting:  http://zetcode.com/articles/cdatetime/
		//strftime(timeStringBuff, sizeof(timeStringBuff), "%A, %B %d %Y %H:%M:%S", &timeinfo);
		//Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
		strftime(timeStringBuff, sizeof(timeStringBuff), "%FT%T%z", &timeinfo);
		//Serial.println(timeStringBuff);
	}

	return timeStringBuff;
}

void logSHT31BitStatus(uint16_t stat) {
  ESP_LOGW("kincony", "SHT31 Checksum status %d", CHECK_BIT(stat,0));
  ESP_LOGW("kincony", "SHT31 Last command status %d", CHECK_BIT(stat,1));
  ESP_LOGW("kincony", "SHT31 Reset detected status %d", CHECK_BIT(stat,4));
  ESP_LOGW("kincony", "SHT31 'T' tracking alert %d", CHECK_BIT(stat,10));
  ESP_LOGW("kincony", "SHT31 'RH' tracking alert %d", CHECK_BIT(stat,11));
  ESP_LOGW("kincony", "SHT31 Heater status %d", CHECK_BIT(stat,13));
  ESP_LOGW("kincony", "SHT31 Alert pending status %d", CHECK_BIT(stat,15));
}

float calibrateACSensitivity(short voltageSensorId, float actualVoltage, float tolerance) {
	ESP_LOGI("calibration", "Calibrating voltage sensor %d, with expected voltage: %.2f",
			voltageSensorId, actualVoltage);

	ZMPT101B* voltageSensor;
	switch (voltageSensorId) {
		case 1:
			voltageSensor = &voltageSensor1;
			break;
		case 2:
			voltageSensor = &voltageSensor2;
			break;
		case 3:
		default:
			voltageSensor = &voltageSensor3;
			break;
	}
	float sensitivityValue = CALIBRATION_START_VALUE;
	voltageSensor->setSensitivity(sensitivityValue);
	float voltageNow = voltageSensor->getRmsVoltage();

	float maxCalibrationVoltage = actualVoltage + tolerance;
	float minCalibrationVoltage = actualVoltage - tolerance;
	ESP_LOGI("calibration", "Start calculate: MAX: %.2f, MIN: %.2f", maxCalibrationVoltage,
			minCalibrationVoltage);

	float minVariance = std::numeric_limits<float>::max();
	float bestCalibrationVoltage = 0, bestCalibrationValue = 0;
	float variance;

	//initial display
	calibrationDisplay(voltageSensorId, CALIBRATION_STOP_VALUE, sensitivityValue,
			bestCalibrationVoltage);

	while (voltageNow > maxCalibrationVoltage|| voltageNow < minCalibrationVoltage) {
		if (sensitivityValue < CALIBRATION_STOP_VALUE) {
			sensitivityValue += CALIBRATION_STEP_VALUE;
			voltageSensor->setSensitivity(sensitivityValue);
			voltageNow = voltageSensor->getRmsVoltage();

			variance = voltageNow <= minCalibrationVoltage ? minCalibrationVoltage - voltageNow
					: voltageNow - maxCalibrationVoltage;
			if (variance < minVariance) {
				bestCalibrationVoltage = voltageNow;
				bestCalibrationValue = sensitivityValue;
				minVariance = variance;
			}
			if (std::fmod(sensitivityValue, 100) == 0) {
				ESP_LOGI("calibration", "%.2f => %.2f (%.2f)", sensitivityValue,
						voltageNow, bestCalibrationVoltage);
				calibrationDisplay(voltageSensorId, CALIBRATION_STOP_VALUE, sensitivityValue,
						bestCalibrationVoltage);
			}
		} else {
			ESP_LOGI("calibration", "FAIL: The sensitivity value cannot be determined, best value found %.2f for sensitivity %.2f",
					bestCalibrationVoltage, bestCalibrationValue);
			return 0;
		}
	}

	ESP_LOGI("calibration","SUCCESS: Closest voltage within tolerance: %.2f, Sensitivity Value: %.2f",
			voltageNow, sensitivityValue);
	return sensitivityValue;
}

void calibrationDisplay(short sensorId, float maxSensitivity, float currentSensitivity,
		float bestFoundVoltage) {
	u8g2.firstPage();
	do {
		u8g2.setFont(u8g2_font_timR10_tf);

		u8g2.setCursor(0, 15);
		u8g2.print("Calib. VSensor: ");
		u8g2.print(sensorId);
		u8g2.print("...");

		u8g2.setCursor(0, 35);
		u8g2.print("Sens: ");
		u8g2.print(currentSensitivity, 2);
		u8g2.print(" (");
		u8g2.print(maxSensitivity, 2);
		u8g2.print(")");

		u8g2.setCursor(0, 55);
		u8g2.print("Voltage: ");
		u8g2.print(bestFoundVoltage);
	} while (u8g2.nextPage());
}
