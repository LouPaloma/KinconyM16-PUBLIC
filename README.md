# Kincony KC868_M16v2 Power Monitor

This application will read CT sensors on the Kincony KC868_M16v2 board and calculate power usage.  It allows 
per sensor configuration and sensor correction algorithms.  Periodic MQTT messages are emitted with various 
readings and configurations can be set via MQTT messages and saved into non-volatile memory.

- [Kincony KC868_M16v2 Power Monitor](#kincony-kc868_m16v2-power-monitor)
- [Credits](#credits)
- [Features](#features)
   * [Sensor Corrections](#sensor-corrections)
   * [Sensor to Grid Mappings](#sensor-to-grid-mappings)
   * [NTP time lookup](#ntp-time-lookup)
   * [Voltage Sensitivity Calculation](#voltage-sensitivity-calculation)
- [MQTT Output Messages](#mqtt-output-messages)
   * [Status](#status)
   * [Power](#power)
   * [Configuration](#configuration)
- [MQTT Input Messages](#mqtt-input-messages)
   * [Grid Mapping](#grid-mapping)
   * [Grid Sensors](#grid-sensors)
   * [Voltage Mapping](#voltage-mapping)
   * [Sensor Corrections](#sensor-corrections-1)
- [Home Assistant](#home-assistant)
   * [MQTT Sensors](#mqtt-sensors)
   * [Influx Sensors](#influx-sensors)
- [Build](#build)
- [3D Print Cover](#3d-print-cover)
- [Notes/TODO](#notestodo)

# Credits
The basis for this came from:
- This Runtime: https://kincony.com/forum/showthread.php?tid=6148
- Calibration: https://kincony.com/forum/showthread.php?tid=3089
- Interpolation algorithms: https://github.com/robcholz/Fitting

# Features

## Sensor Corrections
Each sensor can support up to 5 mappings of expected:actual values allowing for curve fitting of read amperage.
These can be configured at compile time or via an MQTT message.  The MQTT message configuration will be saved 
to NVM and will override any compile time configurations even after reboot.  NVM stored configurations can be
removed via MQTT which will remove any curve fitting until the next reboot, when compile time configurations
will be reloaded.

## Sensor to Grid Mappings
The M16 supports 3 voltage sensors. Each CT sensor may be mapped to any (or 2) of the voltage sensors.  This
mapping will be used to determine power (wattage) consumed for that sensor at that instance.

Also, 2 of the 3 voltage sensors may be mapped as providing full grid voltage, e.g., separate left and right
leg of a 110/220 system.  Using the mapping above, a CT sensor can be tied to 2 voltages, allowing for 220V
calculations.

Both the grid mapping and the CT:voltage mapping can be configured either at compile time or via MQTT configuration, 
the latter of which will be stored in NVM.

## NTP time lookup
The system uses the Arduino time library to look up NTP time via timeservers for accurate time postings

## Voltage Sensitivity Calculation
Via a compile time change (uncomment code), sensitivity on voltage sensors can be calculated with results
displayed in both the logs and on the LCD.  Note that this will defer booting to sensor readings until
the calculation(s) are complete and that the calculated values will NOT be retained.  This is meant as a
pre-deployment feature which calculated values being updated in the code.

---

# MQTT Output Messages
MQTT messages are generated by 2 separate periodic task or on demand via an incoming MQTT request.

## Status
A status message is published at startup, every 5 minutes or on demand via an incoming MQTT request.  The
message will be published on the `sensor/power/status` topic.
```json
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
			"humidity": 60.3
		 }
	 }
```

## Power
Power readings are published every 2 seconds on the topic `sensor/power/power` and of the format:
```json
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
```

## Configuration
Configuration is published via an MQTT request that either changes the configuration or request it to be published.
The message will be published on the `sensor/power/config` topic with a format of:
```json
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
```

---

# MQTT Input Messages
This system has a fairly high amount of post-deployment configurability via an MQTT message published on the 
topic `sensor/power/set` in the form of:
```json
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
```

## Grid Mapping
`gridMapping` field is an array of 3 (required) with up to 2 of the values allowed to be true.  The fields
selected will be used to identify which, if any, of the sensors should be combined to provide for left and
right legs of a 110/220 system.  In the example above, voltage sensors 2 and 3 are connected to the grid
left and right legs.

## Grid Sensors
`gridSensors` is an array of 2 (max) indicating which CT sensors are attached to electical system lines.
It is 1-based identification.  In the example above, CT sensors 6 and 7 are attached to the grid left and
right legs.

## Voltage Mapping
`voltageMapping` field is an array of 16 (required) that will map each CT sensor one of the 3 voltage sensors.
It is 1-based identification of the voltage sensor with an additional allowed value of 256, which indicates
that the voltage of the grid should be used.  Any values should map only to sensors marked as `true` in the
`gridMapping` section.  In the example above, all the sensors are on either voltage sensor 2 or sensor 3, 
except for the last one which, having a value of 256, is using 220 volt and would use the combined voltage 
of sensors 2 and 3 to calculate it's wattage.

## Sensor Corrections
Sensor corrections have a number of options and will be processed in this order:
- `clear`: if true, all corrections in memory and runtime will be removed
- `delete`: if `clear` is also specified in the same message, this will not be run, but otherwise it will
selectively remove the specified correction from memory and runtime.  It is 1-based.
- `set`: The specified (1-based) sensor will be configured and the configuration will be stored in NVM to be
available upon the next reboot.  It is not necessary to delete a configuration before setting a new one.  Up
to 5 pairs of values can be provided to allow for better curve fitting of the amperage read from the sensor.

---

# Home Assistant
Home assistant can read the output via sensor configurations.

## MQTT Sensors
The "raw" sensors can be extracted from the MQTT message with the following:

```yml
  - name: "Power Monitor - Grid Wattage"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.wattage.grid }}"
    device_class: power
    unit_of_measurement: W
    json_attributes_topic: "sensor/power/power"
    json_attributes_template: "{{ value_json | tojson }}"

  - name: "Power Monitor - Main Left Voltage"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.voltages.ac2 }}"
    unit_of_measurement: V
    device_class: voltage

  - name: "Power Monitor - Main Left Current"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.current.sensor10 }}"
    unit_of_measurement: A
    device_class: current

  - name: "Power Monitor - Main Left Wattage"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.wattage.sensor10 }}"
    unit_of_measurement: W
    device_class: power

  - name: "Power Monitor - Main Right Voltage"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.voltages.ac3 }}"
    unit_of_measurement: V
    device_class: voltage

  - name: "Power Monitor - Main Right Current"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.current.sensor12 }}"
    unit_of_measurement: A
    device_class: current

  - name: "Power Monitor - Main Right Wattage"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.wattage.sensor12 }}"
    unit_of_measurement: W
    device_class: power

  - name: "Power Monitor - Sub Current"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.current.sensor16 }}"
    unit_of_measurement: A
    device_class: current

  - name: "Power Monitor - Sub Wattage"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.wattage.sensor16 }}"
    unit_of_measurement: W
    device_class: power

  - name: "Power Monitor - Furnace Current"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.current.sensor1 }}"
    unit_of_measurement: A
    device_class: current

  - name: "Power Monitor - Furnace Wattage"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.wattage.sensor1 }}"
    unit_of_measurement: W
    device_class: power

  - name: "Power Monitor - Hot Water Current"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.current.sensor2 }}"
    unit_of_measurement: A
    device_class: current

  - name: "Power Monitor - Hot Water Wattage"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.wattage.sensor2 }}"
    unit_of_measurement: W
    device_class: power

  - name: "Power Monitor - Well Current"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.current.sensor3 }}"
    unit_of_measurement: A
    device_class: current

  - name: "Power Monitor - Well Wattage"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.wattage.sensor3 }}"
    unit_of_measurement: W
    device_class: power

  - name: "Power Monitor - Freezer Current"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.current.sensor4 }}"
    unit_of_measurement: A
    device_class: current

  - name: "Power Monitor - Freezer Wattage"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.wattage.sensor4 }}"
    unit_of_measurement: W
    device_class: power

  - name: "Power Monitor - Fridge Current"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.current.sensor5 }}"
    unit_of_measurement: A
    device_class: current

  - name: "Power Monitor - Fridge Wattage"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.wattage.sensor5 }}"
    unit_of_measurement: W
    device_class: power

  - name: "Power Monitor - Solis Charger Current"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.current.sensor6 }}"
    unit_of_measurement: A
    device_class: current

  - name: "Power Monitor - Solis Charger Wattage"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.wattage.sensor6 }}"
    unit_of_measurement: W
    device_class: power

  - name: "Power Monitor - Jacuzzi Current"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.current.sensor7 }}"
    unit_of_measurement: A
    device_class: current

  - name: "Power Monitor - Jacuzzi Wattage"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.wattage.sensor7 }}"
    unit_of_measurement: W
    device_class: power

  - name: "Power Monitor - AC Current"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.current.sensor8 }}"
    unit_of_measurement: A
    device_class: current

  - name: "Power Monitor - AC Wattage"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.wattage.sensor8 }}"
    unit_of_measurement: W
    device_class: power

  - name: "Power Monitor - Dryer Current"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.current.sensor9 }}"
    unit_of_measurement: A
    device_class: current

  - name: "Power Monitor - Dryer Wattage"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.wattage.sensor9 }}"
    unit_of_measurement: W
    device_class: power

  - name: "Power Monitor - Sauna Current"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.current.sensor11 }}"
    unit_of_measurement: A
    device_class: current

  - name: "Power Monitor - Sauna Wattage"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.wattage.sensor11 }}"
    unit_of_measurement: W
    device_class: power

  - name: "Power Monitor - Oven Current"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.current.sensor13 }}"
    unit_of_measurement: A
    device_class: current

  - name: "Power Monitor - Oven Wattage"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.wattage.sensor13 }}"
    unit_of_measurement: W
    device_class: power

  - name: "Power Monitor - Dishwasher Current"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.current.sensor14 }}"
    unit_of_measurement: A
    device_class: current

  - name: "Power Monitor - Dishwasher Wattage"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.wattage.sensor14 }}"
    unit_of_measurement: W
    device_class: power

  - name: "Power Monitor - Washing Machine Current"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.current.sensor15 }}"
    unit_of_measurement: A
    device_class: current

  - name: "Power Monitor - Washing Machine Wattage"
    state_topic: "sensor/power/power"
    value_template: "{{ value_json.wattage.sensor15 }}"
    unit_of_measurement: W
    device_class: power
    
  - name: "Power Monitor Uptime"
    state_topic: "sensor/power/power"
    icon: mdi:clock-start
    value_template: >-
      {% set time =  value_json["sensor"]["time"] / 1000 %}
      {% set minutes = ((time % 3600) / 60) | int %}
      {% set hours = ((time % 86400) / 3600) | int %}
      {% set days = (time / 86400) | int %}
      {%- if time < 60 -%}
        Less than a minute
      {%- else -%}
        {%- if days > 0 -%}
          {%- if days == 1 -%}
            1 day
          {%- else -%}
            {{ days }} days
          {%- endif -%}
        {%- endif -%}
        {%- if hours > 0 -%}
          {%- if days > 0 -%}
            {{ ', ' }}
          {%- endif -%}
          {%- if hours == 1 -%}
            1 hour
          {%- else -%}
            {{ hours }} hours
          {%- endif -%}
        {%- endif -%}
        {%- if minutes > 0 -%}
          {%- if days > 0 or hours > 0 -%}
            {{ ', ' }}
          {%- endif -%}
          {%- if minutes == 1 -%}
            1 minute
          {%- else -%}
            {{ minutes }} minutes
          {%- endif -%}
        {%- endif -%}
      {%- endif -%}
    expire_after: 610
    payload_not_available: 0

  - name: "Power Monitor RSSI"
    state_topic: "sensor/power/power"
    value_template: "{{  value_json.sensor.rssi }}"
    device_class: signal_strength

  - name: "Power Monitor Temp"
    state_topic: "sensor/power/status"
    value_template: "{{ value_json.environment.temp }}"
    device_class: temperature

  - name: "Power Monitor Humidity"
    state_topic: "sensor/power/status"
    value_template: "{{ value_json.environment.humidity }}"
    device_class: humidity

```

## Influx Sensors
With the sensors created, their values will be stored in the recorder system and routed to Influx, if so configured.  kWH 
power calculations can then be performed by querying via:

```shell
curl "http://localhost:8086/query?db=home_assistant&q=select%20integral%28%22value%22%2C%201h%29%20%2F%201000%20from%20%22W%22%20where%20entity_id%20%3D%20%27power_monitor_grid_wattage%27%20group%20by%20time%281d%29%20fill%28null%29"
```

which would yield an output like:

```json
{
  "results": [
    {
      "statement_id": 0,
      "series": [
        {
          "name": "W",
          "columns": [
            "time",
            "integral"
          ],
          "values": [
            [
              "2024-12-09T00:00:00Z",
              7.403785195584249
            ]
          ]
        }
      ]
    }
  ]
}
```

These can be configured as REST sensors in Home Assistant via:

```yml
- platform: rest
  name: "Power Monitor kWH Grid"
  resource: !secret power_kwh_grid
  device_class: energy
  state_class: total_increasing
  unit_of_measurement: kWh
  scan_interval: 600
  timeout: 30
  value_template: >-
    {% if value_json["results"][0]["series"] is not defined %}
      0.0
    {% else %}
      {{ (value_json["results"][0]["series"][0]["values"][0][1]) | float | round(3) }}
    {% endif %}


- platform: rest
  name: "Power Monitor kWH Main Left"
  resource: !secret power_kwh_main_left
  device_class: energy
  state_class: total_increasing
  unit_of_measurement: kWh
  scan_interval: 600
  timeout: 30
  value_template: >-
    {% if value_json["results"][0]["series"] is not defined %}
      0.0
    {% else %}
      {{ (value_json["results"][0]["series"][0]["values"][0][1]) | float | round(3) }}
    {% endif %}

- platform: rest
  name: "Power Monitor kWH Main Right"
  resource: !secret power_kwh_main_right
  device_class: energy
  state_class: total_increasing
  unit_of_measurement: kWh  name: "Power Monitor kWH Sub Panel"
  resource: !secret power_kwh_sub
  device_class: energy
  state_class: total_increasing
  unit_of_measurement: kWh
  scan_interval: 600
  timeout: 30
  value_template: >-
    {% if value_json["results"][0]["series"] is not defined %}
      0.0
    {% else %}
      {{ (value_json["results"][0]["series"][0]["values"][0][1]) | float | round(3) }}
    {% endif %}

- platform: rest
  name: "Power Monitor kWH Air Conditioner"
  resource: !secret power_kwh_air_conditioner
  device_class: energy
  state_class: total_increasing
  unit_of_measurement: kWh
  scan_interval: 600
  timeout: 30
  value_template: >-
    {% if value_json["results"][0]["series"] is not defined %}
      0.0
    {% else %}
      {{ (value_json["results"][0]["series"][0]["values"][0][1]) | float | round(3) }}
    {% endif %}

- platform: rest
  name: "Power Monitor kWH Dishwasher"
  resource: !secret power_kwh_dishwasher
  device_class: energy
  state_class: total_increasing
  unit_of_measurement: kWh
  scan_interval: 600
  timeout: 30
  value_template: >-
    {% if value_json["results"][0]["series"] is not defined %}
      0.0
    {% else %}
      {{ (value_json["results"][0]["series"][0]["values"][0][1]) | float | round(3) }}
    {% endif %}

- platform: rest
  name: "Power Monitor kWH Dryer"
  resource: !secret power_kwh_dryer
  device_class: energy
  state_class: total_increasing
  unit_of_measurement: kWh
  scan_interval: 600
  timeout: 30
  value_template: >-
    {% if value_json["results"][0]["series"] is not defined %}
      0.0
    {% else %}
      {{ (value_json["results"][0]["series"][0]["values"][0][1]) | float | round(3) }}
    {% endif %}

- platform: rest
  name: "Power Monitor kWH Freezer"
  resource: !secret power_kwh_freezer
  device_class: energy
  state_class: total_increasing
  unit_of_measurement: kWh
  scan_interval: 600
  timeout: 30
  value_template: >-
    {% if value_json["results"][0]["series"] is not defined %}
      0.0
    {% else %}
      {{ (value_json["results"][0]["series"][0]["values"][0][1]) | float | round(3) }}
    {% endif %}


- platform: rest
  name: "Power Monitor kWH Fridge"
  resource: !secret power_kwh_fridge
  device_class: energy
  state_class: total_increasing
  unit_of_measurement: kWh
  scan_interval: 600
  timeout: 30
  value_template: >-
    {% if value_json["results"][0]["series"] is not defined %}
      0.0
    {% else %}
      {{ (value_json["results"][0]["series"][0]["values"][0][1]) | float | round(3) }}
    {% endif %}

- platform: rest
  name: "Power Monitor kWH Furnace"
  resource: !secret power_kwh_furnace
  device_class: energy
  state_class: total_increasing
  unit_of_measurement: kWh
  scan_interval: 600
  timeout: 30
  value_template: >-
    {% if value_json["results"][0]["series"] is not defined %}
      0.0
    {% else %}
      {{ (value_json["results"][0]["series"][0]["values"][0][1]) | float | round(3) }}
    {% endif %}

- platform: rest
  name: "Power Monitor kWH Hot Water"
  resource: !secret power_kwh_hot_water
  device_class: energy
  state_class: total_increasing
  unit_of_measurement: kWh
  scan_interval: 600
  timeout: 30
  value_template: >-
    {% if value_json["results"][0]["series"] is not defined %}
      0.0
    {% else %}
      {{ (value_json["results"][0]["series"][0]["values"][0][1]) | float | round(3) }}
    {% endif %}

- platform: rest
  name: "Power Monitor kWH Jacuzzi"
  resource: !secret power_kwh_jacuzzi
  device_class: energy
  state_class: total_increasing
  unit_of_measurement: kWh
  scan_interval: 600
  timeout: 30
  value_template: >-
    {% if value_json["results"][0]["series"] is not defined %}
      0.0
    {% else %}
      {{ (value_json["results"][0]["series"][0]["values"][0][1]) | float | round(3) }}
    {% endif %}

- platform: rest
  name: "Power Monitor kWH Oven"
  resource: !secret power_kwh_oven
  device_class: energy
  state_class: total_increasing
  unit_of_measurement: kWh
  scan_interval: 600
  timeout: 30
  value_template: >-
    {% if value_json["results"][0]["series"] is not defined %}
      0.0
    {% else %}
      {{ (value_json["results"][0]["series"][0]["values"][0][1]) | float | round(3) }}
    {% endif %}

- platform: rest
  name: "Power Monitor kWH Sauna"
  resource: !secret power_kwh_sauna
  device_class: energy
  state_class: total_increasing
  unit_of_measurement: kWh
  scan_interval: 600
  timeout: 30
  value_template: >-
    {% if value_json["results"][0]["series"] is not defined %}
      0.0
    {% else %}
      {{ (value_json["results"][0]["series"][0]["values"][0][1]) | float | round(3) }}
    {% endif %}

- platform: rest
  name: "Power Monitor kWH Solis Charger"
  resource: !secret power_kwh_solis
  device_class: energy
  state_class: total_increasing
  unit_of_measurement: kWh
  scan_interval: 600
  timeout: 30
  value_template: >-
    {% if value_json["results"][0]["series"] is not defined %}
      0.0
    {% else %}
      {{ (value_json["results"][0]["series"][0]["values"][0][1]) | float | round(3) }}
    {% endif %}

- platform: rest
  name: "Power Monitor kWH Washing Machine"
  resource: !secret power_kwh_washer
  device_class: energy
  state_class: total_increasing
  unit_of_measurement: kWh
  scan_interval: 600
  timeout: 30
  value_template: >-
    {% if value_json["results"][0]["series"] is not defined %}
      0.0
    {% else %}
      {{ (value_json["results"][0]["series"][0]["values"][0][1]) | float | round(3) }}
    {% endif %}

- platform: rest
  name: "Power Monitor kWH Well Pump"
  resource: !secret power_kwh_well
  device_class: energy
  state_class: total_increasing
  unit_of_measurement: kWh
  scan_interval: 600
  timeout: 30
  value_template: >-
    {% if value_json["results"][0]["series"] is not defined %}
      0.0
    {% else %}
      {{ (value_json["results"][0]["series"][0]["values"][0][1]) | float | round(3) }}
    {% endif %}
```

---

# Build
This system is built with the Eclipse/Sloeber workspace with an ESP32 platform version of `2.0.17`.  Build
configuration uses the following properties:

```properties
Config.Release.board.BOARD.ID=esp32
Config.Release.board.BOARD.MENU.CPUFreq=240
Config.Release.board.BOARD.MENU.DebugLevel=debug
Config.Release.board.BOARD.MENU.EraseFlash=none
Config.Release.board.BOARD.MENU.EventsCore=1
Config.Release.board.BOARD.MENU.FlashFreq=80
Config.Release.board.BOARD.MENU.FlashMode=dout
Config.Release.board.BOARD.MENU.FlashSize=4M
Config.Release.board.BOARD.MENU.JTAGAdapter=default
Config.Release.board.BOARD.MENU.LoopCore=1
Config.Release.board.BOARD.MENU.PSRAM=disabled
Config.Release.board.BOARD.MENU.PartitionScheme=huge_app
Config.Release.board.BOARD.MENU.UploadSpeed=921600
Config.Release.board.BOARD.TXT=${SLOEBER_HOME}/arduinoPlugin/packages/esp32/hardware/esp32/2.0.17/boards.txt
Config.Release.compile.sloeber.extra.all=
Config.Release.compile.sloeber.extra.archive=
Config.Release.compile.sloeber.extra.assembly=
Config.Release.compile.sloeber.extra.c.compile=
Config.Release.compile.sloeber.extra.compile=
Config.Release.compile.sloeber.extra.cpp.compile=
Config.Release.compile.sloeber.extra.link=
Config.Release.compile.sloeber.size.custom=
Config.Release.compile.sloeber.size.type=RAW_RESULT
Config.Release.compile.sloeber.warning_level=MORE
Config.Release.compile.sloeber.warning_level.custom=
Config.Release.other.IS_VERSION_CONTROLLED=true
```
---

# 3D Print Cover
In the `./Cover` folder, an STL model for a 3D printable cover is provided.  This cover will fit over the board and provides
cutouts for all the CT sensors, LCD display and power inputs.  Minor fitting may be required.

---

# Notes/TODO
Numberous TODO and fixes remain in this system, but has been built and deployed on a WiFI environment.

Some high priority updates would include:
- PRIORITY: SHT31 does not read after initial in setup()
- Build and test on a wired Ethernet configuration
- Migration to v3.x platorm
- Additional input checks, especially around inbound MQTT messages
- FIX: unbounded correction array causes: assert failed: block_locate_free heap_tlsf.c:441 (block_size(block) >= size)
- Fix fine grained Preference requirement: works now, slow, but used infrequent enough not to worry
