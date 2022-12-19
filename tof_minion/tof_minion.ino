/*
Name: tof_minion.ino
Author: Jonathan Lee
Description: codebase for capturing Time of Flight sensor data on
             minion ESP32C3 and relaying it to master ESP32S2
*/
#include <Wire.h>
#include <vl53l4cx_class.h>

// Time of Flight Parameters
#define FRONT_XSHUT_PIN 0
#define RIGHT_XSHUT_PIN 1

#define FRONT_BEACON_SIGNAL_PIN 4
#define RIGHT_FAR_SIGNAL_PIN    6
#define RIGHT_CLOSE_SIGNAL_PIN  7

#define FRONT_MM_BEACON_THRESHOLD 35
#define RIGHT_MM_FAR_THRESHOLD    450
#define RIGHT_MM_CLOSE_THRESHOLD  320

TwoWire *DEV_I2C = &Wire;
VL53L4CX front_sensor;
VL53L4CX right_sensor;

void setup() {
  // Signal Pin Setup
  pinMode(FRONT_BEACON_SIGNAL_PIN, OUTPUT);
  pinMode(RIGHT_FAR_SIGNAL_PIN, OUTPUT);
  pinMode(RIGHT_CLOSE_SIGNAL_PIN, OUTPUT);
  digitalWrite(FRONT_BEACON_SIGNAL_PIN, LOW);
  digitalWrite(RIGHT_FAR_SIGNAL_PIN, LOW);
  digitalWrite(RIGHT_CLOSE_SIGNAL_PIN, LOW);

  // I2C Setup
  Wire1.setPins(SDA, SCL);
  DEV_I2C->begin();

  // // Front and Right ToF Sensor Setup
  front_sensor.setI2cDevice(DEV_I2C);
  front_sensor.setXShutPin(FRONT_XSHUT_PIN);
  front_sensor.begin();
  front_sensor.VL53L4CX_Off();
  front_sensor.InitSensor(0x27);
  front_sensor.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_SHORT);
  
  right_sensor.setI2cDevice(DEV_I2C);
  right_sensor.setXShutPin(RIGHT_XSHUT_PIN);
  right_sensor.begin();
  right_sensor.VL53L4CX_Off();
  right_sensor.InitSensor(0x28);
  right_sensor.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_SHORT);

  front_sensor.VL53L4CX_StartMeasurement();
  right_sensor.VL53L4CX_StartMeasurement();
}

void loop() {
  // get front sensor reading and generate signal
  uint16_t front_reading = get_sensor_reading(front_sensor);
  if (front_reading < FRONT_MM_BEACON_THRESHOLD) {
    digitalWrite(FRONT_BEACON_SIGNAL_PIN, HIGH);
  } else {
    digitalWrite(FRONT_BEACON_SIGNAL_PIN, LOW);
  }
  
  // get right sensor reading and generate signal
  uint16_t right_reading = get_sensor_reading(right_sensor);
  if (right_reading < RIGHT_MM_CLOSE_THRESHOLD) {
    digitalWrite(RIGHT_CLOSE_SIGNAL_PIN, HIGH);
    digitalWrite(RIGHT_FAR_SIGNAL_PIN, LOW);
  } else if (right_reading > RIGHT_MM_FAR_THRESHOLD){
    digitalWrite(RIGHT_CLOSE_SIGNAL_PIN, LOW);
    digitalWrite(RIGHT_FAR_SIGNAL_PIN, HIGH);
  } else {
    digitalWrite(RIGHT_CLOSE_SIGNAL_PIN, LOW);
    digitalWrite(RIGHT_FAR_SIGNAL_PIN, LOW);
  }
}

/*
get range data from sensor
:param sensor: given ToF sensor
:return: range measurement in mm
*/
uint16_t get_sensor_reading(VL53L4CX& sensor) {
  uint16_t reading;
  do {
    sensor.VL53L4CX_ClearInterruptAndStartMeasurement();
    VL53L4CX_MultiRangingData_t data;
    VL53L4CX_MultiRangingData_t *p_data = &data;
    uint8_t data_ready = 0;
    uint8_t num_found = 0;
    int status;

    while (!data_ready) {
      status = sensor.VL53L4CX_GetMeasurementDataReady(&data_ready);
    }
    
    if ((!status) && (data_ready != 0)) {
      status = sensor.VL53L4CX_GetMultiRangingData(p_data);
      num_found = p_data->NumberOfObjectsFound;

      reading = p_data->RangeData[0].RangeMilliMeter;
      for (int i = 1; i < num_found; i++) {
        uint16_t measurement = p_data->RangeData[i].RangeMilliMeter;
        if (measurement < reading) {
          reading = measurement;
        }
      }

      if (status == 0) {
        status = sensor.VL53L4CX_ClearInterruptAndStartMeasurement();
      }
    }
  } while (reading > 8190);
  return reading;
}