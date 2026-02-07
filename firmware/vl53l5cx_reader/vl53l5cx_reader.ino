/*
 * VL53L5CX ToF Sensor + BNO08X IMU Reader for ESP32
 *
 * Reads 8x8 distance data from 5 VL53L5CX sensors and orientation from BNO08X,
 * outputs JSON over serial.
 *
 * Wiring (all sensors share I2C bus):
 *   VIN -> 3V3
 *   GND -> GND
 *   SDA -> GPIO 21
 *   SCL -> GPIO 22
 *   LPn pins -> GPIO 19, 18, 5, 17, 16 (one per sensor for address management)
 */

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

// Version - must match viewer config.VERSION
#define VERSION "0.2.0"

// Pin definitions
#define SDA_PIN 21
#define SCL_PIN 22

// Multi-sensor configuration
#define NUM_SENSORS 5
const uint8_t LPN_PINS[NUM_SENSORS] = {19, 18, 5, 17, 16};
const uint8_t I2C_ADDRESSES[NUM_SENSORS] = {0x30, 0x31, 0x32, 0x33, 0x34};

// VL53L5CX ToF sensor instances
SparkFun_VL53L5CX sensors[NUM_SENSORS];
VL53L5CX_ResultsData measurementData[NUM_SENSORS];
bool sensor_active[NUM_SENSORS] = {false};

// BNO08X IMU instance
BNO08x imu;
bool imuAvailable = false;

// Current quaternion (wxyz format)
float quatW = 1.0, quatX = 0.0, quatY = 0.0, quatZ = 0.0;

// I2C speed - use 1MHz for fast data transfer
#define I2C_SPEED 1000000

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("{\"status\":\"initializing\"}");

  // Initialize I2C with specified pins and speed
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_SPEED);

  Serial.println("{\"status\":\"i2c_ready\"}");

  // Set all LPn pins LOW to disable all sensors initially
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(LPN_PINS[i], OUTPUT);
    digitalWrite(LPN_PINS[i], LOW);
  }
  delay(10);

  // Enable sensors one-by-one, change address, configure
  int sensors_initialized = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(LPN_PINS[i], HIGH);  // Enable this sensor
    delay(100);  // Wait for sensor to boot (critical timing)

    // Initialize sensor at default address 0x29
    if (!sensors[i].begin()) {
      Serial.printf("{\"error\":\"sensor_%d_init_failed\"}\n", i);
      digitalWrite(LPN_PINS[i], LOW);  // Disable failed sensor
      continue;
    }

    // Change to unique address
    if (!sensors[i].setAddress(I2C_ADDRESSES[i])) {
      Serial.printf("{\"error\":\"sensor_%d_address_change_failed\"}\n", i);
      continue;
    }

    // Configure sensor for 8x8 resolution at 10Hz
    sensors[i].setResolution(64);  // 64 zones = 8x8
    sensors[i].setRangingFrequency(10);  // Reduced from 15Hz for bandwidth

    // Start ranging
    if (sensors[i].startRanging()) {
      sensor_active[i] = true;
      sensors_initialized++;
      Serial.printf("{\"status\":\"sensor_%d_ready\",\"address\":\"0x%02X\"}\n", i, I2C_ADDRESSES[i]);
    } else {
      Serial.printf("{\"error\":\"sensor_%d_ranging_failed\"}\n", i);
    }
  }

  Serial.printf("{\"status\":\"ranging_started\",\"sensors\":%d,\"resolution\":\"8x8\",\"frequency_hz\":10}\n", sensors_initialized);

  // Initialize BNO08X IMU (shares I2C bus with VL53L5CX)
  // Try default address 0x4A first, then alternate 0x4B
  if (imu.begin(0x4A, Wire)) {
    imuAvailable = true;
  } else if (imu.begin(0x4B, Wire)) {
    imuAvailable = true;
  }

  if (imuAvailable) {
    // Enable game rotation vector at 10ms interval (100Hz)
    // Game rotation uses accel+gyro only (no magnetometer) - immune to magnetic interference
    imu.enableGameRotationVector(10);
    Serial.println("{\"status\":\"imu_ready\",\"mode\":\"game_rotation_vector\",\"frequency_hz\":100}");
  } else {
    Serial.println("{\"status\":\"imu_not_found\"}");
  }
}

void loop() {
  // Poll IMU for new orientation data (non-blocking)
  if (imuAvailable && imu.wasReset()) {
    // Re-enable game rotation vector if IMU was reset
    imu.enableGameRotationVector(10);
  }

  if (imuAvailable && imu.getSensorEvent()) {
    if (imu.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      quatW = imu.getQuatReal();
      quatX = imu.getQuatI();
      quatY = imu.getQuatJ();
      quatZ = imu.getQuatK();
    }
  }

  // Check if any sensor has new data
  bool any_ready = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensor_active[i] && sensors[i].isDataReady()) {
      any_ready = true;
      break;
    }
  }

  if (any_ready) {
    // Begin JSON frame with sensor array
    Serial.print("{\"sensors\":[");
    bool first = true;

    // Gather data from all ready sensors
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (sensor_active[i] && sensors[i].isDataReady() &&
          sensors[i].getRangingData(&measurementData[i])) {

        if (!first) Serial.print(",");
        first = false;

        // Output sensor object
        Serial.printf("{\"id\":%d,\"distances\":[", i);

        for (int j = 0; j < 64; j++) {
          Serial.print(measurementData[i].distance_mm[j]);
          if (j < 63) Serial.print(",");
        }

        Serial.print("],\"status\":[");

        for (int j = 0; j < 64; j++) {
          Serial.print(measurementData[i].target_status[j]);
          if (j < 63) Serial.print(",");
        }

        Serial.print("]}");
      }
    }

    // Add quaternion (wxyz format) with 6 decimal places for accuracy
    Serial.print("],\"quat\":[");
    Serial.print(quatW, 6); Serial.print(",");
    Serial.print(quatX, 6); Serial.print(",");
    Serial.print(quatY, 6); Serial.print(",");
    Serial.print(quatZ, 6);
    Serial.print("],\"v\":\"");
    Serial.print(VERSION);
    Serial.println("\"}");
  }

  // Small delay to prevent overwhelming the serial buffer
  delay(1);
}
