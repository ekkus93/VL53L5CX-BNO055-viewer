/*
 * VL53L5CX ToF Sensor + BNO08X IMU Reader for ESP32
 *
 * Reads 8x8 distance data from VL53L5CX and orientation from BNO08X,
 * outputs JSON over serial.
 *
 * Wiring (both sensors share I2C bus):
 *   VIN -> 3V3
 *   GND -> GND
 *   SDA -> GPIO 21
 *   SCL -> GPIO 47
 *   LPn -> GPIO 48 (VL53L5CX enable, set HIGH)
 *
 * Note: ESP32-S3 - GPIO19/20 are reserved for native USB (D-/D+),
 * GPIO22 does not exist, so classic-ESP32 I2C pins can't be reused here.
 */

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>

// Version - must match viewer config.VERSION
#define VERSION "0.1.0"

// Pin definitions
#define SDA_PIN 21
#define SCL_PIN 47
#define LPN_PIN 48

// VL53L5CX ToF sensor instance
SparkFun_VL53L5CX sensor;
VL53L5CX_ResultsData measurementData;

// BNO055 IMU instance (default I2C address 0x28; 0x29 conflicts with VL53L5CX)
// Note: named "bno" not "imu" - the Adafruit library defines a namespace "imu".
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
bool imuAvailable = false;

// Current quaternion (wxyz format)
float quatW = 1.0, quatX = 0.0, quatY = 0.0, quatZ = 0.0;

// I2C speed - the BNO055 maxes out at 400kHz, so the shared bus must run at 400kHz.
// (The VL53L5CX prefers 1MHz for 8x8; if you see dropped ToF frames, lower the
//  ranging frequency or switch to 4x4 resolution.)
#define I2C_SPEED 400000

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("{\"status\":\"initializing\"}");

  // Enable I2C on sensor by setting LPn HIGH
  pinMode(LPN_PIN, OUTPUT);
  digitalWrite(LPN_PIN, HIGH);
  delay(10);

  // Initialize I2C with specified pins and speed
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_SPEED);

  Serial.println("{\"status\":\"i2c_ready\"}");

  // Initialize sensor
  if (!sensor.begin()) {
    Serial.println("{\"error\":\"sensor_init_failed\"}");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("{\"status\":\"sensor_found\"}");

  // Configure sensor for 8x8 resolution
  sensor.setResolution(64);  // 64 zones = 8x8

  // Set ranging frequency to 15Hz (stable for continuous streaming)
  sensor.setRangingFrequency(15);

  // Start ranging
  sensor.startRanging();

  Serial.println("{\"status\":\"ranging_started\",\"resolution\":\"8x8\",\"frequency_hz\":15}");

  // Initialize BNO055 IMU (shares I2C bus with VL53L5CX)
  // IMUPLUS mode = accel+gyro fusion, no magnetometer - immune to magnetic
  // interference (closest equivalent to the BNO08x game rotation vector).
  if (bno.begin(OPERATION_MODE_IMUPLUS)) {
    imuAvailable = true;
    // If your breakout has an external 32kHz crystal (most Adafruit boards do),
    // uncomment for better stability: bno.setExtCrystalUse(true);
    Serial.println("{\"status\":\"imu_ready\",\"mode\":\"imuplus\"}");
  } else {
    Serial.println("{\"status\":\"imu_not_found\"}");
  }
}

void loop() {
  // Poll IMU for the latest orientation quaternion (non-blocking register read)
  if (imuAvailable) {
    imu::Quaternion q = bno.getQuat();
    quatW = q.w();
    quatX = q.x();
    quatY = q.y();
    quatZ = q.z();
  }

  // Check if new ToF data is available
  if (sensor.isDataReady()) {
    if (sensor.getRangingData(&measurementData)) {
      // Output JSON with distance and quaternion data
      Serial.print("{\"distances\":[");

      for (int i = 0; i < 64; i++) {
        // Distance in mm
        Serial.print(measurementData.distance_mm[i]);
        if (i < 63) Serial.print(",");
      }

      Serial.print("],\"status\":[");

      for (int i = 0; i < 64; i++) {
        // Target status (5 = valid, others = various error states)
        Serial.print(measurementData.target_status[i]);
        if (i < 63) Serial.print(",");
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
  }

  // Small delay to prevent overwhelming the serial buffer
  delay(1);
}
