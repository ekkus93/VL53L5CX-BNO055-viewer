/*
 * VL53L5CX ToF Sensor Reader for ESP32 (Dual I2C Bus)
 *
 * Reads 8x8 distance data from 4 VL53L5CX sensors using two I2C buses
 * for parallel reads, outputs JSON over serial.
 *
 * Wiring:
 *   Bus 0 (Wire):  SDA -> GPIO 21, SCL -> GPIO 22  -> Sensors 0, 1
 *   Bus 1 (Wire1): SDA -> GPIO 25, SCL -> GPIO 26  -> Sensors 2, 3
 *   VIN -> 3V3, GND -> GND (all sensors)
 *   LPn pins -> GPIO 19, 18, 17, 16 (one per sensor for address management)
 */

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

// Version - must match viewer config.VERSION
#define VERSION "0.3.0"

// I2C bus 0 pins (sensors 0-1)
#define SDA0_PIN 21
#define SCL0_PIN 22

// I2C bus 1 pins (sensors 2-3)
#define SDA1_PIN 25
#define SCL1_PIN 26

// Multi-sensor configuration
#define NUM_SENSORS 4
#define BUS0_SENSORS 2  // Sensors 0-1 on Wire
const uint8_t LPN_PINS[NUM_SENSORS] = {19, 18, 17, 16};
const uint8_t I2C_ADDRESSES[NUM_SENSORS] = {0x30, 0x31, 0x32, 0x33};

// VL53L5CX ToF sensor instances
SparkFun_VL53L5CX sensors[NUM_SENSORS];
VL53L5CX_ResultsData measurementData[NUM_SENSORS];
bool sensor_active[NUM_SENSORS] = {false};

// Bus 1 parallel read state (shared between cores)
SemaphoreHandle_t bus1Mutex;
bool bus1_fresh[NUM_SENSORS - BUS0_SENSORS] = {false};

// I2C speed - use 1MHz for fast data transfer
#define I2C_SPEED 1000000

// JSON output buffer
static char buf[4096];

// FreeRTOS task: reads bus 1 sensors on core 0
void bus1ReadTask(void *parameter) {
  while (true) {
    // Phase 1: check which sensors are ready (quick I2C checks)
    bool ready[NUM_SENSORS - BUS0_SENSORS] = {false};
    for (int i = BUS0_SENSORS; i < NUM_SENSORS; i++) {
      if (sensor_active[i] && sensors[i].isDataReady()) {
        ready[i - BUS0_SENSORS] = true;
      }
    }

    // Phase 2: read ALL ready sensors (slow I2C reads)
    for (int i = BUS0_SENSORS; i < NUM_SENSORS; i++) {
      if (ready[i - BUS0_SENSORS]) {
        VL53L5CX_ResultsData tempData;
        if (sensors[i].getRangingData(&tempData)) {
          xSemaphoreTake(bus1Mutex, portMAX_DELAY);
          measurementData[i] = tempData;
          bus1_fresh[i - BUS0_SENSORS] = true;
          xSemaphoreGive(bus1Mutex);
        }
      }
    }
    delay(1);
  }
}

void setup() {
  Serial.begin(460800);
  delay(1000);

  Serial.println("{\"status\":\"initializing\"}");

  // Initialize both I2C buses
  Wire.begin(SDA0_PIN, SCL0_PIN);
  Wire.setClock(I2C_SPEED);
  Wire1.begin(SDA1_PIN, SCL1_PIN);
  Wire1.setClock(I2C_SPEED);

  // Scan both buses for devices
  for (int bus = 0; bus < 2; bus++) {
    TwoWire &w = (bus == 0) ? Wire : Wire1;
    Serial.printf("{\"status\":\"i2c_scan\",\"bus\":%d,\"devices\":[", bus);
    bool first = true;
    for (uint8_t addr = 1; addr < 127; addr++) {
      w.beginTransmission(addr);
      if (w.endTransmission() == 0) {
        if (!first) Serial.print(",");
        Serial.printf("\"0x%02X\"", addr);
        first = false;
      }
    }
    Serial.println("]}");
  }

  Serial.println("{\"status\":\"i2c_ready\",\"buses\":2}");

  // Create mutex for bus 1 data sharing between cores
  bus1Mutex = xSemaphoreCreateMutex();

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

    // Select I2C bus based on sensor index
    TwoWire &wire = (i < BUS0_SENSORS) ? Wire : Wire1;

    // Initialize sensor at default address 0x29 on its bus
    if (!sensors[i].begin(0x29, wire)) {
      Serial.printf("{\"error\":\"sensor_%d_init_failed\"}\n", i);
      digitalWrite(LPN_PINS[i], LOW);  // Disable failed sensor
      continue;
    }

    // Change to unique address
    if (!sensors[i].setAddress(I2C_ADDRESSES[i])) {
      Serial.printf("{\"error\":\"sensor_%d_address_change_failed\"}\n", i);
      continue;
    }

    // Configure sensor for 8x8 resolution at 15Hz (max for 8x8)
    sensors[i].setResolution(64);  // 64 zones = 8x8
    sensors[i].setRangingFrequency(15);

    // Start ranging
    if (sensors[i].startRanging()) {
      sensor_active[i] = true;
      sensors_initialized++;
      Serial.printf("{\"status\":\"sensor_%d_ready\",\"address\":\"0x%02X\",\"bus\":%d}\n",
                    i, I2C_ADDRESSES[i], i < BUS0_SENSORS ? 0 : 1);
    } else {
      Serial.printf("{\"error\":\"sensor_%d_ranging_failed\"}\n", i);
    }
  }

  Serial.printf("{\"status\":\"ranging_started\",\"sensors\":%d,\"resolution\":\"8x8\",\"frequency_hz\":15,\"buses\":2}\n", sensors_initialized);

  // Start bus 1 reader task on core 0 (main loop runs on core 1)
  xTaskCreatePinnedToCore(bus1ReadTask, "Bus1Read", 4096, NULL, 1, NULL, 0);
}

void loop() {
  // Read bus 0 sensors directly (on core 1)
  bool any_data = false;
  bool got_data[NUM_SENSORS] = {false};

  // Phase 1: check which bus 0 sensors are ready (quick I2C checks)
  bool ready[BUS0_SENSORS] = {false};
  for (int i = 0; i < BUS0_SENSORS; i++) {
    if (sensor_active[i] && sensors[i].isDataReady()) {
      ready[i] = true;
    }
  }

  // Phase 2: read ALL ready bus 0 sensors (slow I2C reads)
  for (int i = 0; i < BUS0_SENSORS; i++) {
    if (ready[i]) {
      got_data[i] = sensors[i].getRangingData(&measurementData[i]);
      if (got_data[i]) any_data = true;
    }
  }

  // Grab fresh bus 1 data (read by core 0)
  xSemaphoreTake(bus1Mutex, portMAX_DELAY);
  for (int i = 0; i < NUM_SENSORS - BUS0_SENSORS; i++) {
    if (bus1_fresh[i]) {
      got_data[BUS0_SENSORS + i] = true;
      bus1_fresh[i] = false;
      any_data = true;
    }
  }
  xSemaphoreGive(bus1Mutex);

  if (any_data) {
    // Build entire JSON in buffer, then send all at once
    int pos = 0;
    pos += sprintf(buf + pos, "{\"sensors\":[");
    bool first = true;

    for (int i = 0; i < NUM_SENSORS; i++) {
      if (got_data[i]) {
        if (!first) buf[pos++] = ',';
        first = false;

        pos += sprintf(buf + pos, "{\"id\":%d,\"distances\":[", i);
        for (int j = 0; j < 64; j++) {
          if (j > 0) buf[pos++] = ',';
          pos += sprintf(buf + pos, "%d", (int)measurementData[i].distance_mm[j]);
        }

        pos += sprintf(buf + pos, "],\"status\":[");
        for (int j = 0; j < 64; j++) {
          if (j > 0) buf[pos++] = ',';
          pos += sprintf(buf + pos, "%d", (int)measurementData[i].target_status[j]);
        }
        pos += sprintf(buf + pos, "]}");
      }
    }

    pos += sprintf(buf + pos, "],\"quat\":[1.000000,0.000000,0.000000,0.000000],\"v\":\"%s\"}\n",
                   VERSION);

    Serial.write(buf, pos);
  }

  delay(1);
}
