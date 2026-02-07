/*
 * VL53L5CX Sensor Diagnostic
 *
 * Connects to a single sensor on bus 0 (Wire) with LPn on GPIO 18.
 * Runs a series of tests to diagnose sensor health.
 *
 * Wiring:
 *   SDA -> GPIO 21, SCL -> GPIO 22
 *   LPn -> GPIO 18
 *   VIN -> 3V3, GND -> GND
 */

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define LPN_PIN 18

SparkFun_VL53L5CX sensor;

void i2cScan() {
  Serial.println("\n=== I2C Bus Scan ===");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("  Found device at 0x%02X\n", addr);
      found++;
    } else if (err == 2) {
      // NACK on address - no device (normal)
    } else if (err != 2) {
      Serial.printf("  Address 0x%02X: error %d\n", addr, err);
    }
  }
  Serial.printf("  Total devices found: %d\n", found);
}

void testLPn() {
  Serial.println("\n=== LPn Pin Test ===");

  // LPn LOW = sensor disabled (in reset)
  Serial.println("  Setting LPn LOW (sensor disabled)...");
  digitalWrite(LPN_PIN, LOW);
  delay(100);
  i2cScan();

  // LPn HIGH = sensor enabled
  Serial.println("\n  Setting LPn HIGH (sensor enabled)...");
  digitalWrite(LPN_PIN, HIGH);
  delay(100);
  i2cScan();
}

void testRawI2C() {
  Serial.println("\n=== Raw I2C Register Read ===");

  // VL53L5CX default address is 0x29
  // Try reading the device ID register pair at 0x0000-0x0001
  uint8_t addr = 0x29;

  // Write register address (16-bit)
  Wire.beginTransmission(addr);
  Wire.write(0x00);  // Register high byte
  Wire.write(0x00);  // Register low byte
  uint8_t err = Wire.endTransmission(false);  // Repeated start

  if (err != 0) {
    Serial.printf("  Write to 0x%02X failed with error %d\n", addr, err);
    Serial.println("  (0=ok, 1=too long, 2=NACK addr, 3=NACK data, 4=other, 5=timeout)");
    return;
  }

  uint8_t bytesRead = Wire.requestFrom(addr, (uint8_t)2);
  if (bytesRead == 2) {
    uint8_t id_hi = Wire.read();
    uint8_t id_lo = Wire.read();
    uint16_t device_id = (id_hi << 8) | id_lo;
    Serial.printf("  Device ID register: 0x%04X", device_id);
    if (device_id == 0xF1AA) {
      Serial.println(" (correct - VL53L5CX)");
    } else {
      Serial.println(" (unexpected!)");
    }
  } else {
    Serial.printf("  Requested 2 bytes, got %d\n", bytesRead);
  }
}

void testSensorInit() {
  Serial.println("\n=== Sensor Initialization ===");

  Serial.println("  Calling sensor.begin()...");
  unsigned long t0 = millis();
  bool ok = sensor.begin(0x29, Wire);
  unsigned long elapsed = millis() - t0;

  if (ok) {
    Serial.printf("  begin() succeeded in %lu ms\n", elapsed);
  } else {
    Serial.printf("  begin() FAILED after %lu ms\n", elapsed);
    Serial.println("  This usually means the sensor firmware upload failed.");
    Serial.println("  The VL53L5CX loads firmware from host to RAM on every boot.");
    return;
  }

  Serial.println("  Setting 8x8 resolution...");
  ok = sensor.setResolution(64);
  Serial.printf("  setResolution(64): %s\n", ok ? "OK" : "FAILED");

  Serial.println("  Setting 15 Hz ranging frequency...");
  ok = sensor.setRangingFrequency(15);
  Serial.printf("  setRangingFrequency(15): %s\n", ok ? "OK" : "FAILED");

  Serial.println("  Starting ranging...");
  ok = sensor.startRanging();
  Serial.printf("  startRanging(): %s\n", ok ? "OK" : "FAILED");

  if (!ok) return;

  // Try to get a few measurements
  Serial.println("\n=== Measurement Test (10 seconds) ===");
  int success = 0;
  int timeout = 0;
  int errors = 0;
  unsigned long start = millis();

  while (millis() - start < 10000) {
    if (sensor.isDataReady()) {
      VL53L5CX_ResultsData data;
      if (sensor.getRangingData(&data)) {
        success++;

        // Print first measurement details
        if (success <= 3) {
          Serial.printf("\n  Frame %d:\n", success);

          // Count valid zones (status 5)
          int valid = 0;
          int min_d = 99999, max_d = 0;
          for (int i = 0; i < 64; i++) {
            if (data.target_status[i] == 5) {
              valid++;
              int d = data.distance_mm[i];
              if (d < min_d) min_d = d;
              if (d > max_d) max_d = d;
            }
          }
          Serial.printf("    Valid zones: %d/64\n", valid);
          if (valid > 0) {
            Serial.printf("    Distance range: %d - %d mm\n", min_d, max_d);
          }

          // Print status distribution
          int status_counts[16] = {0};
          for (int i = 0; i < 64; i++) {
            uint8_t s = data.target_status[i];
            if (s < 16) status_counts[s]++;
          }
          Serial.print("    Status distribution:");
          for (int s = 0; s < 16; s++) {
            if (status_counts[s] > 0) {
              Serial.printf(" [%d]=%d", s, status_counts[s]);
            }
          }
          Serial.println();
        }
      } else {
        errors++;
      }
    }
    delay(10);
  }

  float elapsed_s = (millis() - start) / 1000.0;
  Serial.printf("\n  Results over %.1f seconds:\n", elapsed_s);
  Serial.printf("    Successful reads: %d (%.1f Hz)\n", success, success / elapsed_s);
  Serial.printf("    Read errors: %d\n", errors);
  Serial.printf("    Timeouts (no data ready): n/a\n");

  if (success == 0) {
    Serial.println("  WARNING: No data received. Sensor may be damaged.");
  }

  sensor.stopRanging();
}

void testI2CSpeeds() {
  Serial.println("\n=== I2C Speed Test ===");
  uint32_t speeds[] = {100000, 400000, 1000000};
  const char* names[] = {"100 kHz (standard)", "400 kHz (fast)", "1 MHz (fast+)"};

  for (int s = 0; s < 3; s++) {
    Wire.setClock(speeds[s]);
    delay(10);

    Wire.beginTransmission(0x29);
    Wire.write(0x00);
    Wire.write(0x00);
    uint8_t err = Wire.endTransmission(false);

    if (err == 0) {
      uint8_t n = Wire.requestFrom((uint8_t)0x29, (uint8_t)2);
      if (n == 2) {
        Wire.read(); Wire.read();
        Serial.printf("  %s: OK\n", names[s]);
      } else {
        Serial.printf("  %s: read failed (%d bytes)\n", names[s], n);
      }
    } else {
      Serial.printf("  %s: FAILED (error %d)\n", names[s], err);
    }
  }

  // Restore to 1 MHz
  Wire.setClock(1000000);
}

void setup() {
  Serial.begin(460800);
  delay(2000);

  Serial.println("========================================");
  Serial.println("  VL53L5CX Sensor Diagnostic");
  Serial.println("  Bus 0 (GPIO 21/22), LPn GPIO 18");
  Serial.println("========================================");

  // Initialize LPn pin
  pinMode(LPN_PIN, OUTPUT);
  digitalWrite(LPN_PIN, LOW);
  delay(10);

  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(1000000);

  // Run tests
  testLPn();
  testRawI2C();
  testI2CSpeeds();
  testSensorInit();

  Serial.println("\n========================================");
  Serial.println("  Diagnostic complete");
  Serial.println("========================================");
}

void loop() {
  // Nothing - one-shot diagnostic
  delay(1000);
}
