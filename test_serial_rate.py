#!/usr/bin/env python3
"""Test script to measure raw serial packet rate from VL53L5CX sensors."""

import argparse
import json
import time

import serial


def main():
    parser = argparse.ArgumentParser(description="Measure serial packet rate")
    parser.add_argument("--port", required=True, help="Serial port (e.g., /dev/cu.usbserial-0001)")
    parser.add_argument("--baud", type=int, default=500000, help="Baud rate (default: 500000)")
    parser.add_argument("--duration", type=int, default=10, help="Test duration in seconds (default: 10)")
    args = parser.parse_args()

    print(f"Connecting to {args.port} at {args.baud} baud...")
    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(2)  # Wait for ESP32 initialization
    ser.reset_input_buffer()
    print("Connected! Measuring packet rate...")
    print(f"Test duration: {args.duration} seconds\n")

    packet_count = 0
    error_count = 0
    start_time = time.time()
    last_print = start_time

    # Statistics
    sensor_counts = {}
    total_zones = 0

    try:
        while time.time() - start_time < args.duration:
            line = ser.readline()
            if line:
                try:
                    line_str = line.decode("utf-8", errors="ignore").strip()
                    if line_str.startswith("{"):
                        data = json.loads(line_str)

                        # Count packet
                        packet_count += 1

                        # Track sensors in this packet
                        if "sensors" in data:
                            for sensor_obj in data["sensors"]:
                                sensor_id = sensor_obj["id"]
                                sensor_counts[sensor_id] = sensor_counts.get(sensor_id, 0) + 1
                                if "distances" in sensor_obj:
                                    total_zones += len(sensor_obj["distances"])

                        # Print status every second
                        now = time.time()
                        if now - last_print >= 1.0:
                            elapsed = now - start_time
                            fps = packet_count / elapsed
                            print(f"[{elapsed:.1f}s] Packets: {packet_count}, Rate: {fps:.2f} Hz, Errors: {error_count}")
                            last_print = now

                except json.JSONDecodeError:
                    error_count += 1
                except Exception as e:
                    error_count += 1
                    print(f"Error: {e}")

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")

    # Final statistics
    end_time = time.time()
    elapsed = end_time - start_time

    print("\n" + "="*60)
    print("RESULTS")
    print("="*60)
    print(f"Duration:        {elapsed:.2f} seconds")
    print(f"Total packets:   {packet_count}")
    print(f"Packet rate:     {packet_count / elapsed:.2f} Hz")
    print(f"JSON errors:     {error_count}")
    print(f"\nSensor packets received:")
    for sensor_id in sorted(sensor_counts.keys()):
        count = sensor_counts[sensor_id]
        rate = count / elapsed
        print(f"  Sensor {sensor_id}: {count} packets ({rate:.2f} Hz)")

    if total_zones > 0:
        print(f"\nTotal zones read: {total_zones}")
        print(f"Zones per second: {total_zones / elapsed:.0f}")

    # Data throughput estimate
    avg_bytes_per_packet = ser.in_waiting / max(packet_count, 1) if packet_count > 0 else 2400
    throughput_bps = (packet_count * avg_bytes_per_packet * 8) / elapsed
    print(f"\nEstimated throughput: {throughput_bps/1000:.1f} kbps")

    ser.close()


if __name__ == "__main__":
    main()
