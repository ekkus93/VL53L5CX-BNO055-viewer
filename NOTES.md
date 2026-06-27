# Project Notes

## Big picture

A **real-time 3D point-cloud viewer** for a cheap homemade depth scanner. An **ESP32-S3**
reads two I²C sensors — a **VL53L5CX** (an 8×8 = 64-zone time-of-flight depth sensor)
and a **BNO055 IMU** (orientation tracker) — and streams the data as JSON over USB
serial. A Python app parses that stream and renders it live in your browser as a 3D
scene using [viser](https://viser.studio).

Data flow: **ESP32 firmware → serial JSON → background reader thread → main render loop
→ viser browser scene.**

## The two halves

### 1. Firmware (`firmware/vl53l5cx_reader/vl53l5cx_reader.ino`)

Arduino/ESP32 sketch. Configures the ToF sensor for 8×8 @ 15 Hz and the BNO055 IMU in
IMUPLUS mode (accel+gyro fusion, no magnetometer, so it's immune to magnetic
interference). Each frame it emits one JSON line:

```json
{"distances":[...64 mm values...],"status":[...64...],"quat":[w,x,y,z],"v":"0.1.0"}
```

- `status == 5` means a valid measurement
- `quat` is the IMU orientation
- `v` is a version string the viewer checks against its own to warn about stale firmware

Hardware notes: runs on an **ESP32-S3** (native USB, enumerates as `/dev/ttyACM0`).
I²C SDA/SCL are on GPIO21/47 and the VL53L5CX LPn on GPIO48 — GPIO19/20 are reserved
for the S3's USB and GPIO22 doesn't exist, so the classic-ESP32 pins can't be reused.
The bus runs at 400 kHz (the BNO055's max). Built with `USBMode=default,CDCOnBoot=cdc`,
and the board needs a manual RESET tap to launch the app after flashing.

### 2. Python viewer (`viewer/`)

Run with `python -m viewer`. Broken into clean modules:

| File | Role |
|------|------|
| `serial_reader.py` | Background thread reading/validating the serial stream (handles corrupt lines, NaN/Inf, auto-reconnect, tracks data FPS). Thread-safe snapshot via `get_data()`. |
| `geometry.py` | The math core — converts 64 distances into 3D points. |
| `viewer.py` | The main app: render loop, GUI controls, IMU transforms, mapping mode. |
| `scene.py` | Builds the viser 3D scene (board meshes, grid, zone rays). |
| `filters.py` | Temporal smoothing + plane fitting. |
| `config.py` | All constants: sensor specs, board geometry, ST calibration tables. |

## The interesting bits

### Distance → 3D conversion (`geometry.py`)

The VL53L5CX reports *perpendicular* (z-axis) distance per zone, not radial. To place
each zone in 3D it needs each zone's viewing angle, and there are **two methods** you
can toggle:

- **Uniform Grid** — assumes evenly-spaced angles across the 65° FoV (ideal pinhole model).
- **ST Lookup Table** — uses ST's published per-zone pitch/yaw calibration tables
  (`config.py`), accounting for the real lens's non-uniform angular coverage. More accurate.

### IMU orientation (`viewer.py:_corrected_imu_quat`)

The BNO055 quaternion rotates the virtual sensor frame so the on-screen cloud matches how
you physically hold the rig. Orientation is shown **relative to a captured reference pose**:
`_corrected_imu_quat` computes `delta = current × reference⁻¹`, where the reference is set by
the **Zero Orientation** button (or auto-captured on the first IMU frame). Holding the sensor
in any pose and clicking Zero makes that pose "neutral".

This replaces the old hard-coded `correct_imu_to_tof_frame` correction (which assumed a fixed
90° BNO08x mounting) — the relative-orientation approach works regardless of how the IMU is
mounted or which chip's axis convention it uses. `_update_scene_transforms` then applies the
resulting quaternion to the board frames.

### Mapping mode (`viewer.py` `MappingState`)

Normally it shows just the live frame in sensor-local coordinates. In mapping mode it
transforms each frame into *world* coordinates using the IMU rotation and accumulates
points over time to build up a 3D map — with voxel downsampling to keep the point count
bounded.

### Plane fitting (`filters.py`)

Fits a plane to the valid points to detect flat surfaces, either least-squares or
**RANSAC** (robust to outliers), and reports the fit quality as RMSE in mm. The RANSAC
seed is derived from the data so results are reproducible.

### Color

Maps distance to a blue→red gradient; invalid zones are gray.

## What you see in the browser

A 3D scene at `localhost:8081` with: textured mini-meshes of the two sensor boards, a
reference grid, 64 "zone rays" fanning out from the sensor (optionally clipped to the
measured distance), the live colored point cloud, and a GUI panel for point size, the
coordinate method, IMU on/off, the Zero Orientation button, temporal filtering, plane
fitting, and mapping controls.

It's a polished, well-factored hobby-electronics project — clear separation between I/O,
math, and rendering, with thoughtful robustness (serial validation, reconnection,
version checks).
