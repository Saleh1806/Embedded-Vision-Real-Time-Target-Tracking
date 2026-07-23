# System Vision

## Purpose

Sirrah Vision is an embedded vision application for Raspberry Pi 5. It detects a beacon made of four LEDs and calculates its direction relative to the camera.

In practical terms, the system captures a raw image, detects the visible LEDs, calculates the beacon centre, calculates its horizontal and vertical angles, estimates its distance when at least three LEDs are visible, and sends the result to the maintenance application over TCP.

## Processing chain

```
Camera → RAW image → LED detection → angle/distance calculation → TCP
```

The detection step identifies bright points that form a four-LED square. The C++ calculator produces:

- `theta_deg`: horizontal angle;
- `phi_deg`: vertical angle;
- `distance_m`: estimated distance when 3 or 4 LEDs are visible;
- a validity state: full (4 LEDs), degraded (1 to 3 LEDs), or no detection.

The first valid beacon detection initializes the angular reference: that position is considered zero angle.

## Where is the important code?

| File | Purpose |
| --- | --- |
| `Sirrah_Camera/Camera_Acquisition/RawCapture.cpp` | Camera acquisition using `libcamera`, RAW image storage, and processing startup. |
| `Sirrah_Camera/Extraction_LED_coordinates/Image_Processing/led_centroid.py` | LED detection in the RAW image, square-geometry validation, and coordinate calculation. |
| `Sirrah_Camera/Computing_Angle/src/sra_angle_calculator.cpp` | Calculates `theta` and `phi` from LED coordinates and camera calibration data. |
| `Sirrah_Camera/Computing_Angle/include/sra_angle_calculator.hpp` | Data structures for LEDs, the camera model, and calculation results. |
| `Sirrah_Camera/Computing_Angle/app/main.cpp` | Main program: receives coordinates, manages the reference, estimates distance, and sends results. |
| `Sirrah_Camera/Computing_Angle/src/sra_angle_sender_tcp.cpp` | TCP server that formats and sends angle data to the maintenance client. |
| `Sirrah_Camera/meson.build` | Defines the C++ build and the optional camera/test components. |
| `Sirrah_Camera/build.sh` | Meson/Ninja build command. |
| `Sirrah_Camera/rpi5_cross/` | Cross-compilation and ARM64 Raspberry Pi package generation. |

The Python scripts in the repository root (`Jitter.py`, `Distance_between_centroide.py`, and others) are experimental and validation tools. They are not part of the main embedded processing chain.

## How calculations work

`led_centroid.py` reads a 16-bit RAW image, applies a brightness threshold, groups bright pixels into blobs, and retains the three or four blobs whose geometry most closely matches a square beacon.

Angle calculation uses a calibrated camera model: image size, focal lengths in pixels, and a reference position. Distance estimation compares the beacon's apparent area with a physical square beacon assumed to have 70 mm sides.

## Build

```bash
cd Sirrah_Camera
./build.sh
```

To also build camera acquisition and tests:

```bash
cd Sirrah_Camera
ENABLE_CAMERA_ACQUISITION=1 ENABLE_TESTS=1 ./build.sh
./run_test.sh
```

Camera acquisition requires `libcamera`. LED detection requires Python 3, `numpy`, and `opencv-python`.

## Key parameters

- Default image resolution: 2064 × 1552 pixels.
- Current TCP address: `XX.XX.XX.XX:8012`.
- Exposure settings: `configParam.ini`.
- Detection thresholds and geometry tolerances: at the beginning of `led_centroid.py`.
- Camera optical parameters (`Fx`, `Fy`): `Computing_Angle/app/main.cpp`.
