# C++ angle computation from LED centers

This subproject contains a minimal C++ version of the angle computation using the detected pixel coordinates of the four LEDs of a beacon.

## Layout
- `include/`: public headers (`sra_angle_calculator.hpp`).
- `src/`: implementations.
- `app/`: entry point with test data.

## Computation idea
1. Barycenter of the 4 LEDs to locate the tag in the image.
2. Normalization around the image center to obtain Px/Py in [-1;1].
3. Conversion to angles using the camera field of view (linear model; bilinear or calibration table could be added if needed).

## Demo
`app/main.cpp` generates LEDs corresponding to a target angle and checks that the computation retrieves those values.
