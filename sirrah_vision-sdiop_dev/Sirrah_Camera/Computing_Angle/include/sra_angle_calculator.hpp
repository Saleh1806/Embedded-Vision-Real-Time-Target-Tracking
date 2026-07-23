/* sra_angle_calculator.hpp
 * ----------------------------------------------------------------------------
 * Confidential file
 * Copyright (C) ARCK Sensor - All rights reserved
 * ----------------------------------------------------------------------------
 * Description:
 * This module defines the data structures and tools used to compute 
 * the horizontal and vertical viewing angles of a 4-LED beacon from 
 * its detected pixel coordinates, using a calibrated pinhole camera model.
 * ----------------------------------------------------------------------------
 * 28/11/2025 S.Diop: Initial creation
 * 01/12/2025 S.Diop: Respect Arck Sensor coding standards
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <cstdint>

namespace sirrah
{

/* LED pixel coordinates (u,v)
 * Arguments : none
 * History   : 
 *      28/11/2025 S.Diop : creation
 */
typedef struct SRA_LedPixel_s
{
    float X; // Pixel x coordinate
    float Y; // Pixel y coordinate
} SRA_LedPixel_t;

static constexpr std::uint32_t cSRA_LedCount = 4U;

/* Validity flag for angle computation
 * Arguments : none
 * History   : 
 *      28/11/2025 S.Diop : creation
 */
typedef enum SRA_ResultValidity_e
{
    cSRA_ValidityOk = 0,        // 4 LEDs used
    cSRA_ValidityDegraded = 1,  // Some LEDs missing
    cSRA_ValidityNoDetection = 2 // No LED available
} SRA_ResultValidity_t;

/* Pixel data for one 4-LED beacon
 * Arguments : none
 * History   :
 *      28/11/2025 S.Diop : creation
 */
typedef struct SRA_BeaconPixels_s
{
    float LedsUV[cSRA_LedCount][2]; // Pixel coordinates of the 4 LEDs (u,v)
    bool LedsVisible[cSRA_LedCount]{true, true, true, true}; // LED visibility flags
    bool IsVisible{true};           // Indicates if the beacon is visible
    float Datation{0.0f};           // Timestamp of the detection
} SRA_BeaconPixels_t;

/* Calibrated camera model parameters
 * Arguments : none
 * History   :
 *      28/11/2025 S.Diop : creation
 */
typedef struct SRA_CameraModel_s
{
    float Width;   // Image width in pixels
    float Height;  // Image height in pixels
    float FovXDeg; // Horizontal field of view in degrees
    float FovYDeg; // Vertical field of view in degrees
    float Fx;      // Focal length in pixels along X
    float Fy;      // Focal length in pixels along Y
    float URef;    // Reference pixel u coordinate
    float VRef;    // Reference pixel v coordinate
} SRA_CameraModel_t;

/* Result of angle computation
 * Arguments : none
 * History   : 
 *      28/11/2025 S.Diop : creation
 */
typedef struct SRA_AngleResult_s
{
    float ThetaDeg; // Horizontal angle in degrees
    float PhiDeg;   // Vertical angle in degrees
    bool IsValid;   // Indicates if the angle computation is valid
    SRA_ResultValidity_t Validity; // Status: OK/Degraded/NoDetection
    std::uint32_t VisibleLedCount;   // Number of LEDs used for computation
    bool LedsUsed[cSRA_LedCount];  // Flags for each LED contribution
} SRA_AngleResult_t;

/* Angle computation helper using pinhole camera model
 * Arguments : CameraModel -> calibrated camera parameters
 * History   :
 *      28/11/2025 S.Diop : creation
 */
class SRA_AngleCalculator
{
public:
    explicit SRA_AngleCalculator(const SRA_CameraModel_t& CameraModel);

    /* Compute angles from the 4 LED centers of one beacon
     * Arguments : Pixels -> pixel coordinates and visibility of the beacon
     * History   : 
     *      28/11/2025 S.Diop : creation
     */
    SRA_AngleResult_t Compute(const SRA_BeaconPixels_t& Pixels) const;

private:
    SRA_CameraModel_t CameraModelData;  
};

} // namespace sirrah
