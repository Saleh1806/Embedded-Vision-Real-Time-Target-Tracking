/* sra_angle_calculator.cpp
 * ----------------------------------------------------------------------------
 * Confidential file
 * Copyright (C) ARCK Sensor - All rights reserved
 * ----------------------------------------------------------------------------
 * Description:
 * Angle computation from LED pixel centers.
 * ----------------------------------------------------------------------------
 * 28/11/2025 S.Diop: Initial creation
 * 01/12/2025 S.Diop: Respect Arck Sensor coding standards
 * ----------------------------------------------------------------------------
 */

#include "sra_angle_calculator.hpp"

#include <cmath>
#include <array>

namespace sirrah
{

/* Build calculator with calibrated camera model
 * Arguments : CameraModel -> calibrated camera parameters
 * History   : 
 *      28/11/2025 S.Diop : creation
 */
SRA_AngleCalculator::SRA_AngleCalculator(const SRA_CameraModel_t& CameraModel)
    : CameraModelData(CameraModel)
{
}

/* Compute viewing angles of a beacon from pixel coordinates
 * Arguments : Pixels -> pixel coordinates and visibility of the beacon
 * History   : 
 *      28/11/2025 S.Diop : creation
 */
SRA_AngleResult_t SRA_AngleCalculator::Compute(const SRA_BeaconPixels_t& Beacons) const
{
    SRA_AngleResult_t Result{};
    Result.Validity = cSRA_ValidityNoDetection;
    Result.IsValid = false;
    Result.ThetaDeg = 0.0f;
    Result.PhiDeg = 0.0f;
    Result.VisibleLedCount = 0;
    for (std::uint32_t LedIndex = 0; LedIndex < cSRA_LedCount; ++LedIndex)
    {
        Result.LedsUsed[LedIndex] = false;
    }

    if (!Beacons.IsVisible)
    {
        return Result;
    }

    // 1) Compute the barycenter from visible LEDs.
    float SumX = 0.0f;
    float SumY = 0.0f;
    std::uint32_t ValidLedCount = 0;
    std::array<std::array<float, 2>, cSRA_LedCount> VisibleLeds{};
    for (std::uint32_t LedIndex = 0; LedIndex < cSRA_LedCount; ++LedIndex)
    {
        if (!Beacons.LedsVisible[LedIndex])
        {
            continue;
        }
        const float LedX = Beacons.LedsUV[LedIndex][0];
        const float LedY = Beacons.LedsUV[LedIndex][1];
        SumX += LedX;
        SumY += LedY;
        VisibleLeds[ValidLedCount][0] = LedX;
        VisibleLeds[ValidLedCount][1] = LedY;
        Result.LedsUsed[LedIndex] = true;
        ++ValidLedCount;
    }
    if (ValidLedCount == 0U)
    {
        return Result;
    }

    float U = SumX / static_cast<float>(ValidLedCount);
    float V = SumY / static_cast<float>(ValidLedCount);

    // For 3 LEDs, estimate center as midpoint of the farthest pair
    // (assumed to be the square diagonal).
    if (ValidLedCount == 3U)
    {
        float MaxDistSq = -1.0f;
        std::uint32_t IMax = 0U;
        std::uint32_t JMax = 1U;
        for (std::uint32_t I = 0; I < 3U; ++I)
        {
            for (std::uint32_t J = I + 1U; J < 3U; ++J)
            {
                const float Dx = VisibleLeds[I][0] - VisibleLeds[J][0];
                const float Dy = VisibleLeds[I][1] - VisibleLeds[J][1];
                const float DistSq = (Dx * Dx) + (Dy * Dy);
                if (DistSq > MaxDistSq)
                {
                    MaxDistSq = DistSq;
                    IMax = I;
                    JMax = J;
                }
            }
        }
        U = 0.5f * (VisibleLeds[IMax][0] + VisibleLeds[JMax][0]);
        V = 0.5f * (VisibleLeds[IMax][1] + VisibleLeds[JMax][1]);
    }

    // 2) Compute angle 
    const float CenterX = CameraModelData.Width * 0.5f;
    const float CenterY = CameraModelData.Height * 0.5f;
    const float NormalizedXRef = (CameraModelData.URef - CenterX) / CameraModelData.Fx;
    const float NormalizedYRef = (CameraModelData.VRef - CenterY) / CameraModelData.Fy;
    const float NormalizedX = (U - CenterX) / CameraModelData.Fx;
    const float NormalizedY = (V - CenterY) / CameraModelData.Fy;
    const float ThetaRad = std::atan(NormalizedX) - std::atan(NormalizedXRef);
    const float PhiRad = std::atan(-NormalizedY) - std::atan(-NormalizedYRef);
    constexpr float cPi = 3.14159265358979323846f;
    const float ThetaDeg = ThetaRad * 180.0f / cPi;
    const float PhiDeg = PhiRad * 180.0f / cPi;

    // 3) Prepare result
    Result.ThetaDeg = static_cast<float>(ThetaDeg);
    Result.PhiDeg = static_cast<float>(PhiDeg);
    Result.IsValid = true;
    Result.VisibleLedCount = ValidLedCount;
    Result.Validity = (ValidLedCount == cSRA_LedCount) ? cSRA_ValidityOk : cSRA_ValidityDegraded;
    return Result;
}

} // namespace sirrah
