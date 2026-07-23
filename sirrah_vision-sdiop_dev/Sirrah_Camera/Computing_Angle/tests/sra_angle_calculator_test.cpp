/* sra_angle_calculator.cpp
 * ----------------------------------------------------------------------------
 * Confidential file
 * Copyright (C) ARCK Sensor - All rights reserved
 * ----------------------------------------------------------------------------
 * Description:
 * Tests Angle computation.
 * ----------------------------------------------------------------------------
 * 04/12/2025 S.Diop: Initial creation
 * ----------------------------------------------------------------------------
 */

#include "sra_angle_calculator.hpp"
#include "gtest/gtest.h"
#include <array>
#include <cmath>

using namespace sirrah;

namespace
{

/* Helper to create a beacon with given pixel position and visibility
 * Arguments : U -> pixel u coordinate
 *             V -> pixel v coordinate
 *             ledsVisible -> visibility flags for the 4 LEDs
 * History   :
 *      04/12/2025 : creation
 */
SRA_BeaconPixels_t CreateBeacon(float U, float V, bool ledsVisible[cSRA_LedCount])
{
    SRA_BeaconPixels_t beacon{};
    beacon.IsVisible = true;
    for (std::uint32_t i = 0; i < cSRA_LedCount; ++i)
    {
        beacon.LedsUV[i][0] = U;
        beacon.LedsUV[i][1] = V;
        beacon.LedsVisible[i] = ledsVisible[i];
    }
    return beacon;
}
} // namespace

TEST (SraAngleCalculator, Shift_Left_led_001)
{
    const SRA_CameraModel_t cam{1456.0f, 1088.0f, 30.0f, 23.61f, 2717.14f, 2602.63f, 845.3f, 614.8f};
    SRA_AngleCalculator calc(cam);

    bool ledsVisible[cSRA_LedCount] = {true, true, true, true};
    SRA_BeaconPixels_t beacon = CreateBeacon(845.3f + 0.4356f, 614.8f + 0.0929f, ledsVisible);
    beacon.IsVisible = true; 
    const float expectedThetaDeg = 0.01f;
    const float expectedPhiDeg = 0.0001f;
    const SRA_AngleResult_t res = calc.Compute(beacon);

    
    EXPECT_TRUE(res.IsValid);
    EXPECT_NEAR(res.ThetaDeg, expectedThetaDeg, 1e-1f);
    EXPECT_NEAR(res.PhiDeg, expectedPhiDeg, 1e-1f);

    std::cout << "Expected Theta: " << expectedThetaDeg << " deg\n";
    std::cout << "Computed Theta: " << res.ThetaDeg << " deg\n";
    std::cout << "Expected Phi  : " << expectedPhiDeg << " deg\n";
    std::cout << "Computed Phi  : " <<  res.PhiDeg << " deg\n";
}
TEST (SraAngleCalculator, Shift_Right_led_001)
{
    const SRA_CameraModel_t cam{1456.0f, 1088.0f, 30.0f, 23.61f, 2717.14f, 2602.63f, 845.3f, 614.8f};
    SRA_AngleCalculator calc(cam);

    bool ledsVisible[cSRA_LedCount] = {true, true, true, true};
    SRA_BeaconPixels_t beacon = CreateBeacon(845.3f - 0.5039f, 614.8f + 0.0029f, ledsVisible);
    beacon.IsVisible = true; 
    float expectedThetaDeg = -0.01f;
    // float expectedPhiDeg = 0.0001f;
    const SRA_AngleResult_t res = calc.Compute(beacon);

    EXPECT_TRUE(res.IsValid);
    EXPECT_NEAR(res.ThetaDeg, expectedThetaDeg, 1e-1f);
    // EXPECT_NEAR(res.PhiDeg, expectedPhiDeg, 1e-3f);

    std::cout << "Expected Theta: " << expectedThetaDeg << " deg\n";
    std::cout << "Computed Theta: " << res.ThetaDeg << " deg\n";
    // std::cout << "Expected Phi  : " << expectedPhiDeg << " deg\n";
    // std::cout << "Computed Phi  : " <<  res.PhiDeg << " deg\n";
}
TEST (SraAngleCalculator, Shift_Left_led_002)
{
    const SRA_CameraModel_t cam{1456.0f, 1088.0f, 30.0f, 23.61f, 2717.14f, 2602.63f, 845.3f, 614.8f};
    SRA_AngleCalculator calc(cam);

    bool ledsVisible[cSRA_LedCount] = {true, true, true, true};
    SRA_BeaconPixels_t beacon = CreateBeacon(845.3f + 0.9811f, 614.8f + 0.0244f, ledsVisible);
    beacon.IsVisible = true; 
    float expectedThetaDeg = 0.02f;
    // float expectedPhiDeg = 0.0001f;
    const SRA_AngleResult_t res = calc.Compute(beacon);

    
    EXPECT_TRUE(res.IsValid);
    EXPECT_NEAR(res.ThetaDeg, expectedThetaDeg, 1e-1f);
    // EXPECT_NEAR(res.PhiDeg, expectedPhiDeg, 1e-3f);

    std::cout << "Expected Theta: " << expectedThetaDeg << " deg\n";
    std::cout << "Computed Theta: " << res.ThetaDeg << " deg\n";
    // std::cout << "Expected Phi  : " << expectedPhiDeg << " deg\n";
    // std::cout << "Computed Phi  : " <<  res.PhiDeg << " deg\n";
}
TEST (SraAngleCalculator, Shift_Right_led_002)
{
    const SRA_CameraModel_t cam{1456.0f, 1088.0f, 30.0f, 23.61f, 2717.14f, 2602.63f, 845.3f, 614.8f};
    SRA_AngleCalculator calc(cam);

    bool ledsVisible[cSRA_LedCount] = {true, true, true, true};
    SRA_BeaconPixels_t beacon = CreateBeacon(845.3f - 1.0253f, 614.8f + 0.1090f, ledsVisible);
    beacon.IsVisible = true; 
    float expectedThetaDeg = -0.02f;
    // float expectedPhiDeg = 0.0001f;
    const SRA_AngleResult_t res = calc.Compute(beacon);

    EXPECT_TRUE(res.IsValid);
    EXPECT_NEAR(res.ThetaDeg, expectedThetaDeg, 1e-1f);
    // EXPECT_NEAR(res.PhiDeg, expectedPhiDeg, 1e-3f);

    std::cout << "Expected Theta: " << expectedThetaDeg << " deg\n";
    std::cout << "Computed Theta: " << res.ThetaDeg << " deg\n";
    // std::cout << "Expected Phi  : " << expectedPhiDeg << " deg\n";
    // std::cout << "Computed Phi  : " <<  res.PhiDeg << " deg\n";
}

TEST (SraAngleCalculator, Shift_Left_LED_003)
{
    const SRA_CameraModel_t cam{1456.0f, 1088.0f, 30.0f, 23.61f, 2717.14f, 2602.63f, 845.3f, 614.8f};
    SRA_AngleCalculator calc(cam);

    bool ledsVisible[cSRA_LedCount] = {true, true, true, true};
    SRA_BeaconPixels_t beacon = CreateBeacon(845.3f + 1.3526f, 614.8f + 0.0117f, ledsVisible);
    beacon.IsVisible = true; 
    float expectedThetaDeg = 0.03f;
    // float expectedPhiDeg = 0.0001f;
    const SRA_AngleResult_t res = calc.Compute(beacon);

    
    EXPECT_TRUE(res.IsValid);
    EXPECT_NEAR(res.ThetaDeg, expectedThetaDeg, 1e-1f);
    // EXPECT_NEAR(res.PhiDeg, expectedPhiDeg, 1e-3f);

    std::cout << "Expected Theta: " << expectedThetaDeg << " deg\n";
    std::cout << "Computed Theta: " << res.ThetaDeg << " deg\n";
    // std::cout << "Expected Phi  : " << expectedPhiDeg << " deg\n";
    // std::cout << "Computed Phi  : " <<  res.PhiDeg << " deg\n";
}
TEST (SraAngleCalculator, Shift_Right_LED_003)
{
    const SRA_CameraModel_t cam{1456.0f, 1088.0f, 30.0f, 23.61f, 2717.14f, 2602.63f, 845.3f, 614.8f};
    SRA_AngleCalculator calc(cam);

    bool ledsVisible[cSRA_LedCount] = {true, true, true, true};
    SRA_BeaconPixels_t beacon = CreateBeacon(845.3f - 1.5363f, 614.8f + 0.1113f, ledsVisible);
    beacon.IsVisible = true; 
    float expectedThetaDeg = -0.03f;
    // float expectedPhiDeg = 0.0001f;
    const SRA_AngleResult_t res = calc.Compute(beacon);

    EXPECT_TRUE(res.IsValid);
    EXPECT_NEAR(res.ThetaDeg, expectedThetaDeg, 1e-1f);
    // EXPECT_NEAR(res.PhiDeg, expectedPhiDeg, 1e-3f);

    std::cout << "Expected Theta: " << expectedThetaDeg << " deg\n";
    std::cout << "Computed Theta: " << res.ThetaDeg << " deg\n";
    // std::cout << "Expected Phi  : " << expectedPhiDeg << " deg\n";
    // std::cout << "Computed Phi  : " <<  res.PhiDeg << " deg\n";
}
TEST (SraAngleCalculator, Shift_Left_Led_004)
{
    const SRA_CameraModel_t cam{1456.0f, 1088.0f, 30.0f, 23.61f, 2717.14f, 2602.63f, 845.3f, 614.8f};
    SRA_AngleCalculator calc(cam);

    bool ledsVisible[cSRA_LedCount] = {true, true, true, true};
    SRA_BeaconPixels_t beacon = CreateBeacon(845.3f + 1.9221f, 614.8f + 0.0151f, ledsVisible);
    beacon.IsVisible = true; 
    float expectedThetaDeg = 0.04f;
    // float expectedPhiDeg = 0.0001f;
    const SRA_AngleResult_t res = calc.Compute(beacon);

    
    EXPECT_TRUE(res.IsValid);
    EXPECT_NEAR(res.ThetaDeg, expectedThetaDeg, 1e-1f);
    // EXPECT_NEAR(res.PhiDeg, expectedPhiDeg, 1e-3f);

    std::cout << "Expected Theta: " << expectedThetaDeg << " deg\n";
    std::cout << "Computed Theta: " << res.ThetaDeg << " deg\n";
    // std::cout << "Expected Phi  : " << expectedPhiDeg << " deg\n";
    // std::cout << "Computed Phi  : " <<  res.PhiDeg << " deg\n";
}
TEST (SraAngleCalculator, Shift_Right_Led_004)
{
    const SRA_CameraModel_t cam{1456.0f, 1088.0f, 30.0f, 23.61f, 2717.14f, 2602.63f, 845.3f, 614.8f};
    SRA_AngleCalculator calc(cam);

    bool ledsVisible[cSRA_LedCount] = {true, true, true, true};
    SRA_BeaconPixels_t beacon = CreateBeacon(845.3f - 1.8927f, 614.8f - 0.0054f, ledsVisible);
    beacon.IsVisible = true; 
    float expectedThetaDeg = -0.04f;
    // float expectedPhiDeg = 0.0001f;
    const SRA_AngleResult_t res = calc.Compute(beacon);

    EXPECT_TRUE(res.IsValid);
    EXPECT_NEAR(res.ThetaDeg, expectedThetaDeg, 1e-1f);
    // EXPECT_NEAR(res.PhiDeg, expectedPhiDeg, 1e-3f);

    std::cout << "Expected Theta: " << expectedThetaDeg << " deg\n";
    std::cout << "Computed Theta: " << res.ThetaDeg << " deg\n";
    // std::cout << "Expected Phi  : " << expectedPhiDeg << " deg\n";
    // std::cout << "Computed Phi  : " <<  res.PhiDeg << " deg\n";
}
TEST (SraAngleCalculator, Shift_Left_Led_01)
{
    const SRA_CameraModel_t cam{1456.0f, 1088.0f, 30.0f, 23.61f, 2717.14f, 2602.63f, 845.3f, 614.8f};
    SRA_AngleCalculator calc(cam);

    bool ledsVisible[cSRA_LedCount] = {true, true, true, true};
    SRA_BeaconPixels_t beacon = CreateBeacon(845.3f + 4.6688f, 614.8f + 0.0465f, ledsVisible);
    beacon.IsVisible = true; 
    float expectedThetaDeg = 0.1f;
    // float expectedPhiDeg = 0.0001f;
    const SRA_AngleResult_t res = calc.Compute(beacon);

    
    EXPECT_TRUE(res.IsValid);
    EXPECT_NEAR(res.ThetaDeg, expectedThetaDeg, 1e-1f);
    // EXPECT_NEAR(res.PhiDeg, expectedPhiDeg, 1e-3f);

    std::cout << "Expected Theta: " << expectedThetaDeg << " deg\n";
    std::cout << "Computed Theta: " << res.ThetaDeg << " deg\n";
    // std::cout << "Expected Phi  : " << expectedPhiDeg << " deg\n";
    // std::cout << "Computed Phi  : " <<  res.PhiDeg << " deg\n";
}
TEST (SraAngleCalculator, Shift_Right_Led_01)
{
    const SRA_CameraModel_t cam{1456.0f, 1088.0f, 30.0f, 23.61f, 2717.14f, 2602.63f, 845.3f, 614.8f};
    SRA_AngleCalculator calc(cam);

    bool ledsVisible[cSRA_LedCount] = {true, true, true, true};
    SRA_BeaconPixels_t beacon = CreateBeacon(845.3f - 4.8046f, 614.8f - 0.0448f, ledsVisible);
    beacon.IsVisible = true; 
    float expectedThetaDeg = -0.1f;
    // float expectedPhiDeg = 0.0001f;
    const SRA_AngleResult_t res = calc.Compute(beacon);

    EXPECT_TRUE(res.IsValid);
    EXPECT_NEAR(res.ThetaDeg, expectedThetaDeg, 1e-1f);
    // EXPECT_NEAR(res.PhiDeg, expectedPhiDeg, 1e-3f);

    std::cout << "Expected Theta: " << expectedThetaDeg << " deg\n";
    std::cout << "Computed Theta: " << res.ThetaDeg << " deg\n";
    // std::cout << "Expected Phi  : " << expectedPhiDeg << " deg\n";
    // std::cout << "Computed Phi  : " <<  res.PhiDeg << " deg\n";
}
TEST (SraAngleCalculator, Shift_Left_Led_1)
{
    const SRA_CameraModel_t cam{1456.0f, 1088.0f, 30.0f, 23.61f, 2717.14f, 2602.63f, 845.3f, 614.8f};
    SRA_AngleCalculator calc(cam);

    bool ledsVisible[cSRA_LedCount] = {true, true, true, true};
    SRA_BeaconPixels_t beacon = CreateBeacon(845.3f + 47.6485f, 614.8f -0.7638f, ledsVisible);
    beacon.IsVisible = true; 
    float expectedThetaDeg = 1.0f;
    // float expectedPhiDeg = 0.0001f;
    const SRA_AngleResult_t res = calc.Compute(beacon);

    
    EXPECT_TRUE(res.IsValid);
    EXPECT_NEAR(res.ThetaDeg, expectedThetaDeg, 1e-1f);
    // EXPECT_NEAR(res.PhiDeg, expectedPhiDeg, 1e-3f);

    std::cout << "Expected Theta: " << expectedThetaDeg << " deg\n";
    std::cout << "Computed Theta: " << res.ThetaDeg << " deg\n";
    // std::cout << "Expected Phi  : " << expectedPhiDeg << " deg\n";
    // std::cout << "Computed Phi  : " <<  res.PhiDeg << " deg\n";
}
TEST (SraAngleCalculator, Shift_Right_Led_1)
{
    const SRA_CameraModel_t cam{1456.0f, 1088.0f, 30.0f, 23.61f, 2717.14f, 2602.63f, 845.3f, 614.8f};
    SRA_AngleCalculator calc(cam);

    bool ledsVisible[cSRA_LedCount] = {true, true, true, true};
    SRA_BeaconPixels_t beacon = CreateBeacon(845.3f - 48.7123f, 614.8f + 0.8234f, ledsVisible);
    beacon.IsVisible = true; 
    float expectedThetaDeg = -1.0f;
    // float expectedPhiDeg = 0.0001f;
    const SRA_AngleResult_t res = calc.Compute(beacon);

    EXPECT_TRUE(res.IsValid);
    EXPECT_NEAR(res.ThetaDeg, expectedThetaDeg, 1e-1f);
    // EXPECT_NEAR(res.PhiDeg, expectedPhiDeg, 1e-3f);

    std::cout << "Expected Theta: " << expectedThetaDeg << " deg\n";
    std::cout << "Computed Theta: " << res.ThetaDeg << " deg\n";
    // std::cout << "Expected Phi  : " << expectedPhiDeg << " deg\n";
    // std::cout << "Computed Phi  : " <<  res.PhiDeg << " deg\n";
}

TEST (SraAngleCalculator, Shift_Left_Led_2)
{
    const SRA_CameraModel_t cam{1456.0f, 1088.0f, 30.0f, 23.61f, 2717.14f, 2602.63f, 845.3f, 614.8f};
    SRA_AngleCalculator calc(cam);

    bool ledsVisible[cSRA_LedCount] = {true, true, true, true};
    SRA_BeaconPixels_t beacon = CreateBeacon(845.3f + 96.0912f, 614.8f - 2.0108f, ledsVisible);
    beacon.IsVisible = true; 
    float expectedThetaDeg = 2.0f;
    // float expectedPhiDeg = 0.0001f;
    const SRA_AngleResult_t res = calc.Compute(beacon);

    
    EXPECT_TRUE(res.IsValid);
    EXPECT_NEAR(res.ThetaDeg, expectedThetaDeg, 1e-1f);
    // EXPECT_NEAR(res.PhiDeg, expectedPhiDeg, 1e-3f);

    std::cout << "Expected Theta: " << expectedThetaDeg << " deg\n";
    std::cout << "Computed Theta: " << res.ThetaDeg << " deg\n";
    // std::cout << "Expected Phi  : " << expectedPhiDeg << " deg\n";
    // std::cout << "Computed Phi  : " <<  res.PhiDeg << " deg\n";
}
TEST (SraAngleCalculator, Shift_Right_Led_2)
{
    const SRA_CameraModel_t cam{1456.0f, 1088.0f, 30.0f, 23.61f, 2717.14f, 2602.63f, 845.3f, 614.8f};
    SRA_AngleCalculator calc(cam);

    bool ledsVisible[cSRA_LedCount] = {true, true, true, true};
    SRA_BeaconPixels_t beacon = CreateBeacon(845.3f - 98.0392f, 614.8f + 0.2054f, ledsVisible);
    beacon.IsVisible = true; 
    float expectedThetaDeg = -2.0f;
    // float expectedPhiDeg = 0.0001f;
    const SRA_AngleResult_t res = calc.Compute(beacon);

    EXPECT_TRUE(res.IsValid);
    EXPECT_NEAR(res.ThetaDeg, expectedThetaDeg, 1e-1f);
    // EXPECT_NEAR(res.PhiDeg, expectedPhiDeg, 1e-3f);

    std::cout << "Expected Theta: " << expectedThetaDeg << " deg\n";
    std::cout << "Computed Theta: " << res.ThetaDeg << " deg\n";
    // std::cout << "Expected Phi  : " << expectedPhiDeg << " deg\n";
    // std::cout << "Computed Phi  : " <<  res.PhiDeg << " deg\n";
}
TEST (SraAngleCalculator, Shift_Left_Led_4)
{
    const SRA_CameraModel_t cam{1456.0f, 1088.0f, 30.0f, 23.61f, 2717.14f, 2602.63f, 845.3f, 614.8f};
    SRA_AngleCalculator calc(cam);

    bool ledsVisible[cSRA_LedCount] = {true, true, true, true};
    SRA_BeaconPixels_t beacon = CreateBeacon(845.3f + 192.2136f, 614.8f - 4.4341f, ledsVisible);
    beacon.IsVisible = true; 
    float expectedThetaDeg = 4.0f;
    // float expectedPhiDeg = 0.0001f;
    const SRA_AngleResult_t res = calc.Compute(beacon);

    
    EXPECT_TRUE(res.IsValid);
    EXPECT_NEAR(res.ThetaDeg, expectedThetaDeg, 1e-1f);
    // EXPECT_NEAR(res.PhiDeg, expectedPhiDeg, 1e-3f);

    std::cout << "Expected Theta: " << expectedThetaDeg << " deg\n";
    std::cout << "Computed Theta: " << res.ThetaDeg << " deg\n";
    // std::cout << "Expected Phi  : " << expectedPhiDeg << " deg\n";
    // std::cout << "Computed Phi  : " <<  res.PhiDeg << " deg\n";
}
TEST (SraAngleCalculator, Shift_Right_Led_4)
{
    const SRA_CameraModel_t cam{1456.0f, 1088.0f, 30.0f, 23.61f, 2717.14f, 2602.63f, 845.3f, 614.8f};
    SRA_AngleCalculator calc(cam);

    bool ledsVisible[cSRA_LedCount] = {true, true, true, true};
    SRA_BeaconPixels_t beacon = CreateBeacon(845.3f - 198.1689f, 614.8f + 0.3132f, ledsVisible);
    beacon.IsVisible = true; 
    float expectedThetaDeg = -4.0f;
    // float expectedPhiDeg = 0.0001f;
    const SRA_AngleResult_t res = calc.Compute(beacon);

    EXPECT_TRUE(res.IsValid);
    EXPECT_NEAR(res.ThetaDeg, expectedThetaDeg, 1e-1f);
    // EXPECT_NEAR(res.PhiDeg, expectedPhiDeg, 1e-3f);

    std::cout << "Expected Theta: " << expectedThetaDeg << " deg\n";
    std::cout << "Computed Theta: " << res.ThetaDeg << " deg\n";
    //std::cout << "Expected Phi  : " << expectedPhiDeg << " deg\n";
    //std::cout << "Computed Phi  : " <<  res.PhiDeg << " deg\n";
}
TEST (SraAngleCalculator, Shift_High_Led_1)
{
    const SRA_CameraModel_t cam{1456.0f, 1088.0f, 30.0f, 23.61f, 2717.14f, 2602.63f, 845.3f, 614.8f};
    SRA_AngleCalculator calc(cam);

    bool ledsVisible[cSRA_LedCount] = {true, true, true, true};
    SRA_BeaconPixels_t beacon = CreateBeacon(845.3f - 1.9390f, 614.8f - 49.1141f, ledsVisible);
    beacon.IsVisible = true; 
    float expectedThetaDeg = 0.0001f;
    float expectedPhiDeg = 1.0f;
    const SRA_AngleResult_t res = calc.Compute(beacon);

    EXPECT_TRUE(res.IsValid);
    //EXPECT_NEAR(res.ThetaDeg, expectedThetaDeg, 1e-1f);
    EXPECT_NEAR(res.PhiDeg, expectedPhiDeg, 1e-1f);

    //std::cout << "Expected Theta: " << expectedThetaDeg << " deg\n";
    //std::cout << "Computed Theta: " << res.ThetaDeg << " deg\n";
    std::cout << "Expected Phi  : " << expectedPhiDeg << " deg\n";
    std::cout << "Computed Phi  : " <<  res.PhiDeg << " deg\n";
}

TEST (SraAngleCalculator, Shift_Down_Led_1)
{
    const SRA_CameraModel_t cam{1456.0f, 1088.0f, 30.0f, 23.61f, 2717.14f, 2602.63f, 845.3f, 614.8f};
    SRA_AngleCalculator calc(cam);

    bool ledsVisible[cSRA_LedCount] = {true, true, true, true};
    SRA_BeaconPixels_t beacon = CreateBeacon(845.3f - 1.0844f, 614.8f + 55.0095f, ledsVisible);
    beacon.IsVisible = true; 
    float expectedThetaDeg = 0.0001f;
    float expectedPhiDeg = -1.0f;
    const SRA_AngleResult_t res = calc.Compute(beacon);

    EXPECT_TRUE(res.IsValid);
    //EXPECT_NEAR(res.ThetaDeg, expectedThetaDeg, 1e-1f);
    EXPECT_NEAR(res.PhiDeg, expectedPhiDeg, 1e-1f);

    //std::cout << "Expected Theta: " << expectedThetaDeg << " deg\n";
    //std::cout << "Computed Theta: " << res.ThetaDeg << " deg\n";
    std::cout << "Expected Phi  : " << expectedPhiDeg << " deg\n";
    std::cout << "Computed Phi  : " <<  res.PhiDeg << " deg\n";
}

TEST (SraAngleCalculator, Shift_High_Led_2)
{
    const SRA_CameraModel_t cam{1456.0f, 1088.0f, 30.0f, 23.61f, 2717.14f, 2602.63f, 845.3f, 614.8f};
    SRA_AngleCalculator calc(cam);

    bool ledsVisible[cSRA_LedCount] = {true, true, true, true};
    SRA_BeaconPixels_t beacon = CreateBeacon(845.3f - 1.5052f, 614.8f - 94.9947f, ledsVisible);
    beacon.IsVisible = true; 
    float expectedThetaDeg = 0.0001f;
    float expectedPhiDeg = 2.0f;
    const SRA_AngleResult_t res = calc.Compute(beacon);

    EXPECT_TRUE(res.IsValid);
    //EXPECT_NEAR(res.ThetaDeg, expectedThetaDeg, 1e-1f);
    EXPECT_NEAR(res.PhiDeg, expectedPhiDeg, 1e-1f);

    //std::cout << "Expected Theta: " << expectedThetaDeg << " deg\n";
    //std::cout << "Computed Theta: " << res.ThetaDeg << " deg\n";
    std::cout << "Expected Phi  : " << expectedPhiDeg << " deg\n";
    std::cout << "Computed Phi  : " <<  res.PhiDeg << " deg\n";
}
TEST (SraAngleCalculator, Shift_Down_Led_2)
{
    const SRA_CameraModel_t cam{1456.0f, 1088.0f, 30.0f, 23.61f, 2717.14f, 2602.63f, 845.3f, 614.8f};
    SRA_AngleCalculator calc(cam);

    bool ledsVisible[cSRA_LedCount] = {true, true, true, true};
    SRA_BeaconPixels_t beacon = CreateBeacon(845.3f - 0.3518f, 614.8f + 102.2116f, ledsVisible);
    beacon.IsVisible = true; 
    float expectedThetaDeg = 0.0001f;
    float expectedPhiDeg = -2.0f;
    const SRA_AngleResult_t res = calc.Compute(beacon);

    EXPECT_TRUE(res.IsValid);
    //EXPECT_NEAR(res.ThetaDeg, expectedThetaDeg, 1e-1f);
    EXPECT_NEAR(res.PhiDeg, expectedPhiDeg, 1e-1f);

    //std::cout << "Expected Theta: " << expectedThetaDeg << " deg\n";
    //std::cout << "Computed Theta: " << res.ThetaDeg << " deg\n";
    std::cout << "Expected Phi  : " << expectedPhiDeg << " deg\n";
    std::cout << "Computed Phi  : " <<  res.PhiDeg << " deg\n";
}
TEST (SraAngleCalculator, Shift_High_Led_4)
{
    const SRA_CameraModel_t cam{1456.0f, 1088.0f, 30.0f, 23.61f, 2717.14f, 2602.63f, 845.3f, 614.8f};
    SRA_AngleCalculator calc(cam);

    bool ledsVisible[cSRA_LedCount] = {true, true, true, true};
    SRA_BeaconPixels_t beacon = CreateBeacon(845.3f - 0.9126f, 614.8f - 195.2706f, ledsVisible);
    beacon.IsVisible = true; 
    float expectedThetaDeg = 0.0001f;
    float expectedPhiDeg = 4.0f;
    const SRA_AngleResult_t res = calc.Compute(beacon);

    EXPECT_TRUE(res.IsValid);
    // EXPECT_NEAR(res.ThetaDeg, expectedThetaDeg, 1e-1f);
    EXPECT_NEAR(res.PhiDeg, expectedPhiDeg, 1e-1f);

    //std::cout << "Expected Theta: " << expectedThetaDeg << " deg\n";
    //std::cout << "Computed Theta: " << res.ThetaDeg << " deg\n";
    std::cout << "Expected Phi  : " << expectedPhiDeg << " deg\n";
    std::cout << "Computed Phi  : " <<  res.PhiDeg << " deg\n";
}
TEST (SraAngleCalculator, Shift_Down_Led_4)
{
    const SRA_CameraModel_t cam{1456.0f, 1088.0f, 30.0f, 23.61f, 2717.14f, 2602.63f, 845.3f, 614.8f};
    SRA_AngleCalculator calc(cam);

    bool ledsVisible[cSRA_LedCount] = {true, true, true, true};
    SRA_BeaconPixels_t beacon = CreateBeacon(845.3f + 1.0775f, 614.8f + 204.6652f, ledsVisible);
    beacon.IsVisible = true; 
    float expectedThetaDeg = 0.0001f;
    float expectedPhiDeg = -4.0f;
    const SRA_AngleResult_t res = calc.Compute(beacon);

    EXPECT_TRUE(res.IsValid);
    // EXPECT_NEAR(res.ThetaDeg, expectedThetaDeg, 1e-1f);
    EXPECT_NEAR(res.PhiDeg, expectedPhiDeg, 1e-1f);

    //std::cout << "Expected Theta: " << expectedThetaDeg << " deg\n";
    //std::cout << "Computed Theta: " << res.ThetaDeg << " deg\n";
    std::cout << "Expected Phi  : " << expectedPhiDeg << " deg\n";
    std::cout << "Computed Phi  : " <<  res.PhiDeg << " deg\n";
}
