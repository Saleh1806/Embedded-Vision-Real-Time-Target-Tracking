/* main.cpp
 * ----------------------------------------------------------------------------
 * Confidential file
 * Copyright (C) ARCK Sensor - All rights reserved
 * ----------------------------------------------------------------------------
 * Description:
 * Small demo: feed 4 LED pixel centers to the angle calculator and check the result.
 * ----------------------------------------------------------------------------
 * 28/11/2025 S.Diop: Initial creation
 * 01/12/2025 S.Diop: Respect Arck Sensor coding standards
 * ----------------------------------------------------------------------------
 */

#include "sra_angle_calculator.hpp"
#include "sra_angle_sender_tcp.hpp"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <thread>
#include <string>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <vector>
#include <array>
#include <algorithm>
#include <utility>

using namespace sirrah;

/* Create a beacon with the 4 LED pixel coordinates
 * Arguments : U -> pixel u coordinate, V -> pixel v coordinate
 * History   :
 *      28/11/2025 S.Diop : creation
 */
SRA_BeaconPixels_t CreateBeacon(float U, float V, bool LedsVisible[cSRA_LedCount])
{
    SRA_BeaconPixels_t BeaconPixels{};
    BeaconPixels.IsVisible = true;
    for (std::uint32_t LedIndex = 0; LedIndex < cSRA_LedCount; ++LedIndex)
    {
        BeaconPixels.LedsUV[LedIndex][0] = U;
        BeaconPixels.LedsUV[LedIndex][1] = V;
        BeaconPixels.LedsVisible[LedIndex] = LedsVisible[LedIndex];
    }
    return BeaconPixels;
}

/* Sort points around their geometric center */
std::vector<std::array<float, 2>> SortByAngle(const std::vector<std::array<float, 2>>& Points)
{
    std::vector<std::array<float, 2>> Sorted = Points;
    float Cx = 0.0f;
    float Cy = 0.0f;
    for (const auto& Point : Sorted)
    {
        Cx += Point[0];
        Cy += Point[1];
    }
    const float InvCount = 1.0f / static_cast<float>(Sorted.size());
    Cx *= InvCount;
    Cy *= InvCount;

    std::sort(Sorted.begin(), Sorted.end(),
        [Cx, Cy](const std::array<float, 2>& A, const std::array<float, 2>& B)
        {
            const float AngleA = std::atan2(A[1] - Cy, A[0] - Cx);
            const float AngleB = std::atan2(B[1] - Cy, B[0] - Cx);
            return AngleA < AngleB;
        });
    return Sorted;
}

/* Polygon area in pixel units with ordered points */
float PolygonAreaPx(const std::vector<std::array<float, 2>>& Points)
{
    const std::size_t Count = Points.size();
    if (Count < 3U)
    {
        return 0.0f;
    }

    float Sum = 0.0f;
    for (std::size_t Index = 0U; Index < Count; ++Index)
    {
        const std::size_t Next = (Index + 1U) % Count;
        Sum += (Points[Index][0] * Points[Next][1]) - (Points[Index][1] * Points[Next][0]);
    }
    return 0.5f * std::abs(Sum);
}

/* Distance from area ratio: Z = f / sqrt(A_img / A_model) */
bool ComputeDistanceFromArea(const SRA_BeaconPixels_t& BeaconPixels,
                             const SRA_CameraModel_t& CameraModel,
                             float& DistanceMeters)
{
    std::vector<std::array<float, 2>> VisiblePoints;
    VisiblePoints.reserve(cSRA_LedCount);
    for (std::uint32_t LedIndex = 0; LedIndex < cSRA_LedCount; ++LedIndex)
    {
        if (!BeaconPixels.LedsVisible[LedIndex])
        {
            continue;
        }
        VisiblePoints.push_back({BeaconPixels.LedsUV[LedIndex][0], BeaconPixels.LedsUV[LedIndex][1]});
    }

    if (VisiblePoints.size() < 3U || VisiblePoints.size() > cSRA_LedCount)
    {
        return false;
    }

    const std::vector<std::array<float, 2>> OrderedPoints = SortByAngle(VisiblePoints);
    const float ImageAreaPx = PolygonAreaPx(OrderedPoints);
    if (ImageAreaPx <= 0.0f)
    {
        return false;
    }

    constexpr float cBeaconSideMeters = 0.07f;
    const float SquareAreaM2 = cBeaconSideMeters * cBeaconSideMeters;
    float ModelAreaM2 = 0.0f;
    if (VisiblePoints.size() == 4U)
    {
        ModelAreaM2 = SquareAreaM2;
    }
    else if (VisiblePoints.size() == 3U)
    {
        ModelAreaM2 = 0.5f * SquareAreaM2;
    }
    else
    {
        return false;
    }

    const float ScalePxPerMeter = std::sqrt(ImageAreaPx / ModelAreaM2);
    if (ScalePxPerMeter <= 0.0f)
    {
        return false;
    }

    const float FocalPx = std::sqrt(CameraModel.Fx * CameraModel.Fy);
    DistanceMeters = FocalPx / ScalePxPerMeter;
    return std::isfinite(DistanceMeters) && (DistanceMeters > 0.0f);
}

/* Compute barycenter of visible LEDs */
bool ComputeVisibleBarycenter(const SRA_BeaconPixels_t& BeaconPixels, float& U, float& V)
{
    float SumU = 0.0f;
    float SumV = 0.0f;
    std::uint32_t Count = 0U;
    for (std::uint32_t LedIndex = 0; LedIndex < cSRA_LedCount; ++LedIndex)
    {
        if (!BeaconPixels.LedsVisible[LedIndex])
        {
            continue;
        }
        SumU += BeaconPixels.LedsUV[LedIndex][0];
        SumV += BeaconPixels.LedsUV[LedIndex][1];
        ++Count;
    }
    if (Count == 0U)
    {
        return false;
    }
    U = SumU / static_cast<float>(Count);
    V = SumV / static_cast<float>(Count);
    return true;
}

bool LoadReferenceFromFile(const std::string& RefFile, float& URef, float& VRef)
{
    std::ifstream in(RefFile);
    if (!in)
    {
        return false;
    }
    float u = 0.0f;
    float v = 0.0f;
    if (!(in >> u >> v))
    {
        return false;
    }
    URef = u;
    VRef = v;
    return true;
}

bool SaveReferenceToFile(const std::string& RefFile, float URef, float VRef)
{
    std::ofstream out(RefFile, std::ios::trunc);
    if (!out)
    {
        return false;
    }
    out << std::fixed << std::setprecision(6) << URef << " " << VRef << "\n";
    out.flush();
    return static_cast<bool>(out);
}

/* Demo entry point: compute angles from leds pixels barycenter and send over TCP
 * Arguments : none
 * History   :
 *      28/11/2025 S.Diop : creation
 *      08/12/2025 S.Diop : Add TCP sending
 */
int main(int argc, char* argv[])
{
    // Camera characteristics.
    // URef/VRef are initialized from the first valid frame barycenter.
    const SRA_CameraModel_t cBaseCameraModel{2064.0f, 1552.0f, 34.54f, 24.92f, 3319.491f, 3511.91f, 0.0f, 0.0f};
    bool ReferenceInitialized = false;
    float ReferenceU = 0.0f;
    float ReferenceV = 0.0f;

    float cx = 0.0f;
    float cy = 0.0f;
    bool hasCx = false;
    bool hasCy = false;
    std::string coordsFile;
    bool useStdin = true;
    std::string ackFile;
    std::string refFile = "/tmp/sirrah_angle_ref_uv.txt";
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--cx" && (i + 1) < argc)
        {
            cx = std::strtof(argv[++i], nullptr);
            hasCx = true;
        }
        else if (arg == "--cy" && (i + 1) < argc)
        {
            cy = std::strtof(argv[++i], nullptr);
            hasCy = true;
        }
        else if (arg == "--coords-file" && (i + 1) < argc)
        {
            coordsFile = argv[++i];
            useStdin = false;
        }
        else if (arg == "--ack-file" && (i + 1) < argc)
        {
            ackFile = argv[++i];
        }
        else if (arg == "--ref-file" && (i + 1) < argc)
        {
            refFile = argv[++i];
        }
    }

    if (LoadReferenceFromFile(refFile, ReferenceU, ReferenceV))
    {
        ReferenceInitialized = true;
        std::cout << "Reference loaded from file: u_ref=" << ReferenceU
                  << " v_ref=" << ReferenceV << "\n";
    }

    if (hasCx && hasCy)
    {
        SRA_AngleSender AngleSenderTcp("192.168.3.13", 8012);
        SRA_BeaconPixels_t BeaconPixels{};
        BeaconPixels.IsVisible = true;
        for (std::uint32_t LedIndex = 0; LedIndex < cSRA_LedCount; ++LedIndex)
        {
            if (LedIndex == 0U)
            {
                BeaconPixels.LedsUV[LedIndex][0] = cx;
                BeaconPixels.LedsUV[LedIndex][1] = cy;
                BeaconPixels.LedsVisible[LedIndex] = true;
            }
            else
            {
                BeaconPixels.LedsUV[LedIndex][0] = 0.0f;
                BeaconPixels.LedsUV[LedIndex][1] = 0.0f;
                BeaconPixels.LedsVisible[LedIndex] = false;
            }
        }

        if (!ReferenceInitialized)
        {
            ReferenceU = cx;
            ReferenceV = cy;
            ReferenceInitialized = true;
            if (!SaveReferenceToFile(refFile, ReferenceU, ReferenceV))
            {
                std::cerr << "Warning: unable to persist reference file: " << refFile << "\n";
            }
        }

        SRA_CameraModel_t CameraModel = cBaseCameraModel;
        CameraModel.URef = ReferenceU;
        CameraModel.VRef = ReferenceV;
        SRA_AngleCalculator AngleCalculator(CameraModel);
        const SRA_AngleResult_t Result = AngleCalculator.Compute(BeaconPixels);
        if (!Result.IsValid)
        {
            std::cerr << "Angle computation invalid.\n";
            return 2;
        }

        std::cout << std::fixed << std::setprecision(6)
                  << "theta_deg=" << Result.ThetaDeg
                  << " phi_deg=" << Result.PhiDeg << std::endl;
        if (Result.VisibleLedCount > 2U)
        {
            float DistanceMeters = 0.0f;
            if (ComputeDistanceFromArea(BeaconPixels, CameraModel, DistanceMeters))
            {
                std::cout << "distance_m=" << DistanceMeters << std::endl;
            }
            else
            {
                std::cout << "Distance non calculee: echec du calcul par rapport des aires." << std::endl;
            }
        }
        else
        {
            std::cout << "Distance non calculee: moins de 3 LEDs detectees." << std::endl;
        }

        (void)AngleSenderTcp.Send(Result);
        return 0;
    }

    // TCP server to push angles to Sirrah Maintenance (maintenance client connects here).
    SRA_AngleSender AngleSenderTcp("192.168.3.13", 8012);
    auto WriteAck = [&ackFile](long long tsValue)
    {
        if (ackFile.empty())
        {
            return;
        }
        std::ofstream ackOut(ackFile, std::ios::trunc);
        if (!ackOut)
        {
            return;
        }
        ackOut << tsValue << "\n";
        ackOut.flush();
    };

    // Target angles we want to retrieve (in degrees).
    std::ifstream fileInput;
    std::istream* input = &std::cin;
    std::string line;
    const std::chrono::milliseconds PollPeriod(10);
    while (true)
    {
        if (!useStdin && !fileInput.is_open())
        {
            fileInput.open(coordsFile);
            if (!fileInput)
            {
                std::this_thread::sleep_for(PollPeriod);
                continue;
            }
            input = &fileInput;
        }

        if (!std::getline(*input, line))
        {
            if (useStdin)
            {
                break;
            }
            if (fileInput.eof())
            {
                fileInput.clear();
                std::this_thread::sleep_for(PollPeriod);
                continue;
            }
            fileInput.close();
            std::this_thread::sleep_for(PollPeriod);
            continue;
        }

        for (char& ch : line)
        {
            if (ch == ',')
            {
                ch = ' ';
            }
        }

        if (line.find("None") != std::string::npos || line.find("none") != std::string::npos)
        {
            long long ts = static_cast<long long>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count());
            std::istringstream tsStream(line);
            long long parsedTs = 0;
            if (tsStream >> parsedTs)
            {
                ts = parsedTs;
            }

            SRA_AngleResult_t InvalidResult{};
            InvalidResult.ThetaDeg = 0.0f;
            InvalidResult.PhiDeg = 0.0f;
            InvalidResult.IsValid = false;
            InvalidResult.Validity = cSRA_ValidityNoDetection;
            InvalidResult.VisibleLedCount = 0U;
            for (std::uint32_t LedIndex = 0; LedIndex < cSRA_LedCount; ++LedIndex)
            {
                InvalidResult.LedsUsed[LedIndex] = false;
            }

            std::cout << "balise_invisible\n";
            WriteAck(ts);
            (void)AngleSenderTcp.Send(InvalidResult);
            continue;
        }

        std::istringstream iss(line);
        std::vector<float> values;
        values.reserve(9U);
        float value = 0.0f;
        while (iss >> value)
        {
            values.push_back(value);
        }
        if (values.size() < 2U)
        {
            continue;
        }

        long long ts = 0;
        bool hasTs = false;
        std::size_t offset = 0U;
        if ((values.size() % 2U) == 1U)
        {
            ts = static_cast<long long>(values[0]);
            hasTs = true;
            offset = 1U;
        }

        const std::size_t remainingValueCount = values.size() - offset;
        if (remainingValueCount < 2U || (remainingValueCount % 2U) != 0U)
        {
            continue;
        }

        const std::size_t inputLedCount = remainingValueCount / 2U;
        const std::size_t usedLedCount =
            (inputLedCount > static_cast<std::size_t>(cSRA_LedCount))
                ? static_cast<std::size_t>(cSRA_LedCount)
                : inputLedCount;

        SRA_BeaconPixels_t BeaconPixels{};
        BeaconPixels.IsVisible = true;
        for (std::uint32_t LedIndex = 0; LedIndex < cSRA_LedCount; ++LedIndex)
        {
            if (LedIndex < usedLedCount)
            {
                const std::size_t baseIndex = offset + (2U * static_cast<std::size_t>(LedIndex));
                BeaconPixels.LedsUV[LedIndex][0] = values[baseIndex];
                BeaconPixels.LedsUV[LedIndex][1] = values[baseIndex + 1U];
                BeaconPixels.LedsVisible[LedIndex] = true;
            }
            else
            {
                BeaconPixels.LedsUV[LedIndex][0] = 0.0f;
                BeaconPixels.LedsUV[LedIndex][1] = 0.0f;
                BeaconPixels.LedsVisible[LedIndex] = false;
            }
        }

        if (!ReferenceInitialized)
        {
            float U0 = 0.0f;
            float V0 = 0.0f;
            if (ComputeVisibleBarycenter(BeaconPixels, U0, V0))
            {
                ReferenceU = U0;
                ReferenceV = V0;
                ReferenceInitialized = true;
                std::cout << "Reference initialized from first frame barycenter: "
                          << "u_ref=" << ReferenceU << " v_ref=" << ReferenceV << "\n";
                if (!SaveReferenceToFile(refFile, ReferenceU, ReferenceV))
                {
                    std::cerr << "Warning: unable to persist reference file: " << refFile << "\n";
                }
            }
            else
            {
                continue;
            }
        }

        SRA_CameraModel_t CameraModel = cBaseCameraModel;
        CameraModel.URef = ReferenceU;
        CameraModel.VRef = ReferenceV;
        SRA_AngleCalculator AngleCalculator(CameraModel);
        const SRA_AngleResult_t Result = AngleCalculator.Compute(BeaconPixels);
        if (!Result.IsValid)
        {
            std::cerr << "Angle computation invalid.\n";
            if (!hasTs)
            {
                ts = static_cast<long long>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count());
            }
            WriteAck(ts);
            continue;
        }

        std::cout << std::fixed << std::setprecision(6)
                  << "theta_deg=" << Result.ThetaDeg
                  << " phi_deg=" << Result.PhiDeg << "\n";
        if (Result.VisibleLedCount > 2U)
        {
            float DistanceMeters = 0.0f;
            if (ComputeDistanceFromArea(BeaconPixels, CameraModel, DistanceMeters))
            {
                std::cout << "distance_m=" << DistanceMeters << "\n";
            }
            else
            {
                std::cout << "Distance non calculee: echec du calcul par rapport des aires.\n";
            }
        }
        else
        {
            std::cout << "Distance non calculee: moins de 3 LEDs detectees.\n";
        }

        if (!hasTs)
        {
            ts = static_cast<long long>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count());
        }
        WriteAck(ts);
        (void)AngleSenderTcp.Send(Result);
    }
    return 0;
}
