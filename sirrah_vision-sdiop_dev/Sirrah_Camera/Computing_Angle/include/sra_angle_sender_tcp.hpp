/* sra_angle_sender.hpp
 * ----------------------------------------------------------------------------
 * Confidential file
 * Copyright (C) ARCK Sensor - All rights reserved
 * ----------------------------------------------------------------------------
 * Description:
 * Helper to send SRA_AngleResult_t to the Sirrah Maintenance tool over TCP.
 * The payload mirrors appsensor/dataman_eth formatting (PC mode, diag off),
 * with checksum and CR/LF terminator.
 * ----------------------------------------------------------------------------
 * 05/12/2025: Initial creation for Sirrah Camera Ethernet streaming (port 8012)
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <cstdint>
#include <array>
#include <string>

#include "sra_angle_calculator.hpp"

namespace sirrah
{

/** Beacon payload matching dataman_eth (PC mode, no diag, Param.Diag = DIAG_DATA_PULSE),
 * with speed on (Conf.Speed = 1), checksum activé.
 * Layout (big endian angles, checksum bit-sum over bytes 0..14):
 * [0]   beacon_state (BeaconID=1, bit5 set if invalid)
 * [1-4] datation (uint32_t BE, placeholder here)
 * [5]   energy (VisibleLedCount placeholder)
 * [6]   pulseNum (placeholder)
 * [7-14] reserved (8 bytes placeholder to align with 26-byte frame on the wire)
 * [15-16] Theta (int16, milli-deg, BE)
 * [17-18] Phi   (int16, milli-deg, BE)
 * [19-20] Theta'   (int16, milli-deg, BE) placeholder
 * [21-22] Phi'     (int16, milli-deg, BE) placeholder
 * [23]  checksum (bit-sum over bytes 0..22)
 * Followed on the wire by 0x0A 0x0D (LF/CR) terminator.
 */
using SRA_AngleFrame = std::array<uint8_t, 24>;
static constexpr std::size_t kPayloadSize = SRA_AngleFrame{}.size() - 1; // without checksum
static constexpr std::size_t kTerminatorSize = 2; // LF/CR
static constexpr std::size_t kSendSize = SRA_AngleFrame{}.size() + kTerminatorSize;

class SRA_AngleSender
{
public:
    SRA_AngleSender(std::string serverIp, uint16_t serverPort = 8012);
    ~SRA_AngleSender();

    /** Connect to the maintenance software (idempotent). */
    bool Connect();

    /** Send an angle result. Will try to connect if not already. */
    bool Send(const SRA_AngleResult_t& result);

private:
    uint8_t ComputeChecksum(const uint8_t* data, std::size_t len) const;
    bool EnsureConnected();
    bool StartListening();
    bool AcceptClient();

    std::string Ip;
    uint16_t Port;
    int ListenSocketFd;
    int ClientSocketFd;
};

} // namespace sirrah

/* end of file */
