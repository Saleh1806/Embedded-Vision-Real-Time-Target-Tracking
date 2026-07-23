/* sra_angle_sender.cpp
 * ----------------------------------------------------------------------------
 * Confidential file
 * Copyright (C) ARCK Sensor - All rights reserved
 * ----------------------------------------------------------------------------
 * Description:
 * TCP server to stream angle results to Sirrah Maintenance (port 8012).
 * https://www.tutorialspoint.com/cplusplus/cpp_socket_programming.htm
 * ----------------------------------------------------------------------------
 * 08/12/2025: Initial creation for Sirrah Camera Ethernet streaming
 * 15/12/2025: Run as TCP server instead of client (listen/accept then send) for Maintenance SIRRAH
 * ----------------------------------------------------------------------------
 */

#include "sra_angle_sender_tcp.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/time.h>

#include <cstdio>
#include <cstring>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace sirrah
{

namespace
{
inline int16_t ToMilliDeg(float angleDeg)
{
    const float scaled = std::round(angleDeg * 1000.0f);
    const float clamped = std::max(-32768.0f, std::min(32767.0f, scaled));
    return static_cast<int16_t>(clamped);
}
} // namespace

/* Build TCP sender with target address/port
 * Arguments : targetIp -> local IP address to bind the server on
 *             targetPort -> local port to listen on
 * History   :
 *      08/12/2025 : creation
 */
SRA_AngleSender::SRA_AngleSender(const std::string targetIp, uint16_t targetPort)
    : Ip(targetIp), Port(targetPort), ListenSocketFd(-1), ClientSocketFd(-1)
{
}

/* Close sockets on destruction
 * Arguments : none
 * History   :
 *      08/12/2025 : creation
 */
SRA_AngleSender::~SRA_AngleSender()
{
    if (ListenSocketFd >= 0)
    {
        close(ListenSocketFd);
        ListenSocketFd = -1;
    }
    if (ClientSocketFd >= 0)
    {
        close(ClientSocketFd);
        ClientSocketFd = -1;
    }
}

/* Start the TCP listener socket
 * Arguments : none
 * History   :
 *      08/12/2025 : creation
 */
bool SRA_AngleSender::StartListening()
{
    if (ListenSocketFd >= 0)
    {
        return true;
    }

    ListenSocketFd = socket(AF_INET, SOCK_STREAM, 0);
    if (ListenSocketFd < 0)
    {
        std::perror("socket");
        return false;
    }

    int opt = 1;
    if (setsockopt(ListenSocketFd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
    {
        std::perror("setsockopt");
        close(ListenSocketFd);
        ListenSocketFd = -1;
        return false;
    }

    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(Port);
    if (inet_pton(AF_INET, Ip.c_str(), &serverAddr.sin_addr) <= 0)
    {
        std::cerr << "Invalid local IP address: " << Ip << std::endl;
        close(ListenSocketFd);
        ListenSocketFd = -1;
        return false;
    }

    if (bind(ListenSocketFd, (sockaddr*)&serverAddr, sizeof(serverAddr)) < 0)
    {
        std::perror("bind");
        close(ListenSocketFd);
        ListenSocketFd = -1;
        return false;
    }

    if (listen(ListenSocketFd, 1) < 0)
    {
        std::perror("listen");
        close(ListenSocketFd);
        ListenSocketFd = -1;
        return false;
    }

    std::cout << "TCP server listening on " << Ip << ":" << Port << std::endl;
    return true;
}

/* Accept a client connection on the listening socket
 * Arguments : none
 * History   :
 *      08/12/2025 : creation
 */
bool SRA_AngleSender::AcceptClient()
{
    if (!StartListening())
    {
        return false;
    }

    sockaddr_in clientAddr{};
    socklen_t clientLen = sizeof(clientAddr);
    std::cout << "Waiting for TCP client connection..." << std::endl;
    ClientSocketFd = accept(ListenSocketFd, (sockaddr*)&clientAddr, &clientLen);
    if (ClientSocketFd < 0)
    {
        std::perror("accept");
        return false;
    }

    int ka = 1;
    if (setsockopt(ClientSocketFd, SOL_SOCKET, SO_KEEPALIVE, &ka, sizeof(ka)) < 0)
    {
        std::perror("setsockopt(SO_KEEPALIVE)");
    }
    timeval sendTimeout{};
    sendTimeout.tv_sec = 0;
    sendTimeout.tv_usec = 100000; // 100 ms
    if (setsockopt(ClientSocketFd, SOL_SOCKET, SO_SNDTIMEO, &sendTimeout, sizeof(sendTimeout)) < 0)
    {
        std::perror("setsockopt(SO_SNDTIMEO)");
    }
#ifdef TCP_KEEPIDLE
    int idle = 10;
    if (setsockopt(ClientSocketFd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle)) < 0)
    {
        std::perror("setsockopt(TCP_KEEPIDLE)");
    }
#endif
#ifdef TCP_KEEPINTVL
    int intvl = 3;
    if (setsockopt(ClientSocketFd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl)) < 0)
    {
        std::perror("setsockopt(TCP_KEEPINTVL)");
    }
#endif
#ifdef TCP_KEEPCNT
    int cnt = 5;
    if (setsockopt(ClientSocketFd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt)) < 0)
    {
        std::perror("setsockopt(TCP_KEEPCNT)");
    }
#endif

    char clientIp[INET_ADDRSTRLEN] = {};
    if (inet_ntop(AF_INET, &clientAddr.sin_addr, clientIp, sizeof(clientIp)) != nullptr)
    {
        std::cout << "TCP client connected from " << clientIp << ":" << ntohs(clientAddr.sin_port) << std::endl;
    }
    else
    {
        std::cout << "TCP client connected" << std::endl;
    }

    return true;
}

/* Start listening and accept a client
 * Arguments : none
 * History   :
 *      08/12/2025 : creation
 *      12/12/2025 : change to server accept
 */
bool SRA_AngleSender::Connect()
{
    if (ClientSocketFd >= 0)
    {
        return true;
    }

    return AcceptClient();
}

/* Ensure a TCP client is connected before sending
 * Arguments : none
 * History   :
 *      08/12/2025 : creation
 */
bool SRA_AngleSender::EnsureConnected()
{
    if (ClientSocketFd >= 0)
    {
        return true;
    }
    return Connect();
}

/* Compute checksum of the frame bytes
 * Arguments : data -> bytes to checksum (checksum byte excluded)
 * History   :
 *      08/12/2025 : creation
 *      12/12/2025 : accept variable payload size (mirror dataman_eth)
 */
uint8_t SRA_AngleSender::ComputeChecksum(const uint8_t* data, std::size_t len) const
{
    uint8_t chk = 0;
    const uint8_t* end = data + len;
    for (const uint8_t* p = data; p < end; ++p)
    {
        for (int i = 0; i < 8; ++i)
        {
            chk += (*p >> i) & 0x01;
        }
    }
    return chk;
}

/* Send an angle frame over TCP
 * Arguments : result -> computed angle result to send
 * History   :
 *      08/12/2025 : creation
 */
bool SRA_AngleSender::Send(const SRA_AngleResult_t& result)
{
    SRA_AngleFrame frame{};


    // [0] beacon_state 
    // [1-4] datation (placeholder)
    // [5] energy (VisibleLedCount placeholder)
    // [6] pulseNum (placeholder)
    // [7-14] reserved (8 bytes placeholder)
    // [15-16] Theta (int16 BE, milli-deg)
    // [17-18] Phi   (int16 BE, milli-deg)
    // [19-20] Theta' (int16 BE, milli-deg) placeholder
    // [21-22] Phi'   (int16 BE, milli-deg) placeholder
    // [23] checksum (bit-sum over bytes 0..22)
    // On the wire, LF/CR (0x0A 0x0D) are appended like eth_com_sendData.

    uint8_t beaconState = 0x00; // BeaconID=1
    if (result.Validity != cSRA_ValidityOk || !result.IsValid)
    {
        beaconState |= 0x20; // mark invalid
    }
    frame[0] = beaconState;

    // datation placeholder
    frame[1] = 0;
    frame[2] = 0;
    frame[3] = 0;
    frame[4] = 0;

    frame[5] = static_cast<uint8_t>(result.VisibleLedCount); // energy placeholder
    frame[6] = 4; // pulseNum placeholder

    // 8 reserved bytes after pulseNum (keep zeros to reach expected payload size)
    for (std::size_t i = 7; i <= 14; ++i)
    {
        frame[i] = 0;
    }

    const int16_t theta = ToMilliDeg(result.ThetaDeg);
    const int16_t phi = ToMilliDeg(result.PhiDeg);
    const int16_t thetaPrim = 0; // speed placeholders
    const int16_t phiPrim = 0;
    frame[15] = static_cast<uint8_t>(theta >> 8);
    frame[16] = static_cast<uint8_t>(theta);
    frame[17] = static_cast<uint8_t>(phi >> 8);
    frame[18] = static_cast<uint8_t>(phi);
    frame[19] = static_cast<uint8_t>(thetaPrim >> 8);
    frame[20] = static_cast<uint8_t>(thetaPrim);
    frame[21] = static_cast<uint8_t>(phiPrim >> 8);
    frame[22] = static_cast<uint8_t>(phiPrim);
    
    // Checksum over payload bytes 0..22
    frame[23] = ComputeChecksum(frame.data(), kPayloadSize);

    std::array<uint8_t, kSendSize> sendBuffer{};
    std::copy(frame.begin(), frame.end(), sendBuffer.begin());
    sendBuffer[SRA_AngleFrame{}.size()] = 0x0A; // LF
    sendBuffer[SRA_AngleFrame{}.size() + 1] = 0x0D; // CR

    if (!EnsureConnected())
    {
        return false;
    }

    ssize_t sent = send(ClientSocketFd, sendBuffer.data(), sendBuffer.size(), MSG_NOSIGNAL);
    if (sent != static_cast<ssize_t>(sendBuffer.size()))
    {
        std::perror("send");
        close(ClientSocketFd);
        ClientSocketFd = -1;
        if (!EnsureConnected())
        {
            return false;
        }
        sent = send(ClientSocketFd, sendBuffer.data(), sendBuffer.size(), MSG_NOSIGNAL);
        if (sent != static_cast<ssize_t>(sendBuffer.size()))
        {
            std::perror("send");
            close(ClientSocketFd);
            ClientSocketFd = -1;
            return false;
        }
    }
    return true;
}

} // namespace sirrah

/* end of file */
