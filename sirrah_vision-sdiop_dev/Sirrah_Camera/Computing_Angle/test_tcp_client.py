#!/usr/bin/env python3
import socket, struct, sys

host = sys.argv[1] if len(sys.argv) > 1 else "0.0.0.0"
port = int(sys.argv[2]) if len(sys.argv) > 2 else 8012

# Connect to the TCP server
with socket.create_connection((host, port)) as s:
    data = s.recv(1024)

print("bytes:", len(data), data)

# Need at least one full frame (17 bytes)
if len(data) < 17:
    sys.exit("not enough bytes for full frame")

# Unpack the 17-byte frame according to our protocol
magic, ver, validity, leds, reserved, theta, phi, chk = struct.unpack("<4sBBBBffB", data[:17])
print(magic, ver, validity, leds, reserved, theta, phi, chk)
