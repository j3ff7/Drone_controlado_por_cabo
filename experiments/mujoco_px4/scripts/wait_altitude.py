#!/usr/bin/env python3
"""Wait for a ground-truth altitude on the simulator side channel."""

import argparse
import json
import socket
import time


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--minimum", type=float, required=True)
    parser.add_argument("--timeout", type=float, default=300.0)
    args = parser.parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    target = ("127.0.0.1", 14650)
    sock.sendto(json.dumps({"v": 1, "type": "subscribe"}).encode(), target)
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            try:
                message = json.loads(sock.recv(65507))
            except socket.timeout:
                continue
            altitude = -float(message["pos_ned"][2])
            if altitude >= args.minimum:
                print(f"altitude={altitude:.3f} m at t_sim={message['time']:.3f} s")
                return 0
        return 1
    finally:
        sock.sendto(json.dumps({"v": 1, "type": "unsubscribe"}).encode(), target)
        sock.close()


if __name__ == "__main__":
    raise SystemExit(main())
