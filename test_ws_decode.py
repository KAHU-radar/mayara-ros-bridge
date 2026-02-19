#!/usr/bin/env python3
"""
Test script — verify WebSocket connection and protobuf decoding
without ROS2. Run this first to confirm data is flowing.

Usage: python3 test_ws_decode.py [ws_url]
"""

import sys
import time

try:
    import websocket
except ImportError:
    print("Installing websocket-client...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install",
                           "websocket-client", "--break-system-packages"])
    import websocket

from radar_message_pb import decode_radar_message

WS_URL = sys.argv[1] if len(sys.argv) > 1 else \
    "ws://10.0.0.28:6502/v2/api/radars/radar-16/spokes"

count = 0
spoke_count = 0
start = time.time()


def on_message(ws, message):
    global count, spoke_count
    if not isinstance(message, (bytes, bytearray)):
        print(f"  [text] {message[:100]}")
        return

    count += 1
    try:
        msg = decode_radar_message(message)
    except Exception as e:
        print(f"  DECODE ERROR: {e}")
        print(f"  Raw bytes ({len(message)}): {message[:40].hex()}")
        return

    for spoke in msg.spokes:
        spoke_count += 1

    # Print details for first 5 messages, then summary every 100
    if count <= 5 or count % 100 == 0:
        elapsed = time.time() - start
        rate = spoke_count / elapsed if elapsed > 0 else 0

        print(f"[msg #{count}] radar={msg.radar}, {len(msg.spokes)} spokes, "
              f"total={spoke_count} spokes ({rate:.0f}/s)")

        if msg.spokes:
            s = msg.spokes[0]
            print(f"  First spoke: angle={s.angle}, range={s.range}m, "
                  f"bearing={s.bearing}, data_len={len(s.data)}, "
                  f"lat={s.lat}, lon={s.lon}, time={s.time}")

    # Stop after 500 messages
    if count >= 500:
        elapsed = time.time() - start
        print(f"\nDone! {count} messages, {spoke_count} spokes in {elapsed:.1f}s "
              f"({spoke_count/elapsed:.0f} spokes/s)")
        ws.close()


def on_open(ws):
    print(f"Connected to {WS_URL}")
    print("Waiting for spoke data...\n")


def on_error(ws, error):
    print(f"ERROR: {error}")


def on_close(ws, code, msg):
    print(f"Closed: {code} {msg}")


print(f"Connecting to {WS_URL}...")
ws = websocket.WebSocketApp(
    WS_URL,
    on_open=on_open,
    on_message=on_message,
    on_error=on_error,
    on_close=on_close,
)
ws.run_forever()
