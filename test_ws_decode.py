#!/usr/bin/env python3
"""
Test script — verify WebSocket connection and protobuf decoding
without ROS2. Run this first to confirm data is flowing.

Usage:
    python3 test_ws_decode.py                          # auto-discovers radar
    python3 test_ws_decode.py 192.168.1.50:6502        # auto-discovers on a different host
    python3 test_ws_decode.py ws://<host>:6502/v2/api/radars/<id>/spokes  # explicit URL
"""

import json
import sys
import time
import urllib.request

try:
    import websocket
except ImportError:
    print("Installing websocket-client...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install",
                           "websocket-client", "--break-system-packages"])
    import websocket

from radar_message_pb import decode_radar_message

DEFAULT_HOST = "10.0.0.28:6502"


def discover_radar_url(host: str) -> str:
    """Return the WebSocket spoke URL for the first radar Mayara has found."""
    api_url = f"http://{host}/v2/api/radars"
    print(f"Discovering radar from {api_url} ...")
    try:
        with urllib.request.urlopen(api_url, timeout=5) as resp:
            radars = json.loads(resp.read().decode())
    except Exception as e:
        raise RuntimeError(f"Cannot reach Mayara API at {api_url}: {e}")
    if not radars:
        raise RuntimeError("Mayara found no radars")
    # Response is a dict keyed by radar ID, e.g. {"radar-17": {"streamUrl": "ws://...", ...}}
    radar = list(radars.values())[0]
    url = radar["streamUrl"]
    print(f"Found radar: {radar['id']}  →  {url}")
    return url


arg = sys.argv[1] if len(sys.argv) > 1 else ""

if arg.startswith("ws://") or arg.startswith("wss://"):
    # Explicit full WebSocket URL passed
    WS_URL = arg
elif arg:
    # Host (and optional port) passed — auto-discover radar ID
    WS_URL = discover_radar_url(arg)
else:
    # Nothing passed — use default host and auto-discover
    WS_URL = discover_radar_url(DEFAULT_HOST)

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
