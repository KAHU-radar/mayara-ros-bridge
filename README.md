# Mayara → ROS2 Bridge

Bridges Mayara's WebSocket radar spoke stream to ROS2 topics compatible
with EchoFlow (SeawardScience).

## Architecture

```
Mayara (port 6502)
    │  WebSocket (protobuf RadarMessage)
    ▼
┌─────────────────────────┐
│   mayara_ros_bridge      │  Python ROS2 node
│   - Decode protobuf      │
│   - Batch spokes→sectors │
│   - Publish ROS2 topics  │
└──────────┬──────────────┘
           │  ROS2 topics
           ├── /aura/perception/sensors/halo_a/data  (RadarSector)
           └── /aura/nav/odom                         (Odometry)
           ▼
     EchoFlow Lite
```

## Data Mapping

| Mayara (protobuf Spoke)        | → | EchoFlow (RadarSector)       |
|-------------------------------|---|------------------------------|
| `spoke.angle / 2048 × 2π`     | → | `angle_start` (radians)      |
| `2π / 2048`                   | → | `angle_increment`            |
| `scan_time / 2048`            | → | `time_increment`             |
| `~0.967s` (measured)          | → | `scan_time`                  |
| `0.0`                         | → | `range_min`                  |
| `spoke.range`                 | → | `range_max` (meters)         |
| `spoke.data[i] / 15.0`       | → | `echoes[]` (float32, 0-1)    |
| `spoke.lat/lon × 1e-16`      | → | Odometry position            |
| `spoke.bearing`               | → | Odometry heading             |

## Quick Start

### 1. Copy to Pi

```bash
scp -r mayara_ros_bridge/ pi@10.0.0.28:~/kahu-stack/
```

### 2. Test WebSocket connection (no ROS2 needed)

```bash
cd ~/kahu-stack/mayara_ros_bridge
python3 test_ws_decode.py
```

You should see spoke data flowing with angles, ranges, and data lengths.

### 3. Run the bridge

```bash
cd ~/kahu-stack/mayara_ros_bridge
./run_bridge.sh
```

### 4. Verify in another terminal

```bash
# Check topics exist
ros2 topic list | grep -E "halo|odom"

# Check data is flowing
ros2 topic hz /aura/perception/sensors/halo_a/data

# Inspect a message
ros2 topic echo /aura/perception/sensors/halo_a/data --once
```

### 5. View in Foxglove

Connect Foxglove to `ws://10.0.0.28:8765` and subscribe to:
- `/aura/perception/sensors/halo_a/data`
- `/aura/nav/odom`

## Configuration

Parameters can be set via ROS2 params or environment variables:

| Parameter          | Env Var            | Default                                               |
|--------------------|--------------------|-------------------------------------------------------|
| mayara_ws_url      | MAYARA_WS_URL      | ws://10.0.0.28:6502/v2/api/radars/radar-11/spokes     |
| spokes_per_sector  | SPOKES_PER_SECTOR  | 64                                                    |
| radar_frame_id     | RADAR_FRAME_ID     | halo_a                                                |
| radar_topic        | RADAR_TOPIC        | /aura/perception/sensors/halo_a/data                  |
| odom_topic         | ODOM_TOPIC         | /aura/nav/odom                                        |
| spokes_per_rev     | SPOKES_PER_REV     | 2048                                                  |

## Dependencies

- Python 3 with ROS2 Jazzy (`rclpy`)
- `marine_sensor_msgs` (from EchoFlow workspace)
- `websocket-client` (pip, auto-installed by run script)
- No protobuf library needed (hand-coded decoder)

## Intensity Mapping

Mayara spoke bytes use values 0-15 for normal radar returns (see legend).
Values 16+ are metadata (target borders, history/trails). The bridge
normalizes 0-15 → 0.0-1.0 floats and maps 16+ to 0.0.

If EchoFlow expects raw device units instead of normalized values,
change the line in `bridge_node.py`:

```python
# Normalized (current):
echo.echoes = [float(b) / 15.0 if b <= 15 else 0.0 for b in spoke.data]

# Raw (alternative):
echo.echoes = [float(b) for b in spoke.data]
```

## Tuning spokes_per_sector

The original EchoFlow bag had ~25 RadarSector messages/sec with ~80
spokes each. The default of 64 gives ~32 sectors/sec. Adjust based on:

- **Lower (32)**: more frequent, lower latency updates
- **Higher (128)**: fewer messages, more efficient, higher latency
