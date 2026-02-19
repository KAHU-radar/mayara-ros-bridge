#!/usr/bin/env python3
"""
Mayara → ROS2 Bridge Node

Connects to Mayara's WebSocket spoke stream, decodes protobuf RadarMessage,
batches spokes into marine_sensor_msgs/RadarSector messages, and publishes
them on ROS2 topics that EchoFlow can consume.

Also publishes nav_msgs/Odometry from the spoke lat/lon/bearing data.

Usage:
    ros2 run mayara_ros_bridge bridge_node
    # or standalone:
    python3 bridge_node.py

Parameters (ROS2 or env):
    mayara_host         - Mayara host:port (default: 10.0.0.28:6502). Radar ID
                          is auto-discovered from the Mayara REST API on startup.
    mayara_ws_url       - Override: full WebSocket URL. Set this to skip
                          auto-discovery and connect to a specific radar ID.
    spokes_per_sector   - Spokes to batch per RadarSector (default: 64)
    radar_frame_id      - TF frame for radar data (default: halo_a)
    radar_topic         - Topic for RadarSector (default: /aura/perception/sensors/halo_a/data)
    odom_topic          - Topic for Odometry (default: /aura/nav/odom)
    spokes_per_rev      - Spokes per revolution from Mayara (default: 2048)
"""

import json
import math
import os
import sys
import time
import threading
import struct
import urllib.request
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Pose, PoseWithCovariance, Twist, TwistWithCovariance
from builtin_interfaces.msg import Time, Duration

from marine_sensor_msgs.msg import RadarSector, RadarEcho

# Our hand-coded protobuf decoder (no protobuf dependency)
from radar_message_pb import decode_radar_message

# WebSocket client - use the lightweight websocket-client library
try:
    import websocket
except ImportError:
    print("ERROR: websocket-client not installed. Install with:")
    print("  pip install websocket-client --break-system-packages")
    sys.exit(1)


TWO_PI = 2.0 * math.pi


def _discover_radar_url(host: str) -> str:
    """
    Ask the Mayara REST API which radars are available and return the
    WebSocket URL for the first one found.

    Args:
        host: Mayara host:port, e.g. "10.0.0.28:6502"

    Returns:
        WebSocket URL, e.g. "ws://10.0.0.28:6502/v2/api/radars/radar-3/spokes"

    Raises:
        RuntimeError if no radars are found or the API is unreachable.
    """
    api_url = f"http://{host}/v2/api/radars"
    try:
        with urllib.request.urlopen(api_url, timeout=5) as resp:
            radars = json.loads(resp.read().decode())
    except Exception as e:
        raise RuntimeError(f"Cannot reach Mayara API at {api_url}: {e}")

    # Response is a list of radar objects; each has an "id" field
    if not radars:
        raise RuntimeError(f"Mayara found no radars (response: {radars})")

    # Response is a dict keyed by radar ID, e.g. {"radar-17": {"streamUrl": "ws://...", ...}}
    radar = list(radars.values())[0]
    return radar["streamUrl"]


class MayaraRosBridge(Node):
    def __init__(self):
        super().__init__("mayara_ros_bridge")

        # Declare parameters with defaults
        self.declare_parameter("mayara_host",
            os.environ.get("MAYARA_HOST", "10.0.0.28:6502"))
        self.declare_parameter("mayara_ws_url",
            os.environ.get("MAYARA_WS_URL", ""))
        self.declare_parameter("spokes_per_sector", int(os.environ.get("SPOKES_PER_SECTOR", "64")))
        self.declare_parameter("radar_frame_id", os.environ.get("RADAR_FRAME_ID", "halo_a"))
        self.declare_parameter("radar_topic",
            os.environ.get("RADAR_TOPIC", "/aura/perception/sensors/halo_a/data"))
        self.declare_parameter("odom_topic",
            os.environ.get("ODOM_TOPIC", "/aura/nav/odom"))
        self.declare_parameter("spokes_per_rev", int(os.environ.get("SPOKES_PER_REV", "2048")))
        self.declare_parameter("odom_frame_id", os.environ.get("ODOM_FRAME_ID", "odom"))
        self.declare_parameter("base_frame_id", os.environ.get("BASE_FRAME_ID", "base_link"))

        # Read parameters — auto-discover radar ID if no explicit URL is set
        mayara_host = self.get_parameter("mayara_host").value
        ws_url_override = self.get_parameter("mayara_ws_url").value

        if ws_url_override:
            self.ws_url = ws_url_override
            self.get_logger().info(f"Using configured WebSocket URL: {self.ws_url}")
        else:
            self.get_logger().info(f"Auto-discovering radar from http://{mayara_host}/v2/api/radars ...")
            try:
                self.ws_url = _discover_radar_url(mayara_host)
                self.get_logger().info(f"Discovered radar: {self.ws_url}")
            except RuntimeError as e:
                self.get_logger().error(str(e))
                raise
        self.spokes_per_sector = self.get_parameter("spokes_per_sector").value
        self.radar_frame_id = self.get_parameter("radar_frame_id").value
        self.radar_topic = self.get_parameter("radar_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.spokes_per_rev = self.get_parameter("spokes_per_rev").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value

        # Derived constants
        self.angle_increment = TWO_PI / self.spokes_per_rev  # radians per spoke

        # QoS matching the original EchoFlow bag
        radar_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # Publishers
        self.radar_pub = self.create_publisher(RadarSector, self.radar_topic, radar_qos)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, odom_qos)

        # Spoke accumulator for batching into sectors
        self.spoke_buffer = deque()
        self.buffer_lock = threading.Lock()

        # Stats
        self.total_spokes = 0
        self.total_sectors = 0
        self.total_odom = 0
        self.last_stats_time = time.time()

        # Scan time tracking (measure actual revolution time)
        self.last_revolution_time = None
        self.scan_time_sec = 0.967  # default ~1 rev/sec from your logs (967ms)

        # Last known position for odometry
        self.last_lat = None
        self.last_lon = None
        self.last_bearing = None

        # WebSocket connection runs in a separate thread
        self.ws = None
        self.ws_thread = None
        self.running = True

        self.get_logger().info("=" * 60)
        self.get_logger().info("Mayara → ROS2 Bridge")
        self.get_logger().info(f"  WebSocket:        {self.ws_url}")
        self.get_logger().info(f"  Spokes/rev:       {self.spokes_per_rev}")
        self.get_logger().info(f"  Spokes/sector:    {self.spokes_per_sector}")
        self.get_logger().info(f"  Radar topic:      {self.radar_topic}")
        self.get_logger().info(f"  Odom topic:       {self.odom_topic}")
        self.get_logger().info(f"  Radar frame:      {self.radar_frame_id}")
        self.get_logger().info("=" * 60)

        # Start WebSocket connection
        self._start_ws()

        # Timer for stats logging (every 5 seconds)
        self.stats_timer = self.create_timer(5.0, self._log_stats)

    def _start_ws(self):
        """Start WebSocket connection in background thread."""
        self.ws_thread = threading.Thread(target=self._ws_loop, daemon=True)
        self.ws_thread.start()

    def _ws_loop(self):
        """WebSocket connection loop with auto-reconnect."""
        while self.running:
            try:
                self.get_logger().info(f"Connecting to {self.ws_url}...")
                self.ws = websocket.WebSocketApp(
                    self.ws_url,
                    on_open=self._on_ws_open,
                    on_message=self._on_ws_message,
                    on_error=self._on_ws_error,
                    on_close=self._on_ws_close,
                )
                self.ws.run_forever(ping_interval=10, ping_timeout=5)
            except Exception as e:
                self.get_logger().error(f"WebSocket error: {e}")

            if self.running:
                self.get_logger().warn("Reconnecting in 3 seconds...")
                time.sleep(3)

    def _on_ws_open(self, ws):
        self.get_logger().info("WebSocket connected! Receiving spokes...")

    def _on_ws_error(self, ws, error):
        self.get_logger().error(f"WebSocket error: {error}")

    def _on_ws_close(self, ws, close_status_code, close_msg):
        self.get_logger().warn(f"WebSocket closed: {close_status_code} {close_msg}")

    def _on_ws_message(self, ws, message):
        """Called for each WebSocket message (binary protobuf RadarMessage)."""
        if not isinstance(message, (bytes, bytearray)):
            return

        try:
            radar_msg = decode_radar_message(message)
        except Exception as e:
            self.get_logger().error(f"Protobuf decode error: {e}")
            return

        now = self.get_clock().now()

        for spoke in radar_msg.spokes:
            self.total_spokes += 1

            # Track revolution timing (when angle wraps near 0)
            if spoke.angle < 10 and self.last_revolution_time is not None:
                elapsed = time.time() - self.last_revolution_time
                if 0.5 < elapsed < 5.0:  # sanity check
                    self.scan_time_sec = elapsed
            if spoke.angle < 10:
                self.last_revolution_time = time.time()

            # Update position from spoke data
            if spoke.lat is not None and spoke.lon is not None:
                self.last_lat = spoke.lat * 1e-16  # convert to degrees
                self.last_lon = spoke.lon * 1e-16
            if spoke.bearing is not None:
                self.last_bearing = spoke.bearing

            # Buffer the spoke
            with self.buffer_lock:
                self.spoke_buffer.append(spoke)

                # When we have enough spokes, emit a sector
                if len(self.spoke_buffer) >= self.spokes_per_sector:
                    spokes_batch = [self.spoke_buffer.popleft()
                                    for _ in range(self.spokes_per_sector)]
                    self._publish_sector(spokes_batch, now)

        # Publish odometry at ~20Hz (every ~100 spokes at 2048/sec)
        if self.total_spokes % 100 == 0 and self.last_lat is not None:
            self._publish_odom(now)

    def _publish_sector(self, spokes: list, stamp):
        """Convert a batch of spokes into a RadarSector and publish."""
        if not spokes:
            return

        sector = RadarSector()

        # Header
        sector.header = Header()
        sector.header.stamp = stamp.to_msg()
        sector.header.frame_id = self.radar_frame_id

        # Angle geometry
        # Mayara angle is clockwise from bow (0 = ahead, 512 = starboard for 2048 spokes)
        # ROS convention: counterclockwise from X-axis (forward)
        # So ROS angle = -mayara_angle (or equivalently 2π - mayara_angle)
        first_angle = spokes[0].angle
        sector.angle_start = -(first_angle * self.angle_increment)  # negate for CCW convention

        # Angle increment (negative because Mayara goes CW, ROS expects CCW)
        sector.angle_increment = -self.angle_increment

        # Time between rays
        time_per_spoke = self.scan_time_sec / self.spokes_per_rev
        sector.time_increment = Duration()
        time_inc_sec = int(time_per_spoke)
        time_inc_nsec = int((time_per_spoke - time_inc_sec) * 1e9)
        sector.time_increment.sec = time_inc_sec
        sector.time_increment.nanosec = time_inc_nsec

        # Scan time (full revolution)
        sector.scan_time = Duration()
        scan_sec = int(self.scan_time_sec)
        scan_nsec = int((self.scan_time_sec - scan_sec) * 1e9)
        sector.scan_time.sec = scan_sec
        sector.scan_time.nanosec = scan_nsec

        # Range — use the range from the first spoke (all spokes in a
        # batch should have the same range setting)
        sector.range_min = 0.0
        sector.range_max = float(spokes[0].range) if spokes[0].range > 0 else 1000.0

        # Intensities — one RadarEcho per spoke (ray)
        sector.intensities = []
        for spoke in spokes:
            echo = RadarEcho()
            # Convert byte intensities to float32
            # Values 0-15 are "Normal" returns, 16+ are metadata/history
            # Normalize 0-15 range to 0.0-1.0 for EchoFlow
            echo.echoes = [float(b) / 15.0 if b <= 15 else 0.0
                           for b in spoke.data]
            sector.intensities.append(echo)

        self.radar_pub.publish(sector)
        self.total_sectors += 1

    def _publish_odom(self, stamp):
        """Publish Odometry from Mayara's position/heading data."""
        if self.last_lat is None or self.last_lon is None:
            return

        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        # Position — use lat/lon directly as x/y for now
        # A proper implementation would use a UTM or local tangent plane projection
        # For simulation purposes, we'll convert to approximate meters from origin
        odom.pose = PoseWithCovariance()
        odom.pose.pose = Pose()
        odom.pose.pose.position = Point()
        odom.pose.pose.position.x = self.last_lon  # degrees (for sim, or convert to meters)
        odom.pose.pose.position.y = self.last_lat
        odom.pose.pose.position.z = 0.0

        # Orientation from bearing
        if self.last_bearing is not None:
            # Convert spoke bearing (0=North, CW) to ROS yaw (0=East, CCW)
            bearing_rad = self.last_bearing * self.angle_increment
            yaw = math.pi / 2.0 - bearing_rad  # convert from North-CW to East-CCW
            odom.pose.pose.orientation = _yaw_to_quaternion(yaw)
        else:
            odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        odom.twist = TwistWithCovariance()
        odom.twist.twist = Twist()

        self.odom_pub.publish(odom)
        self.total_odom += 1

    def _log_stats(self):
        """Periodic stats logging."""
        elapsed = time.time() - self.last_stats_time
        if elapsed > 0:
            spoke_rate = self.total_spokes / elapsed if self.total_spokes > 0 else 0
            self.get_logger().info(
                f"Stats: {self.total_spokes} spokes ({spoke_rate:.0f}/s), "
                f"{self.total_sectors} sectors, {self.total_odom} odom msgs, "
                f"scan_time={self.scan_time_sec:.3f}s"
            )
        self.total_spokes = 0
        self.total_sectors = 0
        self.total_odom = 0
        self.last_stats_time = time.time()

    def destroy_node(self):
        self.running = False
        if self.ws:
            self.ws.close()
        super().destroy_node()


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert yaw angle (radians) to a Quaternion (rotation about Z)."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def main(args=None):
    rclpy.init(args=args)
    node = MayaraRosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
