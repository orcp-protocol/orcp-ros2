#!/usr/bin/env python3
"""
ORCP ROS 2 driver.

A ROS 2 node that bridges ROS to an ORCP-compliant motor controller via the
`orcp` Python client library.

Subscribes:
    /cmd_vel   (geometry_msgs/Twist)        drive commands -> ORCP CMD_VEL

Publishes:
    /odom            (nav_msgs/Odometry)         dead-reckoned pose + velocity
    /battery_state   (sensor_msgs/BatteryState)  battery voltage
    TF: odom -> base_link

Telemetry from the controller arrives on the orcp library's background thread;
that thread only stashes the latest values, and a ROS timer does the odometry
integration and publishing on the ROS executor thread (keeps ROS calls on the
ROS thread).

Parameters:
    port          (str)    controller location: /dev/ttyACM0, socket://host:port,
                           or an orcp-sim PTY like /tmp/orcp
    preset        (str)    'SLOW' (default) or 'NORMAL'
    odom_frame    (str)    odometry frame id (default 'odom')
    base_frame    (str)    robot body frame id (default 'base_link')
    publish_rate  (float)  odom/TF publish rate in Hz (default 20.0)
"""
import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformBroadcaster

from orcp import ORCP


class OrcpDriver(Node):
    def __init__(self):
        super().__init__("orcp_driver")

        # --- Parameters ---
        self.declare_parameter("port", "/tmp/orcp")
        self.declare_parameter("preset", "SLOW")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate", 20.0)
        port = self.get_parameter("port").value
        preset = self.get_parameter("preset").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        rate = float(self.get_parameter("publish_rate").value)

        # --- Connect to the controller ---
        self.get_logger().info(f"Connecting to ORCP controller at {port} ...")
        self.robot = ORCP(port)
        self.robot.ping()
        self.robot.preset(preset)
        info = self.robot.info()
        # Self-configure geometry from the controller itself.
        self.wheel_radius = float(self.robot.get("kin.wheel_radius"))
        self.track_width = float(self.robot.get("kin.track_width"))
        self.get_logger().info(
            f"Connected: hw={info.hw} fw={info.fw} {info.proto} | preset={preset} | "
            f"wheel_radius={self.wheel_radius} track_width={self.track_width}"
        )

        # --- Odometry state (protected by a lock; written from two threads) ---
        self._lock = threading.Lock()
        self._vl = 0.0          # latest measured left wheel velocity (rad/s)
        self._vr = 0.0          # latest measured right wheel velocity (rad/s)
        self._vbat = 0.0
        self._battery = ""
        self.x = self.y = self.yaw = 0.0
        self._last_t = self.get_clock().now()

        # --- ROS interfaces ---
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.batt_pub = self.create_publisher(BatteryState, "battery_state", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Surface controller faults as ROS warnings.
        self.robot.on_fault(lambda ev: self.get_logger().warn(f"controller FAULT: {ev.code}"))

        # Telemetry stream -> background callback just stashes the latest values.
        self.robot.stream_on(rate=int(rate), callback=self._on_stream)

        # ROS timer does the integration + publishing on the ROS thread.
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info("Listening on /cmd_vel; publishing /odom, /battery_state, TF.")

    # ---- ROS -> controller ----

    def on_cmd_vel(self, msg: Twist):
        try:
            self.robot.cmd_vel(v=msg.linear.x, w=msg.angular.z)
        except Exception as exc:
            self.get_logger().warn(f"cmd_vel failed: {exc}")

    # ---- controller -> ROS (background thread: stash only) ----

    def _on_stream(self, data):
        with self._lock:
            self._vl = data.vl
            self._vr = data.vr
            self._vbat = data.vbat
            self._battery = data.battery

    # ---- ROS timer: integrate odometry and publish ----

    def _publish(self):
        now = self.get_clock().now()
        dt = (now - self._last_t).nanoseconds * 1e-9
        self._last_t = now
        if dt <= 0.0:
            return

        with self._lock:
            vl, vr, vbat, battery = self._vl, self._vr, self._vbat, self._battery

        # Diff-drive forward kinematics: wheel rad/s -> body linear/angular.
        v_left = vl * self.wheel_radius          # m/s of the left wheel
        v_right = vr * self.wheel_radius
        v = (v_right + v_left) / 2.0             # body linear velocity (m/s)
        w = (v_right - v_left) / self.track_width  # body angular velocity (rad/s)

        # Integrate pose (simple Euler).
        self.yaw += w * dt
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        stamp = now.to_msg()

        # --- /odom ---
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        # --- TF odom -> base_link ---
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = math.sin(self.yaw / 2.0)
        tf.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf_broadcaster.sendTransform(tf)

        # --- /battery_state ---
        batt = BatteryState()
        batt.header.stamp = stamp
        batt.voltage = float(vbat)
        batt.percentage = (float(battery.rstrip("%")) / 100.0
                           if battery.endswith("%") else float("nan"))
        batt.present = vbat > 1.0
        self.batt_pub.publish(batt)

    def destroy_node(self):
        try:
            self.robot.stop()
            self.robot.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OrcpDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
