#!/usr/bin/env python3
"""
ORCP ROS 2 driver.

A ROS 2 node that bridges ROS to an ORCP-compliant motor controller via the
`orcp` Python client library. The driver holds the single connection to the
controller and exposes the ORCP surface (except runtime configuration) as ROS
topics, services, and parameters.

Subscribes:
    /cmd_vel        (geometry_msgs/Twist)            unicycle command -> CMD_VEL
    /wheel          (std_msgs/Float64MultiArray)      [left, right] rad/s -> WHEEL

Publishes:
    /odom           (nav_msgs/Odometry)              dead-reckoned pose + velocity
    /battery_state  (sensor_msgs/BatteryState)       battery voltage
    /diagnostics    (diagnostic_msgs/DiagnosticArray) full STATUS (fault/estop/mode/...)
    TF: odom -> base_link

Services:
    ~/stop          (std_srvs/Trigger)               immediate STOP (brake)
    ~/coast         (std_srvs/Trigger)               STOP by coasting to rest
    ~/hold          (std_srvs/Trigger)               STOP and hold position
    ~/enable        (std_srvs/SetBool)               ENABLE ON (true) / OFF (false)

~/coast and ~/hold are ORCP vendor extensions. A controller without them
degrades to a brake, which is safe. ⚠️ A position hold is a CONVENIENCE, NOT A
SAFETY FUNCTION: it needs power, a live controller and working encoders, and
releases on any power-stage fault — at which point /diagnostics reports ERROR
"position hold RELEASED". Never use it as a parking brake on a gradient.

Parameters:
    port          (str)    controller location: /dev/ttyACM0, socket://host:port, /tmp/orcp
    preset        (str)    'SLOW' or 'NORMAL' — set at runtime to switch presets;
                           NORMAL automatically starts a background heartbeat.
    odom_frame    (str)    default 'odom'
    base_frame    (str)    default 'base_link'
    publish_rate  (float)  odom/TF/battery rate, Hz (default 20.0)
    status_rate   (float)  /diagnostics (STATUS poll) rate, Hz (default 4.0)
    cmd_timeout   (float)  ROS-layer command watchdog: if > 0, stop the robot
                           when no /cmd_vel or /wheel message has arrived within
                           this many seconds. Default 0.0 (disabled). This is a
                           dead-man at the ROS layer, independent of the
                           controller's heartbeat. Settable at runtime.

Runtime configuration (GET/SET/SAVE/LOAD/DEFAULTS) is intentionally not exposed.
"""
import math
import threading

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger, SetBool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_ros import TransformBroadcaster

from orcp import ORCP, CommandError, HoldRefused


class OrcpDriver(Node):
    def __init__(self):
        super().__init__("orcp_driver")

        # --- Parameters ---
        self.declare_parameter("port", "/tmp/orcp")
        self.declare_parameter("preset", "SLOW")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("status_rate", 4.0)
        self.declare_parameter("cmd_timeout", 0.0)
        port = self.get_parameter("port").value
        self.preset = self.get_parameter("preset").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        rate = float(self.get_parameter("publish_rate").value)
        status_rate = float(self.get_parameter("status_rate").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)

        # --- Connect & configure from the controller ---
        self.get_logger().info(f"Connecting to ORCP controller at {port} ...")
        self.robot = ORCP(port)
        self.robot.ping()
        self._apply_preset(self.preset)
        info = self.robot.info()
        self.hw = info.hw
        self.wheel_radius = float(self.robot.get("kin.wheel_radius"))
        self.track_width = float(self.robot.get("kin.track_width"))
        self.get_logger().info(
            f"Connected: hw={info.hw} fw={info.fw} {info.proto} | preset={self.preset} | "
            f"wheel_radius={self.wheel_radius} track_width={self.track_width}"
        )

        # --- Odometry state (written from the stream thread, read by the timer) ---
        self._lock = threading.Lock()
        self._vl = self._vr = 0.0
        self._vbat = 0.0
        self._battery = ""
        self.x = self.y = self.yaw = 0.0
        self._last_t = self.get_clock().now()

        # --- /cmd_vel watchdog state (ROS thread only — no lock needed) ---
        self._last_cmd_t = self.get_clock().now()
        self._got_cmd = False
        self._watchdog_stopped = False

        # --- Subscriptions ---
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(Float64MultiArray, "wheel", self.on_wheel, 10)

        # --- Publishers ---
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.batt_pub = self.create_publisher(BatteryState, "battery_state", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Services ---
        self.create_service(Trigger, "~/stop", self.srv_stop)
        self.create_service(Trigger, "~/coast", self.srv_coast)
        self.create_service(Trigger, "~/hold", self.srv_hold)
        self.create_service(SetBool, "~/enable", self.srv_enable)

        # --- Runtime preset switching via the 'preset' parameter ---
        self.add_on_set_parameters_callback(self.on_set_parameters)

        # --- Faults -> ROS warnings ---
        self.robot.on_fault(lambda ev: self.get_logger().warn(f"controller FAULT: {ev.code}"))

        # --- Telemetry stream (background thread just stashes the latest values) ---
        self.robot.stream_on(rate=int(rate), callback=self._on_stream)

        # --- Timers (run on the ROS thread) ---
        self.create_timer(1.0 / rate, self._publish_odom)
        self.create_timer(1.0 / status_rate, self._publish_status)
        self.create_timer(0.05, self._check_cmd_watchdog)  # 20 Hz command watchdog
        self.get_logger().info(
            "Ready: /cmd_vel, /wheel | /odom, /battery_state, /diagnostics, TF | "
            "services ~/stop, ~/coast, ~/hold, ~/enable | param 'preset'."
        )

    # ---- preset / heartbeat management ----

    def _apply_preset(self, preset: str):
        self.robot.preset(preset)
        # NORMAL needs a heartbeat; SLOW does not. The library runs it in a thread.
        if preset == "NORMAL":
            self.robot.start_heartbeat(interval=0.1)
        else:
            self.robot.stop_heartbeat()
        self.preset = preset

    def on_set_parameters(self, params):
        for p in params:
            if p.name == "preset":
                if p.value not in ("SLOW", "NORMAL"):
                    return SetParametersResult(
                        successful=False, reason="preset must be 'SLOW' or 'NORMAL'")
                try:
                    self._apply_preset(p.value)
                    self.get_logger().info(f"preset -> {p.value}")
                except Exception as exc:
                    return SetParametersResult(successful=False, reason=str(exc))
            elif p.name == "cmd_timeout":
                self.cmd_timeout = float(p.value)
                self._watchdog_stopped = False     # re-arm
                self.get_logger().info(f"cmd_timeout -> {self.cmd_timeout}s")
        return SetParametersResult(successful=True)

    # ---- ROS -> controller ----

    def on_cmd_vel(self, msg: Twist):
        self._note_command()
        try:
            self.robot.cmd_vel(v=msg.linear.x, w=msg.angular.z)
        except Exception as exc:
            self.get_logger().warn(f"cmd_vel failed: {exc}")

    def on_wheel(self, msg: Float64MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn("/wheel expects data=[left_rad_s, right_rad_s]")
            return
        self._note_command()
        try:
            self.robot.wheel(l=msg.data[0], r=msg.data[1])
        except Exception as exc:
            self.get_logger().warn(f"wheel failed: {exc}")

    # ---- /cmd_vel watchdog (ROS-layer dead-man, independent of the HB) ----

    def _note_command(self):
        """Record that a fresh drive command arrived (re-arms the watchdog)."""
        self._got_cmd = True
        self._watchdog_stopped = False
        self._last_cmd_t = self.get_clock().now()

    def _check_cmd_watchdog(self):
        """If commands have gone silent for longer than cmd_timeout, STOP once."""
        if self.cmd_timeout <= 0.0 or not self._got_cmd or self._watchdog_stopped:
            return
        silent = (self.get_clock().now() - self._last_cmd_t).nanoseconds * 1e-9
        if silent > self.cmd_timeout:
            self._watchdog_stopped = True
            try:
                self.robot.stop()
            except Exception as exc:
                self.get_logger().warn(f"watchdog stop failed: {exc}")
            self.get_logger().warn(
                f"cmd_vel watchdog: no command for {silent:.2f}s "
                f"(> {self.cmd_timeout}s) — stopping"
            )

    def srv_stop(self, request, response):
        try:
            self.robot.stop()
            response.success = True
            response.message = "stopped (brake)"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def srv_coast(self, request, response):
        """Stop by coasting to rest. Vendor extension; controllers that lack it
        simply brake, which is a safe degradation."""
        try:
            self.robot.stop("COAST")
            response.success = True
            response.message = "coasting to rest"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def srv_hold(self, request, response):
        """Stop and actively hold position.

        ⚠️ A CONVENIENCE, NOT A SAFETY FUNCTION. It requires power, a live
        controller and working encoders, and releases on any power-stage fault.
        Do not use it as a parking brake on a gradient — that needs a mechanical
        or electrically-released brake in the drivetrain.
        """
        try:
            self.robot.hold()
            response.success = True
            response.message = "holding position"
        except HoldRefused as exc:
            # ⚠️ The stop DID happen; only the hold was refused. Reported as a
            # failed service call because the caller asked for a hold and has
            # not got one — but the message must not imply the robot is still
            # moving.
            response.success = False
            response.message = f"stopped, but NOT holding: {exc.reason}"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def srv_enable(self, request, response):
        try:
            if request.data:
                self.robot.enable()
                response.message = "enabled (ENABLE ON)"
            else:
                self.robot.disable()
                response.message = "disabled (ENABLE OFF)"
            response.success = True
        except CommandError as exc:
            response.success = False
            response.message = f"{exc.code}: {exc.message}"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    # ---- controller -> ROS ----

    def _on_stream(self, data):
        with self._lock:
            self._vl, self._vr = data.vl, data.vr
            self._vbat, self._battery = data.vbat, data.battery

    def _publish_odom(self):
        now = self.get_clock().now()
        dt = (now - self._last_t).nanoseconds * 1e-9
        self._last_t = now
        if dt <= 0.0:
            return

        with self._lock:
            vl, vr, vbat, battery = self._vl, self._vr, self._vbat, self._battery

        v_left = vl * self.wheel_radius
        v_right = vr * self.wheel_radius
        v = (v_right + v_left) / 2.0
        w = (v_right - v_left) / self.track_width

        self.yaw += w * dt
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        stamp = now.to_msg()

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

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = math.sin(self.yaw / 2.0)
        tf.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf_broadcaster.sendTransform(tf)

        batt = BatteryState()
        batt.header.stamp = stamp
        batt.voltage = float(vbat)
        batt.percentage = (float(battery.rstrip("%")) / 100.0
                           if battery.endswith("%") else float("nan"))
        batt.present = vbat > 1.0
        self.batt_pub.publish(batt)

    def _publish_status(self):
        """Poll STATUS and publish the full controller state as diagnostics."""
        try:
            st = self.robot.status()
        except Exception as exc:
            self.get_logger().warn(f"status poll failed: {exc}")
            return

        s = DiagnosticStatus()
        s.name = "orcp_driver: controller"
        s.hardware_id = self.hw
        if st.estop or st.fault:
            s.level = DiagnosticStatus.ERROR
            s.message = st.fault or "ESTOP"
        elif st.hold_broken:
            # ⚠️ hold=2: the controller WAS holding position and is not any
            # more, because a fault or its thermal limit ended the hold. On a
            # gradient the robot may now be moving. ERROR rather than WARN — a
            # released hold is not a degraded state, it is the absence of the
            # thing that was being relied on.
            s.level = DiagnosticStatus.ERROR
            s.message = "position hold RELEASED (fault or timeout)"
        elif self.preset == "NORMAL" and not st.enabled:
            s.level = DiagnosticStatus.WARN
            s.message = "not enabled"
        else:
            s.level = DiagnosticStatus.OK
            s.message = "OK"
        fields = {
            "preset": st.preset, "mode": st.mode,
            "enabled": str(st.enabled), "estop": str(st.estop),
            "fault": st.fault or "OK",
            "vl": f"{st.vl:.3f}", "vr": f"{st.vr:.3f}",
            "dl": f"{st.dl:.3f}", "dr": f"{st.dr:.3f}",
            "duty_limit": f"{st.duty_limit:.3f}",
            "vbat": f"{st.vbat:.2f}", "battery": st.battery,
        }
        # Vendor-extension fields: reported only when the controller has them.
        # ⚠️ Absent and 0 are different answers — publishing "hold: 0" for a
        # controller that cannot hold would read as "not holding right now".
        if st.hold is not None:
            fields["hold"] = {0: "off", 1: "holding", 2: "RELEASED"}.get(
                st.hold, str(st.hold))
        if st.coast is not None:
            fields["coast"] = str(st.coast)
        if self.robot.dropped_lines:
            # Non-zero means the link handed us something that was neither a
            # push nor a response — line noise, or a mid-stream reconnect.
            # Surfaced because a silently degrading link is worth seeing early.
            fields["dropped_lines"] = str(self.robot.dropped_lines)
        s.values = [KeyValue(key=k, value=v) for k, v in fields.items()]

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [s]
        self.diag_pub.publish(arr)

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
