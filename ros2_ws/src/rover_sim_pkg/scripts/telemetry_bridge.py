#!/usr/bin/env python3
"""
Telemetry Bridge Node
Listens to ROS 2 /joint_states from simulation and atomically updates
the JSON telemetry cache consumed by the Flask control room dashboard.
"""

import os
import json

RAD_S_TO_RPM = 9.54929658551372  # 60 / (2 * pi)

def rad_per_sec_to_rpm(rad_per_sec: float) -> float:
    """Convert angular velocity from rad/s to Revolutions Per Minute (RPM)."""
    return round(float(rad_per_sec) * RAD_S_TO_RPM, 2)

def extract_wheel_rpms(joint_names, velocities, default_left=0.0, default_right=0.0):
    """
    Extracts left and right wheel RPMs from joint names and velocity vectors.
    Supports named lookup with positional fallback.
    """
    left_rpm = default_left
    right_rpm = default_right

    if not velocities:
        return {"left_rpm": left_rpm, "right_rpm": right_rpm}

    name_map = {}
    if joint_names and len(joint_names) == len(velocities):
        name_map = dict(zip(joint_names, velocities))

    # Check known joint names
    if 'left_wheel_joint' in name_map:
        left_rpm = rad_per_sec_to_rpm(name_map['left_wheel_joint'])
    elif len(velocities) > 0:
        left_rpm = rad_per_sec_to_rpm(velocities[0])

    if 'right_wheel_joint' in name_map:
        right_rpm = rad_per_sec_to_rpm(name_map['right_wheel_joint'])
    elif len(velocities) > 1:
        right_rpm = rad_per_sec_to_rpm(velocities[1])

    return {"left_rpm": left_rpm, "right_rpm": right_rpm}

def write_telemetry_atomic(filepath: str, data: dict):
    """Writes telemetry dict atomically using temporary file and atomic replace."""
    tmp_path = f"{filepath}.tmp"
    with open(tmp_path, 'w') as f:
        json.dump(data, f)
    os.replace(tmp_path, filepath)

# Conditional import so utility functions can be unit-tested without ROS 2 installed
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False


if HAS_RCLPY:
    class TelemetryBridge(Node):
        def __init__(self):
            super().__init__('telemetry_bridge')
            self.declare_parameter('telemetry_file', '/tmp/rover_telemetry.json')
            self.telemetry_path = self.get_parameter('telemetry_file').get_parameter_value().string_value

            self.subscription = self.create_subscription(
                JointState,
                '/joint_states',
                self.listener_callback,
                10)
            self.latest_data = {
                "left_rpm": 0.0,
                "right_rpm": 0.0,
                "system_voltage": 12.4
            }
            self.get_logger().info(f"TelemetryBridge initialized. Output path: {self.telemetry_path}")

        def listener_callback(self, msg):
            rpms = extract_wheel_rpms(msg.name, msg.velocity)
            self.latest_data.update(rpms)
            try:
                write_telemetry_atomic(self.telemetry_path, self.latest_data)
            except Exception as e:
                self.get_logger().warn(f"Failed to write telemetry: {e}")

    def main(args=None):
        rclpy.init(args=args)
        bridge = TelemetryBridge()
        try:
            rclpy.spin(bridge)
        except KeyboardInterrupt:
            pass
        finally:
            bridge.destroy_node()
            rclpy.shutdown()

    if __name__ == '__main__':
        main()