#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import json

class TelemetryBridge(Node):
    def __init__(self):
        super().__init__('telemetry_bridge')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.latest_data = {"left_rpm": 0.0, "right_rpm": 0.0}

    def listener_callback(self, msg):
        # Translate simulation rad/s to RPM for your dashboard
        self.latest_data['left_rpm'] = round(msg.velocity[0] * 9.549, 2)
        self.latest_data['right_rpm'] = round(msg.velocity[1] * 9.549, 2)
        
        # Save to a file that Flask can read
        with open('/tmp/rover_telemetry.json', 'w') as f:
            json.dump(self.latest_data, f)

def main(args=None):
    rclpy.init(args=args)
    bridge = TelemetryBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()