#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json

class FireSafetyController(Node):
    def __init__(self):
        super().__init__("fire_safety_controller")

        # fire_sensor 토픽 받아오기
        self.sub = self.create_subscription(
            String, "fire_sensor", self.sensor_callback, 10
        )

        # 로봇 정지 명령 퍼블리셔
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.fire_detected = False

    def sensor_callback(self, msg):
        try:
            data = json.loads(msg.data)
            flame = data.get("flame", 0)
            temp = data.get("temp", 0)

            # 화재 감지 조건
            if flame == 1 or temp < 600:
                if not self.fire_detected:
                    self.get_logger().warn("🔥 FIRE DETECTED! STOPPING ROBOT!")
                self.fire_detected = True
                self.stop_robot()
            else:
                self.fire_detected = False

        except Exception as e:
            self.get_logger().error(f"JSON Parse Error: {e}")

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FireSafetyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
