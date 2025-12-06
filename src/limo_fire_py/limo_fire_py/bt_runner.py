#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class BTRunner(Node):
    def __init__(self):
        super().__init__("bt_runner")

        self.publisher_ = self.create_publisher(String, "bt_status", 10)

        # 데모용 BT 단계들
        self.steps = [
            "[RUNNING] CheckFireStatus",
            "[SUCCESS] CheckFireStatus",
            "[RUNNING] TriggerAlarm",
            "[SUCCESS] TriggerAlarm",
            "[RUNNING] PublishInfo",
            "[SUCCESS] PublishInfo",
            "[IDLE] BT Finished"
        ]
        self.index = 0
        # 1초마다 step() 호출
        self.timer = self.create_timer(1.0, self.step)

    def step(self):
        msg = String()

        if self.index < len(self.steps):
            msg.data = self.steps[self.index]
            self.index += 1
        else:
            msg.data = "[IDLE] Waiting..."

        self.get_logger().info(f"BT: {msg.data}")
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BTRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
