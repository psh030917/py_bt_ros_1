#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import json

class FireSensorReader(Node):
    def __init__(self):
        super().__init__("fire_sensor_reader")

        # 아두이노 USB 포트 (ls /dev/ttyACM* 로 확인)
        self.ser = serial.Serial('/dev/ttyACM0', 115200)

        self.pub = self.create_publisher(String, "fire_sensor", 10)
        self.timer = self.create_timer(0.2, self.read_sensor)

    def read_sensor(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode().strip()

            # JSON 형식인지 체크
            if not line.startswith("{"):
                return

            msg = String()
            msg.data = line
            self.pub.publish(msg)
            self.get_logger().info(f"SENSOR: {line}")

def main(args=None):
    rclpy.init(args=args)
    node = FireSensorReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
