import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import json
import time

class FireSensorPublisher(Node):
    def __init__(self):
        super().__init__("fire_sensor_publisher")

        # 아두이노 포트 (윈도우는 COM3/COM4, LIMO는 /dev/ttyACM0)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)
        time.sleep(2)

        self.pub = self.create_publisher(String, "fire_sensor", 10)
        self.timer = self.create_timer(0.2, self.read_serial)

    def read_serial(self):
        try:
            raw = self.ser.readline().decode().strip()
            if not raw:
                return

            flame_raw, temp_raw = raw.split(",")
            flame_raw = int(flame_raw)
            temp_raw = float(temp_raw)

            # 불꽃 감지 기준
            flame_detected = 1 if flame_raw < 600 else 0

            msg_dict = {
                "flame": flame_detected,
                "temp": temp_raw
            }

            msg = String()
            msg.data = json.dumps(msg_dict)

            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FireSensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
