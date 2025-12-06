#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json

# 본인 텔레그램 정보 넣기!!!
BOT_TOKEN = "8306753560:AAHF1FeWaaLyor3XyDvFfS7IxAPJoIRLUqA"
CHAT_ID = "8561902821"

class FireAlarmNotifier(Node):
    def __init__(self):
        super().__init__("fire_alarm_notifier")

        # fire_sensor 토픽 수신
        self.sub = self.create_subscription(
            String, "fire_sensor", self.on_sensor_msg, 10
        )

        self.fire_sent = False  # 중복 알림 방지

    def send_telegram(self, text):
        url = f"https://api.telegram.org/bot{BOT_TOKEN}/sendMessage"
        params = {
            "chat_id": CHAT_ID,
            "text": text
        }
        try:
            requests.get(url, params=params)
            self.get_logger().info("Telegram Alarm Sent!")
        except Exception as e:
            self.get_logger().error(f"Telegram Error: {e}")

    def on_sensor_msg(self, msg):
        try:
            data = json.loads(msg.data)
            flame = data.get("flame", 0)
            temp = data.get("temp", 0)

            # 화재 조건
            if (flame == 1 or temp < 600) and not self.fire_sent:
                self.send_telegram("🔥🔥 화재 감지! 로봇이 즉시 정지합니다! 🔥🔥")
                self.fire_sent = True

            if flame == 0 and temp < 600:
                self.fire_sent = False  # 다시 알림 가능

        except Exception as e:
            self.get_logger().error(f"JSON Parse Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FireAlarmNotifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
