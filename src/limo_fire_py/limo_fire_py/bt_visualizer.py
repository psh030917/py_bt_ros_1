#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame

class BTVisualizer(Node):
    def __init__(self):
        super().__init__("bt_visualizer")

        # ROS2 구독자
        self.subscription = self.create_subscription(
            String, "bt_status", self.bt_callback, 10
        )

        # Pygame 초기화
        pygame.init()
        self.screen = pygame.display.set_mode((900, 450))
        pygame.display.set_caption("Behavior Tree Visualizer")

        self.font = pygame.font.SysFont(None, 30)

        # BT 노드 구조
        self.nodes = ["CheckFireStatus", "TriggerAlarm", "PublishInfo"]

        # 노드 좌표
        self.positions = {
            "Sequence": (450, 100),
            "CheckFireStatus": (250, 300),
            "TriggerAlarm": (450, 300),
            "PublishInfo": (650, 300)
        }

        # 부모-자식 간선
        self.edges = [
            ("Sequence", "CheckFireStatus"),
            ("Sequence", "TriggerAlarm"),
            ("Sequence", "PublishInfo")
        ]

        # 상태 초기화
        # 0=idle, 1=running, 2=success
        self.states = {
            "Sequence": 2,  # 고정
            "CheckFireStatus": 0,
            "TriggerAlarm": 0,
            "PublishInfo": 0
        }

        self.timer = self.create_timer(0.05, self.update_screen)

    # -------------------------------
    # ROS2 메시지 처리
    # -------------------------------
    def bt_callback(self, msg):
        text = msg.data

        # 초기화
        for n in self.nodes:
            if "IDLE" in text:
                self.states[n] = 0

        # running
        if "[RUNNING]" in text:
            node = text.split("]")[1].strip()
            self.states[node] = 1

        # success
        if "[SUCCESS]" in text:
            node = text.split("]")[1].strip()
            self.states[node] = 2

    # -------------------------------
    # BT 트리 그리기
    # -------------------------------
    def draw_tree(self):
        self.screen.fill((255, 255, 255))

        # 간선
        for parent, child in self.edges:
            px, py = self.positions[parent]
            cx, cy = self.positions[child]
            pygame.draw.line(self.screen, (0, 0, 0), (px, py+20), (cx, cy-20), 3)

        # 상태 색상
        colors = {
            0: (160, 160, 160),  # idle
            1: (0, 120, 255),    # running
            2: (255, 70, 70)     # success
        }

        # Sequence Node
        sx, sy = self.positions["Sequence"]
        pygame.draw.rect(self.screen, (200, 200, 255), (sx-100, sy-25, 200, 50), border_radius=12)
        text = self.font.render("Sequence", True, (0, 0, 0))
        self.screen.blit(text, (sx - text.get_width()//2, sy - 10))

        # 자식 노드들
        for node in self.nodes:
            x, y = self.positions[node]
            color = colors[self.states[node]]

            pygame.draw.rect(self.screen, color, (x-100, y-25, 200, 50), border_radius=12)
            label = self.font.render(node, True, (255, 255, 255))
            self.screen.blit(label, (x - label.get_width()//2, y - 10))

    def update_screen(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        self.draw_tree()
        pygame.display.flip()

def main(args=None):
    rclpy.init(args=args)
    node = BTVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
