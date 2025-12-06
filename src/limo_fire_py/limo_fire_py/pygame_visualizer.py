#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import threading

class PygameVisualizer(Node):
    def __init__(self):
        super().__init__("pygame_visualizer")

        self.current_status = "Waiting for BT..."

        self.subscription = self.create_subscription(
            String,
            "bt_status",
            self.bt_callback,
            10
        )

        # pygame을 별도 스레드에서 실행
        thread = threading.Thread(target=self.run_pygame, daemon=True)
        thread.start()

    def bt_callback(self, msg: String):
        self.current_status = msg.data

    def run_pygame(self):
        pygame.init()
        screen = pygame.display.set_mode((800, 300))
        pygame.display.set_caption("Behavior Tree Status Viewer")

        font = pygame.font.SysFont(None, 40)

        running = True
        clock = pygame.time.Clock()

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            screen.fill((30, 30, 30))

            # 상태 문자열 표시
            text_surface = font.render(self.current_status, True, (255, 255, 255))
            screen.blit(text_surface, (20, 120))

            pygame.display.flip()
            clock.tick(30)

        pygame.quit()

def main(args=None):
    rclpy.init(args=args)
    node = PygameVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
