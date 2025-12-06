from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # 1) 아두이노 센서 → ROS2 전달
        Node(
            package='limo_fire_py',
            executable='fire_sensor_reader',
            name='fire_sensor_reader',
            output='screen'
        ),

        # 2) 화재 감지 → 로봇 정지
        Node(
            package='limo_fire_py',
            executable='fire_safety_controller',
            name='fire_safety_controller',
            output='screen'
        ),

        # 3) 화재 감지 → 텔레그램 알림
        Node(
            package='limo_fire_py',
            executable='fire_alarm_notifier',
            name='fire_alarm_notifier',
            output='screen'
        ),

        # 🔥 필요하면 나중에 추가 (pygame BT 시각화)
        # Node(
        #     package='limo_fire_py',
        #     executable='pygame_visualizer',
        #     name='bt_visualizer',
        #     output='screen'
        # ),
    ])
