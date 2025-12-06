import math
import os
from datetime import datetime
from modules.base_bt_nodes import BTNodeList, Status, Node, Sequence, Fallback, ReactiveSequence, ReactiveFallback

# BT Node List
CUSTOM_ACTION_NODES = [
    'MoveToGoal',
    'CaptureImage',
    'Return',
]

CUSTOM_CONDITION_NODES = [
    'IsGoalReached',
    'SaveInitialPose'
]

# BT Node List
BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from modules.base_bt_nodes_ros import ConditionWithROSTopics, ActionWithROSAction, ActionWithROSService


class IsGoalReached(ConditionWithROSTopics):
    """목표 지점 도달 여부 확인 (선택적 사용)"""
    def __init__(self, name, agent, default_thresh=0.5):
        ns = agent.ros_namespace or ""
        super().__init__(name, agent, [
            (PoseStamped, f"/amcl_pose", 'current_pose'),
        ])
        self.default_thresh = default_thresh

    def _predicate(self, agent, blackboard):
        cache = self._cache
        if "current_pose" not in cache:
            return False
        
        goal_pose = blackboard.get('goal_pose')
        if goal_pose is None:
            return False
        
        current = cache["current_pose"]
        dx = current.pose.position.x - goal_pose.pose.position.x
        dy = current.pose.position.y - goal_pose.pose.position.y
        dist = math.sqrt(dx*dx + dy*dy)
        
        return dist <= self.default_thresh


class MoveToGoal(ActionWithROSAction):
    def __init__(self, name, agent):
        # Nav2는 보통 루트 네임스페이스 사용
        super().__init__(name, agent, 
            (NavigateToPose, "/navigate_to_pose")
        )
        self.ros = agent.ros_bridge
        self._goal_pose = None
        self._goal_used = False  # goal이 사용되었는지 추적
        
        # /bt/goal_pose 토픽 구독 (Rviz Nav2 Goal에서 발행)
        goal_topic = "/bt/goal_pose"
        self.ros.node.create_subscription(
            PoseStamped, 
            goal_topic,
            self._goal_callback,
            10
        )
    
    def _goal_callback(self, msg):
        """목표 위치가 들어오면 저장"""
        self._goal_pose = msg
        self._goal_used = False  # 새로운 goal이 도착하면 리셋
        x = msg.pose.position.x
        y = msg.pose.position.y
                
    def _build_goal(self, agent, bb):
        # goal이 이미 사용되었으면 대기
        if self._goal_used:

            return None
            
        if self._goal_pose is None:
            return None
        
        # Blackboard에 goal_pose 저장
        bb['goal_pose'] = self._goal_pose
        
        goal = NavigateToPose.Goal()
        goal.pose = self._goal_pose
        return goal
    
    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            bb['nav_result'] = 'succeeded'
            self._goal_used = True  # goal 사용 완료
            return Status.SUCCESS
        elif status_code == GoalStatus.STATUS_CANCELED:
            bb['nav_result'] = 'canceled'
            return Status.FAILURE
        else:
            bb['nav_result'] = 'aborted'
            return Status.FAILURE


class CaptureImage(Node):
    """
    카메라 이미지를 캡처해서 파일로 저장
    카메라가 없으면 더미 모드로 동작
    """
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self._latest_image = None
        self._bridge = CvBridge()
        
        # 이미지 저장 경로 설정
        self._save_dir = os.path.join(
            os.path.dirname(__file__), 
            "captured_images"
        )
        os.makedirs(self._save_dir, exist_ok=True)
        
        # 카메라 토픽 구독 - TurtleBot3Burger의 카메라
        camera_topic = "/camera/rgb/image_raw"
        self.ros.node.create_subscription(
            Image,
            camera_topic,
            self._image_callback,
            10
        )
        self.type = "Action"
        
        # 카메라 없을 때 더미 모드
        self._dummy_mode = True  # 카메라 추가되면 자동으로 False
    
    def _image_callback(self, msg):
        """최신 이미지 저장"""
        self._latest_image = msg
        self._dummy_mode = False  # 실제 이미지 받으면 더미 모드 해제
    
    async def run(self, agent, blackboard):
        # 더미 모드: 카메라 없어도 성공
        if self._dummy_mode:

            blackboard['capture_result'] = 'succeeded_dummy'
            self.status = Status.SUCCESS
            return self.status
        
        # 실제 모드: 이미지 필요
        if self._latest_image is None:
            self.status = Status.FAILURE
            return self.status
        
        try:
            # ROS Image를 OpenCV 이미지로 변환
            cv_image = self._bridge.imgmsg_to_cv2(self._latest_image, "bgr8")
            
            # 타임스탬프로 파일명 생성
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(self._save_dir, f"image_{timestamp}.jpg")
            
            # 이미지 파일로 저장
            cv2.imwrite(filename, cv_image)
            
            # Blackboard에도 저장
            blackboard['captured_image'] = self._latest_image
            blackboard['capture_result'] = 'succeeded'
            blackboard['image_path'] = filename
            
        except Exception as e:
            self.status = Status.FAILURE
            return self.status
        
        self.status = Status.SUCCESS
        return self.status


class SaveInitialPose(Node):
    """
    로봇의 초기 위치를 blackboard에 저장
    /amcl_pose 또는 /odom 토픽에서 현재 위치를 받아서 저장
    """
    def __init__(self, name, agent):
        super().__init__(name)
        self.type = "Condition"
        self.ros = agent.ros_bridge
        self._current_pose = None
        self._start_time = None
        
        # /amcl_pose 토픽 구독 (우선)
        self.ros.node.create_subscription(
            PoseStamped,
            '/amcl_pose',
            self._amcl_callback,
            10
        )
        
        # /odom 토픽도 구독 (백업)
        from nav_msgs.msg import Odometry
        self.ros.node.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10
        )
    
    def _amcl_callback(self, msg):
        """amcl_pose 업데이트 (우선순위)"""
        self._current_pose = msg
    
    def _odom_callback(self, msg):
        """odom에서 위치 변환 (백업용)"""
        if self._current_pose is None:  # amcl_pose 없을 때만
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose = msg.pose.pose
            self._current_pose = pose_stamped

    async def run(self, agent, blackboard):
        # 초기 위치가 아직 저장 안 됐으면 저장
        if "initial_pose" not in blackboard:
            if self._start_time is None:
                self._start_time = self.ros.node.get_clock().now()
            
            # 5초 타임아웃
            elapsed = (self.ros.node.get_clock().now() - self._start_time).nanoseconds / 1e9
            
            if self._current_pose is None:
                if elapsed > 5.0:
                    # 타임아웃: 고정 위치 사용

                    initial_pose = PoseStamped()
                    initial_pose.header.frame_id = 'map'
                    initial_pose.header.stamp = self.ros.node.get_clock().now().to_msg()
                    initial_pose.pose.position.x = 6.36
                    initial_pose.pose.position.y = 0.0
                    initial_pose.pose.orientation.w = 1.0
                    blackboard["initial_pose"] = initial_pose
                else:
                    # 아직 대기 중
                    self.status = Status.RUNNING
                    return self.status
            else:
                # 현재 위치를 초기 위치로 저장
                blackboard["initial_pose"] = self._current_pose
                x = self._current_pose.pose.position.x
                y = self._current_pose.pose.position.y
        self.status = Status.SUCCESS
        return self.status
    

class Return(ActionWithROSAction):
    """
    저장된 초기 위치로 돌아가는 노드
    """
    def __init__(self, name, agent):
        # Nav2는 보통 루트 네임스페이스 사용
        super().__init__(name, agent, 
            (NavigateToPose, "/navigate_to_pose")
        )

    def _build_goal(self, agent, bb):
        initial_pose = bb.get('initial_pose')
        if initial_pose is None:
            return None
        
        goal = NavigateToPose.Goal()
        goal.pose = initial_pose
        return goal

    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            bb['return_result'] = 'succeeded'
            # Blackboard 초기화 (다음 미션 준비)
            bb.pop('initial_pose', None)
            bb.pop('goal_pose', None)
            bb.pop('nav_result', None)
            bb.pop('capture_result', None)
            # FAILURE 반환하여 Sequence를 다시 처음부터 시작
            return Status.FAILURE
        elif status_code == GoalStatus.STATUS_CANCELED:
            bb['return_result'] = 'canceled'
            return Status.FAILURE
        else:
            bb['return_result'] = 'aborted'
            return Status.FAILURE
