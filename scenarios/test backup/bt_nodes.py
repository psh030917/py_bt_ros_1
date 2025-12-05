import math
import os
import cv2
os.environ.setdefault('QT_QPA_PLATFORM', 'xcb')
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from modules.base_bt_nodes import BTNodeList, Status, Node, Sequence, Fallback, ReactiveSequence, ReactiveFallback
from modules.base_bt_nodes_ros import ConditionWithROSTopics

try:
    from ultralytics import YOLO
except Exception:
    YOLO = None

# BT Node List
CUSTOM_ACTION_NODES = [
    'CaptureCameraImage',
    'DisplayYOLODetection',
    'CaptureScreen'
]

CUSTOM_CONDITION_NODES = [
    'IsDetectedSomething'
]

# BT Node List
BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)

class CaptureCameraImage(ConditionWithROSTopics):
    """
    지정된 카메라 토픽을 구독하여 최신 이미지를 blackboard에 저장.
    - 카메라 토픽: /camera/color/image_raw (### 경우에 따라 토픽 변경)
    - blackboard['camera_image']에 최신 이미지 저장
    - 항상 SUCCESS 반환 (이미지가 없으면 RUNNING)
    """
    def __init__(self, name, agent, camera_topic="psh"):  ### 경우에 따라 토픽 변경
        super().__init__(name, agent, [
            (Image, camera_topic, 'camera_image'),
        ])
        self.type = "Action"  # Action으로 사용
        self.camera_topic = camera_topic
        agent.ros_bridge.node.get_logger().info(f'CaptureCameraImage: 구독 시작 - 토픽: {camera_topic}')
    
    def _predicate(self, agent, blackboard) -> bool:
        """이미지가 있으면 blackboard에 저장하고 SUCCESS"""
        cache = self._cache
        if "camera_image" in cache:
            # 최신 이미지를 blackboard에 저장
            blackboard['camera_image'] = cache["camera_image"]
            agent.ros_bridge.node.get_logger().debug(f'CaptureCameraImage: 이미지 수신됨 (토픽: {self.camera_topic})')
            return True
        else:
            # 이미지가 없으면 blackboard에서 제거 (이전 이미지가 남아있지 않도록)
            if blackboard.get('camera_image') is not None:
                agent.ros_bridge.node.get_logger().warning(f'CaptureCameraImage: 이미지 수신 실패 - 토픽 "{self.camera_topic}"에서 메시지를 받지 못함. blackboard 초기화.')
            blackboard['camera_image'] = None
            return False

# 모듈 레벨 변수로 초기화 상태 관리 (ReactiveSequence가 halt()를 매 틱마다 호출하므로 모듈 변수 사용)
_display_yolo_initialized = False
_display_yolo_publisher = None
_display_yolo_yolo_model = None
_display_yolo_window_created = False

class DisplayYOLODetection(Node):
    """
    YOLO 검출을 수행하고 결과를 ROS 토픽으로 퍼블리시하는 노드.
    - blackboard['camera_image']에서 이미지 가져오기
    - YOLO 모델 경로: blackboard['yolo_model_path'] 또는 기본값 'yolov8n.pt'
    - ROS 토픽: blackboard['output_topic'] 또는 기본값 '/yolo/image' (### 경우에 따라 토픽 변경)
    - 매 틱마다 카메라 이미지를 읽어 YOLO 검출 후 ROS 토픽으로 퍼블리시
    """
    
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self._bridge = CvBridge()
        self.type = "Action"
    
    async def run(self, agent, blackboard):
        global _display_yolo_initialized, _display_yolo_publisher
        global _display_yolo_yolo_model, _display_yolo_window_created
        
        # 설정 가져오기
        model_path = blackboard.get('yolo_model_path', 'yolov10n.pt')
        output_topic = blackboard.get('output_topic', '/yolo/image')  ### 경우에 따라 토픽 변경
        
        # 초기화 (최초 1회만)
        if not _display_yolo_initialized:
            if not self._initialize(model_path, output_topic):
                self.status = Status.FAILURE
                return self.status
        
        # blackboard에서 카메라 이미지 가져오기
        camera_image_msg = blackboard.get('camera_image')
        if camera_image_msg is None:
            self.ros.node.get_logger().warning('No camera image in blackboard yet - DisplayYOLODetection cannot display')
            self.status = Status.RUNNING
            return self.status
        
        # ROS Image 메시지를 OpenCV 이미지로 변환
        try:
            frame = self._bridge.imgmsg_to_cv2(camera_image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.ros.node.get_logger().error(f'Failed to convert image: {e}', exc_info=True)
            self.status = Status.FAILURE
            return self.status
        
        # YOLO 검출 및 퍼블리시
        try:
            # YOLO 검출 수행
            results = _display_yolo_yolo_model(frame, verbose=False)
            res = results[0]
            
            # 검출 결과 파싱
            detections = self._parse_detections(res)
            
            # blackboard에 검출 결과 저장
            annotated_frame = res.plot()
            blackboard['yolo_detections'] = detections
            blackboard['current_frame'] = frame
            blackboard['annotated_frame'] = annotated_frame
            
            # OpenCV 윈도우에 표시
            try:
                cv2.imshow("YOLO Detection", annotated_frame)
                cv2.waitKey(1)
                self.ros.node.get_logger().debug(f'Displayed frame in OpenCV window: shape={annotated_frame.shape}')
            except Exception as e:
                self.ros.node.get_logger().error(f'Failed to display OpenCV window: {e}', exc_info=True)
            
            # ROS 토픽으로 퍼블리시
            self._publish_frame(annotated_frame)
            
            self.status = Status.SUCCESS
            return self.status
            
        except Exception as e:
            self.ros.node.get_logger().error(f'YOLO processing failed: {e}', exc_info=True)
            self.status = Status.FAILURE
            return self.status
    
    def _initialize(self, model_path, output_topic):
        """초기화 로직 (최초 1회만 실행)"""
        global _display_yolo_initialized, _display_yolo_publisher
        global _display_yolo_yolo_model, _display_yolo_window_created
        
        try:
            # YOLO 모델 로드
            if YOLO is None:
                self.ros.node.get_logger().error('ultralytics YOLO package not found. Install `ultralytics` to use this node.')
                return False
            
            _display_yolo_yolo_model = YOLO(model_path)
            self.ros.node.get_logger().info(f'YOLO model loaded: {model_path}')
            
            # ROS 퍼블리셔 생성
            _display_yolo_publisher = self.ros.node.create_publisher(Image, output_topic, 10)
            
            # OpenCV 윈도우 생성
            try:
                cv2.namedWindow("YOLO Detection", cv2.WINDOW_NORMAL)
                _display_yolo_window_created = True
                self.ros.node.get_logger().info('OpenCV window "YOLO Detection" created successfully')
            except Exception as e:
                self.ros.node.get_logger().error(f'Failed to create OpenCV window: {e}', exc_info=True)
                return False
            
            _display_yolo_initialized = True
            self.ros.node.get_logger().info(f'YOLO Detection initialized: model={model_path}, topic={output_topic}')
            return True
            
        except Exception as e:
            self.ros.node.get_logger().error(f'Initialization failed: {e}', exc_info=True)
            return False
    
    def _parse_detections(self, res):
        """YOLO 검출 결과를 파싱하여 리스트로 반환"""
        detections = []
        if hasattr(res, 'boxes') and res.boxes is not None:
            try:
                boxes = res.boxes.xyxy.cpu().numpy()
                confs = res.boxes.conf.cpu().numpy()
                cls_inds = res.boxes.cls.cpu().numpy().astype(int)
                
                for box, conf, cls_i in zip(boxes, confs, cls_inds):
                    class_name = _display_yolo_yolo_model.names[int(cls_i)] if hasattr(_display_yolo_yolo_model, 'names') else str(int(cls_i))
                    detections.append({
                        'class': class_name,
                        'confidence': float(conf),
                        'bbox': [float(box[0]), float(box[1]), float(box[2]), float(box[3])]
                    })
            except Exception as e:
                self.ros.node.get_logger().error(f'Failed to parse detections: {e}', exc_info=True)
        return detections
    
    def _publish_frame(self, annotated_frame):
        """검출된 프레임을 ROS 토픽으로 퍼블리시"""
        try:
            msg = self._bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            msg.header.stamp = self.ros.node.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            _display_yolo_publisher.publish(msg)
        except Exception as e:
            self.ros.node.get_logger().error(f'Failed to publish: {e}', exc_info=True)
    
    def halt(self):
        """노드 중지 시 정리 (ReactiveSequence가 매 틱마다 호출하므로 초기화 플래그는 유지)"""
        # ReactiveSequence가 매 틱마다 halt()를 호출하므로
        # 여기서 초기화 플래그를 리셋하면 안 됩니다.
        pass

class IsDetectedSomething(Node):
    """
    blackboard에서 YOLO 검출 결과를 확인하여 새로운 객체가 감지되었는지 확인.
    - 감지할 클래스: XML 파라미터 'target_class' (기본값: "person")
    - blackboard['yolo_detections']에서 지정된 클래스 검색
    - confidence threshold: XML 파라미터 'confidence_threshold' 또는 기본값 0.5
    - 이전 프레임 대비 객체 개수가 증가했을 때만 SUCCESS, 아니면 FAILURE
    """
    def __init__(self, name, agent, target_class="person", confidence_threshold=0.5):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.target_class = target_class.lower()  # 소문자로 변환하여 비교
        self.default_confidence_threshold = confidence_threshold
        self.type = "Condition"
        agent.ros_bridge.node.get_logger().info(
            f'IsDetectedSomething: 감지 대상 클래스 = "{self.target_class}", confidence threshold = {confidence_threshold}'
        )
    
    async def run(self, agent, blackboard):
        # 검출 결과 가져오기
        detections = blackboard.get('yolo_detections', [])
        
        # confidence threshold 가져오기 (blackboard 우선, 없으면 기본값)
        threshold_key = f'{self.target_class}_confidence_threshold'
        threshold = blackboard.get(threshold_key, self.default_confidence_threshold)
        
        # 현재 프레임의 객체 개수 계산 (confidence threshold 이상만)
        current_count = 0
        detected_objects = []
        for detection in detections:
            class_name = detection.get('class', '').lower()
            if class_name == self.target_class:
                confidence = detection.get('confidence', 0.0)
                if confidence >= threshold:
                    current_count += 1
                    detected_objects.append(detection)
        
        # 이전 프레임의 개수 가져오기 (초기값: 0)
        previous_count_key = f'previous_{self.target_class}_count'
        previous_count = blackboard.get(previous_count_key, 0)
        
        # 새로운 객체가 추가되었는지 확인 (현재 개수 > 이전 개수)
        if current_count > previous_count:
            # 새로운 객체 감지됨
            if detected_objects:
                # 가장 높은 confidence인 객체 정보 저장
                best_object = max(detected_objects, key=lambda x: x.get('confidence', 0.0))
                blackboard[f'detected_{self.target_class}'] = best_object
                blackboard[f'{self.target_class}_confidence'] = best_object.get('confidence', 0.0)
            blackboard[previous_count_key] = current_count
            self.ros.node.get_logger().info(
                f'New {self.target_class} detected! Previous: {previous_count}, Current: {current_count}'
            )
            self.status = Status.SUCCESS
            return self.status
        
        # 개수가 증가하지 않음 (같거나 감소)
        blackboard[previous_count_key] = current_count
        if current_count == 0:
            # 객체가 없는 경우
            self.status = Status.FAILURE
        else:
            # 객체는 있지만 새로운 객체가 추가되지 않은 경우
            self.status = Status.FAILURE
        return self.status


class CaptureScreen(Node):
    """
    현재 프레임을 이미지 파일로 저장.
    - blackboard['annotated_frame'] 또는 blackboard['current_frame']에서 프레임 가져오기
    - 저장 경로: blackboard['save_path'] 또는 기본값 'scenarios/test/captured_images/'
    - 타임스탬프를 포함한 파일명으로 저장
    """
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.type = "Action"
        
        # 저장 디렉토리 기본값
        self.default_save_dir = os.path.join(
            os.path.dirname(__file__), 'captured_images'
        )
    
    async def run(self, agent, blackboard):
        # 저장 디렉토리 설정
        save_dir = blackboard.get('save_path', self.default_save_dir)
        
        # 디렉토리가 없으면 생성
        os.makedirs(save_dir, exist_ok=True)
        
        # 프레임 가져오기 (annotated_frame 우선, 없으면 current_frame)
        frame = blackboard.get('annotated_frame')
        if frame is None:
            frame = blackboard.get('current_frame')
        
        if frame is None:
            self.ros.node.get_logger().warn('No frame available in blackboard for capture')
            self.status = Status.FAILURE
            return self.status
        
        try:
            # 타임스탬프로 파일명 생성
            from datetime import datetime
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # 밀리초 포함
            
            # 사람 감지 정보가 있으면 파일명에 포함
            person_info = blackboard.get('detected_person')
            if person_info:
                confidence = person_info.get('confidence', 0.0)
                filename = os.path.join(save_dir, f"person_{confidence:.2f}_{timestamp}.jpg")
            else:
                filename = os.path.join(save_dir, f"frame_{timestamp}.jpg")
            
            # 이미지 파일로 저장
            cv2.imwrite(filename, frame)
            
            # 저장된 파일 경로를 blackboard에 기록
            blackboard['last_captured_image'] = filename
            blackboard['capture_result'] = 'succeeded'
            
            self.ros.node.get_logger().info(f'Image saved: {filename}')
            self.status = Status.SUCCESS
            return self.status
            
        except Exception as e:
            self.ros.node.get_logger().error(f'Failed to save image: {e}', exc_info=True)
            self.status = Status.FAILURE
            return self.status


