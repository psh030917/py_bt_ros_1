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
    'CaptureScreen',
    'SendTelegramMessage'
]

CUSTOM_CONDITION_NODES = [
    'IsDetectedSomething'
]

# BT Node List
BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)


# Helper function for IoU calculation
def _calculate_iou(box1, box2):
    """
    IoU(Intersection over Union) 계산 함수.
    두 바운딩 박스의 겹침 정도를 0.0 ~ 1.0으로 반환.
    
    Args:
        box1: [x1, y1, x2, y2] 형식의 바운딩 박스
        box2: [x1, y1, x2, y2] 형식의 바운딩 박스
    
    Returns:
        float: IoU 값 (0.0 ~ 1.0)
    """
    # 교집합 영역 계산
    x1_inter = max(box1[0], box2[0])
    y1_inter = max(box1[1], box2[1])
    x2_inter = min(box1[2], box2[2])
    y2_inter = min(box1[3], box2[3])
    
    # 교집합이 없는 경우
    if x2_inter < x1_inter or y2_inter < y1_inter:
        return 0.0
    
    # 교집합 면적
    intersection_area = (x2_inter - x1_inter) * (y2_inter - y1_inter)
    
    # 각 박스의 면적
    box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1])
    box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1])
    
    # 합집합 면적
    union_area = box1_area + box2_area - intersection_area
    
    # IoU 계산
    if union_area == 0:
        return 0.0
    
    iou = intersection_area / union_area
    return iou

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
    - YOLO 모델 경로: blackboard['yolo_model_path'] 또는 기본값 'yolov10n.pt'
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
        model_path = blackboard.get('yolo_model_path', 'yolo12n.pt')
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


# IsDetectedSomething 노드의 이전 바운딩박스 및 마지막 감지 시간 추적
_is_detected_something_previous_boxes = {}  # {node_name: [(bbox, timestamp), ...]}
_is_detected_something_last_detection = {}  # {node_name: timestamp}


class IsDetectedSomething(Node):
    """
    blackboard에서 YOLO 검출 결과를 확인하여 새로운 객체가 감지되었는지 확인.
    - 감지할 클래스: XML 파라미터 'target_class' (기본값: "person")
    - blackboard['yolo_detections']에서 지정된 클래스 검색
    - confidence threshold: XML 파라미터 'confidence_threshold' 또는 기본값 0.5
    - IoU 기반 중복 감지 방지: XML 파라미터 'iou_threshold' (기본값: 0.3)
    - 시간 기반 쿨다운: XML 파라미터 'cooldown_time' (기본값: 10.0초)
    - 이전 프레임의 바운딩박스와 IoU 비교 + 쿨다운 시간 체크
    """
    def __init__(self, name, agent, target_class="person", confidence_threshold=0.5, 
                 cooldown_time=10.0, iou_threshold=0.3):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.target_class = target_class.lower()  # 소문자로 변환하여 비교
        self.default_confidence_threshold = confidence_threshold
        self.cooldown_time = float(cooldown_time)  # 쿨다운 시간 (초)
        self.iou_threshold = float(iou_threshold)  # IoU 임계값
        self.type = "Condition"
        agent.ros_bridge.node.get_logger().info(
            f'IsDetectedSomething: 감지 대상 = "{self.target_class}", '
            f'confidence = {confidence_threshold}, '
            f'cooldown = {cooldown_time}초, '
            f'iou_threshold = {iou_threshold}'
        )
    
    async def run(self, agent, blackboard):
        import time
        global _is_detected_something_previous_boxes
        global _is_detected_something_last_detection
        
        # 검출 결과 가져오기
        detections = blackboard.get('yolo_detections', [])
        
        # confidence threshold 가져오기 (blackboard 우선, 없으면 기본값)
        threshold_key = f'{self.target_class}_confidence_threshold'
        threshold = blackboard.get(threshold_key, self.default_confidence_threshold)
        
        # 현재 프레임의 객체 필터링 (confidence threshold 이상만)
        detected_objects = []
        for detection in detections:
            class_name = detection.get('class', '').lower()
            if class_name == self.target_class:
                confidence = detection.get('confidence', 0.0)
                if confidence >= threshold:
                    detected_objects.append(detection)
        
        # 감지된 객체가 없으면 FAILURE
        if not detected_objects:
            self.status = Status.FAILURE
            return self.status
        
        # 현재 시간
        current_time = time.time()
        
        # 이전 바운딩박스 목록 가져오기
        if self.name not in _is_detected_something_previous_boxes:
            _is_detected_something_previous_boxes[self.name] = []
        
        previous_boxes = _is_detected_something_previous_boxes[self.name]
        
        # 각 감지된 객체에 대해 새로운 객체인지 확인
        new_objects = []
        for obj in detected_objects:
            bbox = obj.get('bbox')
            if not bbox:
                continue
            
            # 이전 바운딩박스들과 IoU 비교
            is_new = True
            for prev_bbox, prev_time in previous_boxes:
                iou = _calculate_iou(bbox, prev_bbox)
                if iou > self.iou_threshold:
                    # 같은 물체로 판단 (IoU가 임계값보다 높음)
                    is_new = False
                    self.ros.node.get_logger().debug(
                        f'{self.target_class}: 같은 물체 감지됨 (IoU: {iou:.2f})'
                    )
                    break
            
            if is_new:
                new_objects.append(obj)
        
        # 새로운 객체가 없으면 FAILURE
        if not new_objects:
            self.ros.node.get_logger().debug(
                f'{self.target_class}: 모두 기존 물체임 (IoU 기반 중복 감지)'
            )
            self.status = Status.FAILURE
            return self.status
        
        # 쿨다운 시간 체크
        last_detection_time = _is_detected_something_last_detection.get(self.name, 0)
        time_since_last = current_time - last_detection_time
        
        if time_since_last < self.cooldown_time:
            # 쿨다운 시간이 아직 안 지남
            self.ros.node.get_logger().debug(
                f'{self.target_class}: 쿨다운 대기 중 ({time_since_last:.1f}/{self.cooldown_time}초)'
            )
            self.status = Status.FAILURE
            return self.status
        
        # 새로운 객체이고 쿨다운도 지남 -> SUCCESS
        # 가장 높은 confidence인 객체 선택
        best_object = max(new_objects, key=lambda x: x.get('confidence', 0.0))
        best_bbox = best_object.get('bbox')
        
        # blackboard에 저장
        blackboard[f'detected_{self.target_class}'] = best_object
        blackboard[f'{self.target_class}_confidence'] = best_object.get('confidence', 0.0)
        
        # 이전 바운딩박스 목록 업데이트 (최근 N개만 유지)
        _is_detected_something_previous_boxes[self.name].append((best_bbox, current_time))
        
        # 오래된 바운딩박스 제거 (쿨다운 시간의 2배 이상 지난 것)
        cutoff_time = current_time - (self.cooldown_time * 2)
        _is_detected_something_previous_boxes[self.name] = [
            (bbox, timestamp) for bbox, timestamp in _is_detected_something_previous_boxes[self.name]
            if timestamp > cutoff_time
        ]
        
        # 마지막 감지 시간 업데이트
        _is_detected_something_last_detection[self.name] = current_time
        
        self.ros.node.get_logger().info(
            f'새로운 {self.target_class} 감지! (신뢰도: {best_object.get("confidence", 0.0):.2f}, '
            f'쿨다운 경과: {time_since_last:.1f}초)'
        )
        
        self.status = Status.SUCCESS
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
            
            # 감지된 객체 정보를 동적으로 가져오기
            # IsDetectedSomething에서 설정한 detected_{target_class} 키를 찾음
            detected_object = None
            object_class = None
            
            # blackboard에서 detected_* 패턴의 키를 찾음
            for key in blackboard.keys():
                if key.startswith('detected_') and key != 'detected_person':
                    detected_object = blackboard.get(key)
                    object_class = key.replace('detected_', '')
                    break
            
            # detected_person도 확인 (하위 호환성)
            if detected_object is None:
                detected_object = blackboard.get('detected_person')
                if detected_object:
                    object_class = 'person'
            
            # 파일명 생성
            if detected_object and object_class:
                confidence = detected_object.get('confidence', 0.0)
                filename = os.path.join(save_dir, f"captured_{object_class}_{confidence:.2f}_{timestamp}.jpg")
                self.ros.node.get_logger().info(
                    f'CaptureScreen: {object_class} 감지 (신뢰도: {confidence:.2f}) - 스크린샷 저장'
                )
            else:
                filename = os.path.join(save_dir, f"captured_frame_{timestamp}.jpg")
                self.ros.node.get_logger().info('CaptureScreen: 일반 프레임 저장')
            
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


# SendTelegramMessage 노드의 마지막 전송 시간 추적 (노드별로 관리)
_send_telegram_last_sent = {}  # {node_name: last_sent_time}

class SendTelegramMessage(Node):
    """
    텔레그램으로 메시지를 전송하는 노드.
    - 메시지: XML 파라미터 'message' 또는 blackboard['telegram_message']
    - 주기: XML 파라미터 'interval' (초 단위, 기본값: 0 = 매번 전송)
    - ROS 토픽: '/message_from_ros' (std_msgs/String)
    - telegram_ros2_bridge가 실행 중이어야 함
    """
    def __init__(self, name, agent, message="사람이 감지되었습니다.", interval=0.0):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.default_message = message
        self.interval = float(interval)  # 초 단위
        self.type = "Action"
        
        # 텔레그램 브리지 토픽으로 퍼블리셔 생성
        from std_msgs.msg import String
        self._publisher = self.ros.node.create_publisher(String, '/message_from_ros', 10)
        agent.ros_bridge.node.get_logger().info(
            f'SendTelegramMessage: 초기화 완료 - 기본 메시지: "{message}", 주기: {interval}초'
        )
    
    async def run(self, agent, blackboard):
        import time
        
        global _send_telegram_last_sent
        
        # 메시지 가져오기 (blackboard 우선, 없으면 기본값)
        message = blackboard.get('telegram_message', self.default_message)
        
        # 주기 가져오기 (blackboard 우선, 없으면 기본값)
        interval = blackboard.get('telegram_interval', self.interval)
        
        if not message:
            self.ros.node.get_logger().warn('SendTelegramMessage: 전송할 메시지가 없습니다')
            self.status = Status.FAILURE
            return self.status
        
        # 주기 확인
        current_time = time.time()
        last_sent_time = _send_telegram_last_sent.get(self.name, 0)
        
        if interval > 0 and (current_time - last_sent_time) < interval:
            # 아직 주기 시간이 지나지 않음
            self.ros.node.get_logger().debug(
                f'SendTelegramMessage: 주기 대기 중 ({current_time - last_sent_time:.1f}/{interval}초)'
            )
            self.status = Status.SUCCESS
            return self.status
        
        # 메시지 전송
        try:
            from std_msgs.msg import String
            msg = String()
            msg.data = str(message)
            
            self._publisher.publish(msg)
            _send_telegram_last_sent[self.name] = current_time
            
            if interval > 0:
                self.ros.node.get_logger().info(
                    f'SendTelegramMessage: 텔레그램 메시지 전송 - "{message}" (다음 전송: {interval}초 후)'
                )
            else:
                self.ros.node.get_logger().info(f'SendTelegramMessage: 텔레그램 메시지 전송 - "{message}"')
            
            self.status = Status.SUCCESS
            return self.status
            
        except Exception as e:
            self.ros.node.get_logger().error(f'SendTelegramMessage: 메시지 전송 실패 - {e}', exc_info=True)
            self.status = Status.FAILURE
            return self.status
