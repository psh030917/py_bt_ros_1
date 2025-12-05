# TurtleBot3 Webots 카메라 설정 가이드

## 목표
Webots 시뮬레이션의 TurtleBot3Burger에 카메라를 추가하여 ROS 2 토픽으로 이미지를 발행하기

## 문제 상황
- 초기에는 TurtleBot3Burger 모델에 카메라가 없었음
- `ros2 topic list`에서 카메라 관련 토픽이 보이지 않음
- CaptureImage 노드가 더미 모드로 동작

## 해결 과정

### 1. Webots World 파일 직접 수정

**파일 경로:**
```
~/webots_ros2_ws/src/webots_ros2/webots_ros2_turtlebot/worlds/turtlebot3_burger_example.wbt
```

**주의사항:**
- ⚠️ Webots GUI에서 카메라 추가 시도 → 크래시 발생
- ✅ 텍스트 에디터로 `.wbt` 파일을 직접 수정하는 것이 안전

### 2. 카메라 노드 추가

TurtleBot3Burger의 `extensionSlot` 섹션에 다음 코드 추가:

```proto
Camera {
  translation 0.05 0 0.1
  rotation 0 1 0 0
  children [
    Transform {
      translation 0 0 0
      rotation 0 1 0 0
      children [
        Shape {
          appearance PBRAppearance {
            baseColor 0.1 0.1 0.1
            roughness 1
            metalness 0
          }
          geometry Box {
            size 0.02 0.03 0.02
          }
        }
      ]
    }
  ]
  name "front_camera"
  fieldOfView 1.0472
  width 640
  height 480
}
```

**카메라 위치:**
- `translation 0.05 0 0.1`: 로봇 앞쪽(x=0.05), 위쪽(z=0.1)
- `name "front_camera"`: ROS 토픽 이름에 사용됨

### 3. 패키지 빌드

카메라 추가 후 반드시 패키지를 다시 빌드해야 합니다:

```bash
cd ~/webots_ros2_ws
colcon build --packages-select webots_ros2_turtlebot
source install/setup.bash
```

### 4. 카메라 토픽 확인

Webots + ROS 2 실행 후 토픽 확인:

```bash
ros2 topic list | grep camera
```

**예상 출력:**
```
/TurtleBot3Burger/front_camera/image_color
/TurtleBot3Burger/front_camera/camera_info
```

**토픽 상세 정보:**
```bash
ros2 topic info /TurtleBot3Burger/front_camera/image_color
```

출력:
```
Type: sensor_msgs/msg/Image
Publisher count: 1
Subscription count: X
```

### 5. 이미지 데이터 확인

**실시간 이미지 보기 (rqt_image_view):**
```bash
ros2 run rqt_image_view rqt_image_view
```
토픽 선택: `/TurtleBot3Burger/front_camera/image_color`

**토픽 메시지 확인:**
```bash
ros2 topic echo /TurtleBot3Burger/front_camera/image_color --once
```

## py_bt_ros에서 카메라 사용

### bt_nodes.py 설정

```python
class CaptureImage(Node):
    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self._latest_image = None
        
        # 카메라 토픽 구독
        camera_topic = "/TurtleBot3Burger/front_camera/image_color"
        self.ros.node.create_subscription(
            Image,
            camera_topic,
            self._image_callback,
            10
        )
        self.type = "Action"
        self._dummy_mode = True  # 이미지 수신 시 자동으로 False
    
    def _image_callback(self, msg):
        """최신 이미지 저장"""
        self._latest_image = msg
        self._dummy_mode = False  # 실제 이미지 받으면 더미 모드 해제
```

### 동작 확인

py_bt_ros 실행 시 로그 확인:
- 더미 모드: `"CaptureImage: Running in dummy mode (no camera topic)"`
- 정상 모드: `"Image captured successfully!"`

## 트러블슈팅

### 카메라 토픽이 안 보일 때

1. **Webots 재시작**
   ```bash
   # Webots 종료 후 다시 실행
   ros2 launch webots_ros2_turtlebot <your_launch_file>
   ```

2. **빌드 확인**
   ```bash
   cd ~/webots_ros2_ws
   colcon build --packages-select webots_ros2_turtlebot
   source install/setup.bash
   ```

3. **.wbt 파일 문법 확인**
   - Webots를 실행하여 world 파일 로드 테스트
   - 에러 메시지 확인

### Webots 크래시 발생 시

- GUI를 통한 카메라 추가는 불안정함
- 반드시 텍스트 에디터로 `.wbt` 파일 직접 수정

## 요약

✅ **성공적인 카메라 추가 절차:**
1. `.wbt` 파일을 텍스트 에디터로 열기
2. Camera 노드를 extensionSlot에 추가
3. `colcon build` 실행
4. `source install/setup.bash`
5. Webots 재시작
6. `ros2 topic list`로 카메라 토픽 확인
7. py_bt_ros에서 토픽 구독

**결과:** `/TurtleBot3Burger/front_camera/image_color` 토픽이 정상적으로 발행되며, CaptureImage 노드가 이미지를 캡처할 수 있음! 📸
