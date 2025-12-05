# Nav2 + TurtleBot3 + py_bt_ros 시나리오

## 개요

이 시나리오는 TurtleBot3가 Webots 시뮬레이션 환경에서 다음 3단계 미션을 수행합니다:

1. **MoveToGoal**: 목표 지점으로 이동
2. **CaptureImage**: 목표 지점에서 카메라로 이미지 캡처
3. **Return**: 초기 위치로 복귀

## 실행 순서

### 1단계: Webots + Nav2 실행

```bash
# 깃허브 주소 클론
git clone https://github.com/dongmin8350/py_bt_ros.git

# 빌드 및 환경 소싱
colcon build
source install/local_setup.bash

# Webots 워크스페이스로 이동
cd ~/webots_ros2_ws

# Webots + Nav2 실행
ros2 launch webots_ros2_turtlebot robot_launch.py nav:=true
```


**확인사항:**
- Webots 시뮬레이터 창이 열림
- Rviz2가 실행되어 맵이 보임
- TurtleBot3 로봇이 시뮬레이션에 표시됨

### 2단계: py_bt_ros 실행

**새 터미널에서:**

```bash
# py_bt_ros 디렉토리로 이동
cd ~/py_bt_ros

# py_bt_ros 실행
pip3 install pygame
python3 main.py
```


### 3단계: 목표 지점 설정

**방법 1: Rviz에서 2D Goal Pose 사용 (2D Goal Pose 추가 필요)**
```bash
panels -> tool properties -> 2D goal_pose의 Topic에서 /goal_pose 를 /bt/goal_pose로 수정

# 새로운 터미널을 열고
cd ~webots_ros2_ws
ros2 topic list

/bt/goal_pose Topic확인 가능
```


### 4단계: Behavior Tree 동작 확인 (pygamewindow 확인)


**Behavior Tree 순서:**
1. `SaveInitialPose` - 초기 위치 저장
2. `MoveToGoal` - 목표로 이동
3. `CaptureImage` - 이미지 캡처
4. `Return` - 초기 위치로 복귀

## 주요 ROS 2 토픽

### 구독 (Subscribe)
- `/goal_pose` (geometry_msgs/msg/PoseStamped) - 목표 위치 수신
- `/amcl_pose` (geometry_msgs/msg/PoseStamped) - 로봇 현재 위치
- `/odom` (nav_msgs/msg/Odometry) - 오도메트리
- `/TurtleBot3Burger/front_camera/image_color` (sensor_msgs/msg/Image) - 카메라 이미지

### 액션 (Action)
- `/navigate_to_pose` (nav2_msgs/action/NavigateToPose) - Nav2 네비게이션

## 트러블슈팅

### 문제 1: " Waiting for goal pose..." 계속 표시

**원인:** 목표 위치가 발행되지 않음

**해결:**
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

### 문제 2: " Navigation aborted/failed: 6"

**원인:** 목표 지점이 도달 불가능한 위치 (장애물 또는 맵 밖)

**해결:**
1. Rviz에서 목표 위치가 맵 안에 있는지 확인
2. 더 가까운 목표로 변경 (예: x=1.0, y=1.0)
3. Nav2 로그 확인

### 문제 3: 카메라 토픽이 없음

**확인:**
```bash
ros2 topic list | grep camera
```

**해결:** [CAMERA_SETUP.md](./CAMERA_SETUP.md) 참고

### 문제 4: 초기 위치가 저장되지 않음

**원인:** `/amcl_pose` 또는 `/odom` 토픽이 발행되지 않음

**확인:**
```bash
ros2 topic echo /amcl_pose --once
ros2 topic echo /odom --once
```

## 파일 구조

```
scenarios/nav2_turtlebot3/
├── README.md              # 실행 가이드 (이 파일)
├── CAMERA_SETUP.md        # 카메라 설정 가이드
├── bt_nodes.py            # Behavior Tree 노드 구현
├── nav2_bt.xml            # Behavior Tree 구조
└── bt_nodes_backup.py     # 백업 파일
```

## config.yaml 설정

```yaml
scenario: scenarios.nav2_turtlebot3
namespaces: ""
behavior_tree_xml: "nav2_bt.xml"
```

## Behavior Tree 구조

```xml
<Sequence>
    <SaveInitialPose/>      <!-- 초기 위치 저장 -->
    <MoveToGoal/>           <!-- 목표로 이동 -->
    <CaptureImage/>         <!-- 이미지 캡처 -->
    <Return/>               <!-- 초기 위치로 복귀 -->
</Sequence>
```

## 참고 자료

- [py_bt_ros GitHub](https://github.com/inmo-jang/py_bt_ros)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Webots ROS 2](https://github.com/cyberbotics/webots_ros2)

## 종료 방법

1. **py_bt_ros 종료**: `Ctrl + C`
2. **Webots + Nav2 종료**: `Ctrl + C`
3. **모든 ROS 2 노드 종료**: 
   ```bash
   pkill -f "ros2"
   pkill -f "python3 main.py"
   ```

---

**작성일:** 2025-11-12  
**py_bt_ros 버전:** main branch  
**ROS 2 버전:** Humble
