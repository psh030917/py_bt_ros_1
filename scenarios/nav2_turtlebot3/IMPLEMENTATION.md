# Nav2 + TurtleBot3 + py_bt_ros 구현 상세 가이드

## 📋 목차
1. [개요](#개요)
2. [시스템 구조](#시스템-구조)
3. [실행 방법](#실행-방법)
4. [구현 세부사항](#구현-세부사항)
5. [Behavior Tree 설계](#behavior-tree-설계)
6. [트러블슈팅](#트러블슈팅)

---

## 개요

### 🎯 프로젝트 목적
TurtleBot3 로봇이 Webots 시뮬레이션 환경에서 자율적으로 3단계 미션을 수행하는 시스템 구현

### 🚀 미션 시나리오
```
1단계: SaveInitialPose  - 초기 위치 저장
2단계: MoveToGoal       - 목표 지점으로 이동
3단계: CaptureImage     - 목표 지점에서 이미지 캡처
4단계: Return           - 초기 위치로 복귀
5단계: 대기             - 새로운 목표 수신 대기 (무한 반복)
```

### ✨ 주요 기능
- ✅ Nav2 기반 자율 주행
- ✅ 목표 지점 자동 네비게이션
- ✅ 카메라 이미지 캡처 및 파일 저장
- ✅ 초기 위치 자동 복귀
- ✅ 무한 반복 미션 수행 (새 목표 대기)

---

## 시스템 구조

### 기술 스택
| 구성 요소 | 기술/버전 |
|---------|---------|
| **OS** | Ubuntu 22.04 |
| **ROS** | ROS 2 Humble |
| **Simulator** | Webots R2023b |
| **Navigation** | Nav2 |
| **BT Framework** | py_bt_ros |
| **Robot** | TurtleBot3 Burger |
| **Python** | 3.10 |

### 아키텍처 다이어그램
```
┌─────────────────┐
│   Webots        │
│   Simulator     │
└────────┬────────┘
         │
         ▼
┌─────────────────┐      ┌──────────────┐
│   Nav2 Stack    │◄────►│  py_bt_ros   │
│   (Navigation)  │      │  (BT Engine) │
└────────┬────────┘      └──────┬───────┘
         │                      │
         ▼                      ▼
┌─────────────────────────────────┐
│     ROS 2 Topics/Actions        │
│  /goal_pose, /navigate_to_pose  │
│  /amcl_pose, /camera/image      │
└─────────────────────────────────┘
```

### 파일 구조
```
scenarios/nav2_turtlebot3/
├── bt_nodes.py              # BT 노드 구현 (핵심 로직)
├── nav2_bt.xml              # BT 구조 정의
├── README.md                # 실행 가이드
├── CAMERA_SETUP.md          # 카메라 설정 가이드
├── IMPLEMENTATION.md        # 구현 상세 (이 파일)
├── bt_nodes_backup.py       # 백업 파일
└── captured_images/         # 캡처된 이미지 저장 폴더
    ├── image_20251112_143052.jpg
    └── ...
```

---

## 실행 방법

### 1️⃣ 사전 준비

**필수 패키지 설치:**
```bash
# NumPy 버전 확인 및 다운그레이드 (중요!)
pip install "numpy<2"

# cv_bridge는 ROS 2 Humble과 함께 설치됨
# opencv-python은 이미 설치되어 있어야 함
```

### 2️⃣ Webots + Nav2 실행

**터미널 1:**
```bash
cd ~/webots_ros2_ws
source install/setup.bash
ros2 launch webots_ros2_turtlebot robot_launch.py
```

**확인사항:**
- ✅ Webots 시뮬레이터 창 열림
- ✅ Rviz2 실행
- ✅ TurtleBot3 로봇 표시
- ✅ 맵 로드 완료

### 3️⃣ py_bt_ros 실행

**터미널 2:**
```bash
cd ~/py_bt_ros
python3 main.py
```

**예상 로그:**
```
[INFO] Initial pose saved: (0.43, 1.89)
[INFO] ⏸️  Waiting for new goal pose...
```

### 4️⃣ 목표 지점 설정

**터미널 3:**
```bash
# 예시 1: 원점으로 이동
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"

# 예시 2: (3, 2) 위치로 이동
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}"
```

### 5️⃣ 실행 흐름 확인

**py_bt_ros 로그:**
```
[INFO] 🎯 Goal received: (3.00, 2.00)
[INFO] 📤 Building Nav2 goal...
[INFO] ✅ Navigation succeeded!
[INFO] 📸 Image saved: /home/.../captured_images/image_20251112_143052.jpg
[INFO] ✅ Return succeeded! Resetting BT for next mission...
[INFO] ⏸️  Waiting for new goal pose...
```

**동작 확인:**
1. 로봇이 목표 지점으로 이동
2. 도착 후 이미지 캡처 (파일 저장)
3. 초기 위치로 복귀
4. 제자리에서 새 목표 대기 ⏸️

---

## 구현 세부사항

### 🏗️ BT 노드 구현

#### 1. SaveInitialPose (Condition Node)
**목적:** 로봇의 초기 위치를 저장

**구현 로직:**
```python
class SaveInitialPose(Node):
    def __init__(self, name, agent):
        self.ros = agent.ros_bridge
        self._current_pose = None
        self._timeout = 5.0  # 5초 타임아웃
        
        # /amcl_pose 토픽 구독
        self.ros.node.create_subscription(
            PoseStamped, "/amcl_pose", 
            self._pose_callback, 10
        )
```

**주요 기능:**
- `/amcl_pose` 또는 `/odom` 토픽에서 현재 위치 수신
- 타임아웃 메커니즘 (5초 대기 후 하드코딩 위치 사용)
- Blackboard에 `initial_pose` 저장

**타임아웃 처리:**
```python
if self._current_pose is None:
    elapsed = time.time() - self._start_time
    if elapsed > self._timeout:
        # 하드코딩된 기본 위치 사용
        initial_pose = PoseStamped()
        initial_pose.pose.position.x = 6.36
        initial_pose.pose.position.y = 0.0
```

#### 2. MoveToGoal (Action Node with ROS Action)
**목적:** 목표 지점으로 네비게이션

**핵심 메커니즘:**
```python
class MoveToGoal(ActionWithROSAction):
    def __init__(self, name, agent):
        super().__init__(name, agent, 
            (NavigateToPose, "/navigate_to_pose")
        )
        self._goal_pose = None
        self._goal_used = False  # 🔑 핵심: goal 재사용 방지
```

**Goal 관리 시스템:**
1. **목표 수신:** `/goal_pose` 토픽 구독
2. **목표 플래그:** `_goal_used` 로 사용 여부 추적
3. **재사용 방지:** 사용된 goal은 다시 실행 안 함

**목표 대기 로직:**
```python
def _build_goal(self, agent, bb):
    # 이미 사용된 goal이면 대기
    if self._goal_used:
        self.ros.node.get_logger().info("⏸️  Waiting for new goal pose...")
        return None  # None 반환 → RUNNING 상태 유지
    
    # 새로운 goal이면 실행
    if self._goal_pose is None:
        return None
    
    bb['goal_pose'] = self._goal_pose
    goal = NavigateToPose.Goal()
    goal.pose = self._goal_pose
    return goal
```

**성공 처리:**
```python
def _interpret_result(self, result, agent, bb, status_code=None):
    if status_code == GoalStatus.STATUS_SUCCEEDED:
        self._goal_used = True  # 🔑 goal 사용 완료 표시
        return Status.SUCCESS
```

**새 목표 수신 시:**
```python
def _goal_callback(self, msg):
    self._goal_pose = msg
    self._goal_used = False  # 🔑 플래그 리셋
```

#### 3. CaptureImage (Action Node)
**목적:** 카메라 이미지 캡처 및 파일 저장

**이미지 처리 파이프라인:**
```python
class CaptureImage(Node):
    def __init__(self, name, agent):
        self._bridge = CvBridge()
        self._save_dir = os.path.join(
            os.path.dirname(__file__), 
            "captured_images"
        )
        os.makedirs(self._save_dir, exist_ok=True)
```

**캡처 및 저장:**
```python
async def run(self, agent, blackboard):
    # ROS Image → OpenCV 변환
    cv_image = self._bridge.imgmsg_to_cv2(
        self._latest_image, "bgr8"
    )
    
    # 타임스탬프 파일명
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(
        self._save_dir, 
        f"image_{timestamp}.jpg"
    )
    
    # 파일 저장
    cv2.imwrite(filename, cv_image)
    
    # Blackboard 저장
    blackboard['captured_image'] = self._latest_image
    blackboard['image_path'] = filename
```

**더미 모드:**
- 카메라 토픽 없어도 동작 (개발/테스트용)
- 실제 카메라 데이터 수신 시 자동 전환

#### 4. Return (Action Node with ROS Action)
**목적:** 초기 위치로 복귀 + BT 리셋

**복귀 로직:**
```python
class Return(ActionWithROSAction):
    def _build_goal(self, agent, bb):
        initial_pose = bb.get('initial_pose')
        if initial_pose is None:
            return None
        
        goal = NavigateToPose.Goal()
        goal.pose = initial_pose
        return goal
```

**🔑 핵심: BT 리셋 메커니즘:**
```python
def _interpret_result(self, result, agent, bb, status_code=None):
    if status_code == GoalStatus.STATUS_SUCCEEDED:
        # Blackboard 초기화
        bb.pop('initial_pose', None)
        bb.pop('goal_pose', None)
        bb.pop('nav_result', None)
        bb.pop('capture_result', None)
        
        # 🔑 FAILURE 반환하여 Sequence 리셋
        return Status.FAILURE  # ← 이게 핵심!
```

**왜 FAILURE를 반환?**
- Sequence는 모든 자식이 SUCCESS면 다시 반복
- Return이 SUCCESS를 반환하면 → Sequence SUCCESS → 즉시 다시 시작
- Return이 FAILURE를 반환하면 → Sequence FAILURE → 처음부터 재시작
- 하지만 MoveToGoal이 `_goal_used=True` 상태라서 대기 모드로 진입!

---

## Behavior Tree 설계

### BT 구조 (XML)
```xml
<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4">
  <BehaviorTree ID="space-simulator">
    <Sequence>
      <SaveInitialPose/>
      <MoveToGoal/>
      <CaptureImage/>
      <Return/>
    </Sequence>
  </BehaviorTree>
</root>
```

### 실행 흐름 다이어그램
```
┌─────────────────────────────────────────────────┐
│              Sequence (루트)                     │
└─────────────────────────────────────────────────┘
         │
         ├─► [1] SaveInitialPose
         │    └─► SUCCESS → initial_pose 저장
         │
         ├─► [2] MoveToGoal
         │    ├─► goal_used = False? → Nav2 실행
         │    ├─► SUCCESS → goal_used = True
         │    └─► goal_used = True? → RUNNING (대기)
         │
         ├─► [3] CaptureImage
         │    └─► SUCCESS → 이미지 파일 저장
         │
         └─► [4] Return
              ├─► SUCCESS (초기 위치 도착)
              ├─► Blackboard 초기화
              └─► FAILURE 반환 → Sequence 리셋
                   │
                   └─► 다시 [1]로 이동
                        └─► MoveToGoal은 goal_used=True이므로 대기
```

### 상태 전이표

| 단계 | 노드 | 입력 | 동작 | 출력 | Blackboard 변화 |
|-----|------|------|------|------|----------------|
| 1 | SaveInitialPose | /amcl_pose | 초기 위치 저장 | SUCCESS | initial_pose 저장 |
| 2 | MoveToGoal | /goal_pose | Nav2 네비게이션 | SUCCESS | goal_used=True |
| 3 | CaptureImage | /camera/image | 이미지 캡처 및 저장 | SUCCESS | image_path 저장 |
| 4 | Return | initial_pose | 초기 위치로 복귀 | FAILURE | 모든 데이터 초기화 |
| 5 | SaveInitialPose | - | 재실행 | SUCCESS | initial_pose 재저장 |
| 6 | MoveToGoal | goal_used=True | 대기 모드 | RUNNING | - |
| 7 | - | 새 /goal_pose | goal_used=False | - | 2단계로 복귀 |

### 무한 반복 메커니즘

**핵심 아이디어:**
1. **Return이 FAILURE 반환** → Sequence가 실패하여 처음부터 재실행
2. **MoveToGoal의 goal_used 플래그** → 사용된 goal은 재실행 안 함
3. **결과:** Return 후 MoveToGoal에서 대기 → 새 goal 오면 다시 시작

**코드 흐름:**
```python
# Return 성공 → FAILURE 반환
Return: SUCCESS (물리적) → FAILURE (BT 상태)

# Sequence 리셋
Sequence: FAILURE → 처음부터 재실행

# SaveInitialPose 재실행
SaveInitialPose: SUCCESS

# MoveToGoal 체크
if _goal_used == True:
    return None  # RUNNING 상태 유지
    # 🛑 여기서 멈춤! 새 goal 대기

# 새 goal 도착
_goal_callback():
    _goal_used = False  # 플래그 리셋

# 다시 네비게이션 시작
MoveToGoal: 실행!
```

---

## ROS 2 통신

### 토픽 (Topics)

| 토픽 이름 | 메시지 타입 | 방향 | 용도 |
|----------|-----------|------|------|
| `/goal_pose` | geometry_msgs/PoseStamped | Subscribe | 목표 위치 수신 |
| `/amcl_pose` | geometry_msgs/PoseStamped | Subscribe | 현재 위치 (AMCL) |
| `/odom` | nav_msgs/Odometry | Subscribe | 오도메트리 |
| `/TurtleBot3Burger/front_camera/image_color` | sensor_msgs/Image | Subscribe | 카메라 이미지 |

### 액션 (Actions)

| 액션 이름 | 액션 타입 | 용도 |
|----------|----------|------|
| `/navigate_to_pose` | nav2_msgs/NavigateToPose | Nav2 네비게이션 실행 |

### 메시지 구조 예시

**목표 위치 발행:**
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
'{
  header: {
    frame_id: "map"
  },
  pose: {
    position: {x: 3.0, y: 2.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}'
```

---

## 트러블슈팅

### 문제 1: NumPy 버전 충돌
**증상:**
```
AttributeError: _ARRAY_API not found
ImportError: numpy.core.multiarray failed to import
```

**원인:** NumPy 2.x와 cv_bridge 호환 문제

**해결:**
```bash
pip install "numpy<2"
```

### 문제 2: 카메라 토픽 없음
**증상:** `CaptureImage: Running in dummy mode`

**해결:** [CAMERA_SETUP.md](./CAMERA_SETUP.md) 참고
1. Webots world 파일에 카메라 추가
2. `colcon build --packages-select webots_ros2_turtlebot`
3. `source install/setup.bash`

### 문제 3: 목표 도달 실패 (status_code=6)
**증상:** `❌ Navigation aborted/failed: 6`

**원인:** 
- 목표 위치가 맵 밖
- 장애물 위치
- Nav2 파라미터 문제

**해결:**
1. Rviz에서 목표 위치 확인
2. 더 가까운 목표로 변경
3. Nav2 로그 확인

### 문제 4: 초기 위치 저장 안 됨
**증상:** SaveInitialPose가 계속 RUNNING

**원인:** `/amcl_pose` 토픽 발행 안 됨

**해결:**
```bash
# 토픽 확인
ros2 topic echo /amcl_pose --once
ros2 topic echo /odom --once

# Nav2가 제대로 실행되었는지 확인
ros2 node list | grep nav2
```

### 문제 5: 두 번째 목표부터 동작 안 함
**증상:** 첫 번째 미션 후 멈춤

**원인:** MoveToGoal의 goal_used 플래그 문제

**확인:** 
- `_goal_callback`에서 `_goal_used = False` 리셋 확인
- 로그에서 "⏸️ Waiting for new goal pose..." 확인

---

## 성능 및 제한사항

### 성능 지표
- **네비게이션 속도:** Nav2 기본 설정 사용
- **이미지 캡처 시간:** ~0.1초
- **BT Tick Rate:** 10Hz (기본값)

### 알려진 제한사항
1. **시뮬레이션 환경 한정:** 실제 로봇에서는 추가 설정 필요
2. **단일 로봇:** 다중 로봇 시나리오 미지원
3. **이미지 저장:** 로컬 파일 시스템만 지원 (클라우드 X)
4. **장애물 회피:** Nav2 기본 설정 의존

### 향후 개선 사항
- [ ] 클라우드 이미지 업로드 (AWS S3, Google Cloud)
- [ ] 다중 로봇 지원
- [ ] 동적 목표 우선순위
- [ ] 배터리 상태 모니터링
- [ ] 실패 시 재시도 로직

---

## 참고 자료

### 공식 문서
- [py_bt_ros GitHub](https://github.com/inmo-jang/py_bt_ros)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Webots ROS 2](https://github.com/cyberbotics/webots_ros2)
- [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)

### 관련 파일
- [README.md](./README.md) - 실행 가이드
- [CAMERA_SETUP.md](./CAMERA_SETUP.md) - 카메라 설정
- [bt_nodes.py](./bt_nodes.py) - 소스 코드

---

## 라이센스 및 기여

**작성일:** 2025-11-12  
**py_bt_ros 버전:** main branch  
**ROS 2 버전:** Humble  
**작성자:** Based on py_bt_ros framework

**기여 방법:**
1. 이슈 리포트: [GitHub Issues](https://github.com/inmo-jang/py_bt_ros/issues)
2. 개선 제안 환영!

---

**🎉 프로젝트 완성을 축하합니다! 🚀**
