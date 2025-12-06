# 화재 감지 Behavior Tree 노드 문서

## 개요

이 문서는 화재 감지 시스템을 위한 Behavior Tree 노드들에 대한 설명입니다. `limo_fire_py` 패키지를 참고하여 구현되었습니다.

## 노드 목록

1. **UpdateFireState** (Condition): 화재 센서 데이터를 읽어 blackboard에 상태를 업데이트
2. **StopRobot** (Action): 로봇을 정지시키는 명령 발행

---

## UpdateFireState (Condition)

### 설명

화재 센서 데이터를 구독하여 blackboard에 화재 상태를 업데이트하는 Condition 노드입니다. 센서 데이터가 있으면 SUCCESS, 없으면 FAILURE를 반환합니다.

### 토픽

- **구독**: `/fire_sensor` (std_msgs/String)
- **메시지 형식**: JSON
  ```json
  {
    "flame": 1,
    "temp": 600
  }
  ```

### 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `fire_sensor_topic` | string | `"/fire_sensor"` | 화재 센서 토픽 이름 |
| `flame_threshold` | int | `1` | 불꽃 감지 임계값 (flame >= 이 값이면 화재) |
| `temp_threshold` | float | `600` | 온도 감지 임계값 (temp < 이 값이면 화재) |

### 화재 감지 로직

```python
fire_detected = (flame >= flame_threshold) or (temp < temp_threshold)
```

두 조건 중 하나라도 만족하면 화재로 판단:
- **불꽃 감지**: `flame >= flame_threshold` (기본값: 1)
- **온도 감지**: `temp < temp_threshold` (기본값: 600)

### Blackboard 업데이트

노드가 실행되면 다음 값들이 blackboard에 저장됩니다:

- `fire_detected` (bool): 화재 감지 여부
- `fire_level` (int): flame 값 (0 또는 1)
- `temp` (float): 온도 값

### 동적 파라미터 변경

런타임에 blackboard를 통해 파라미터를 변경할 수 있습니다:

```python
blackboard['fire_flame_threshold'] = 2  # flame_threshold 변경
blackboard['fire_temp_threshold'] = 500  # temp_threshold 변경
```

### XML 사용 예시

```xml
<!-- 기본값 사용 -->
<UpdateFireState name="update_fire_state"/>

<!-- 파라미터 지정 -->
<UpdateFireState name="update_fire_state" 
                 fire_sensor_topic="/fire_sensor"
                 flame_threshold="1" 
                 temp_threshold="600"/>
```

### 센서 동작 원리

`fire_sensor_publisher.py`에서 센서 데이터를 처리하는 방식:

1. 아두이노에서 `flame_raw, temp_raw` 값을 읽음
2. `flame_raw < 600`이면 `flame_detected = 1`, 아니면 `0`
3. JSON 형식으로 `/fire_sensor` 토픽에 발행

**참고**: 센서 특성상 값이 낮을수록 화재를 의미합니다.

---

## StopRobot (Action)

### 설명

로봇을 정지시키는 Action 노드입니다. `/cmd_vel` 토픽에 `Twist` 메시지를 발행하여 로봇의 선속도와 각속도를 0으로 설정합니다.

### 토픽

- **발행**: `/cmd_vel` (geometry_msgs/Twist)
- **메시지 내용**:
  ```python
  twist.linear.x = 0.0
  twist.angular.z = 0.0
  ```

### 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `cmd_vel_topic` | string | `"/cmd_vel"` | 로봇 속도 제어 토픽 이름 |

### 동작

1. `Twist` 메시지 생성
2. `linear.x = 0.0`, `angular.z = 0.0` 설정
3. `/cmd_vel` 토픽에 발행
4. SUCCESS 반환

### XML 사용 예시

```xml
<!-- 기본값 사용 -->
<StopRobot name="stop_robot"/>

<!-- 파라미터 지정 -->
<StopRobot name="stop_robot" cmd_vel_topic="/cmd_vel"/>
```

---

## Behavior Tree 예시

### 화재 감지 및 대응 트리

```xml
<ReactiveSequence name="MonitorAndReact">
  <!-- 1. 센서 상태 업데이트 -->
  <UpdateFireState name="update_fire_state"
                   fire_sensor_topic="/fire_sensor"
                   flame_threshold="1"
                   temp_threshold="600"/>
  
  <!-- 2. 화재 감지 여부 체크 (별도 Condition 노드 필요) -->
  <!-- <IsFireDetected name="check_fire"/> -->
  
  <!-- 3. 화재 대응 절차 실행 -->
  <Sequence name="HandleFire">
    <StopRobot name="stop_robot"/>
    <!-- <PlayLocalAlarm name="play_alarm"/> -->
    <!-- <SendTelegramAlert name="telegram_alert"/> -->
  </Sequence>
</ReactiveSequence>
```

---

## 참고 파일

- **구현 파일**: `scenarios/test/bt_nodes.py`
- **참고 패키지**: `MinGeun-SMG/limo_fire_py-main/limo_fire_py/`
  - `fire_sensor_publisher.py`: 센서 데이터 발행
  - `fire_safety_controller.py`: 화재 감지 로직 참고
  - `fire_alarm_notifier.py`: 알림 기능 참고

---

## 주의사항

1. **센서 토픽**: `/fire_sensor` 토픽이 발행되고 있어야 `UpdateFireState`가 정상 동작합니다.
2. **화재 감지 기준**: 센서 특성상 값이 낮을수록 화재를 의미하므로, `temp < temp_threshold` 조건을 사용합니다.
3. **노드 타입**: `UpdateFireState`는 Condition 노드이므로, `TreeNodesModel`에서 `<Condition>`으로 정의해야 합니다.
4. **구독 시작 시점**: `main.py` 실행 시 노드 객체가 생성되면서 센서 토픽 구독이 시작됩니다.

---

## 작성 정보

- **작성일**: 2025-01-XX
- **작성자**: MinGeun-SMG (limo_fire_py 패키지 참고)
- **구현 위치**: `psh030917/py_bt_ros_1/scenarios/test/bt_nodes.py`

