# 텔레그램 브리지 문제 해결 가이드

## 로봇 환경에서 텔레그램 패키지가 작동하지 않는 원인

### 1. Python 패키지 의존성 문제 (가장 가능성 높음)

로봇에 `python-telegram-bot` 패키지가 설치되지 않았을 수 있습니다.

**해결 방법:**
```bash
# 로봇에 SSH 접속 후
pip3 install python-telegram-bot
# 또는
pip3 install --user python-telegram-bot
```

### 2. 네트워크 연결 문제

로봇이 인터넷에 연결되어 있지 않거나, 텔레그램 API 서버에 접근할 수 없을 수 있습니다.

**확인 방법:**
```bash
ping -c 3 api.telegram.org
curl -I https://api.telegram.org
```

**해결 방법:**
- 로봇의 네트워크 설정 확인
- 방화벽 설정 확인
- 프록시 설정이 필요한 경우 환경 변수 설정:
  ```bash
  export HTTP_PROXY=http://proxy.example.com:8080
  export HTTPS_PROXY=http://proxy.example.com:8080
  ```

### 3. 텔레그램 브리지 코드 버그

현재 코드의 `start()` 메서드가 실제로 텔레그램 애플리케이션을 시작하지 않습니다.

**문제 코드:**
- `start()` 메서드가 실제로 `self._telegram_app.run_polling()`을 호출하지 않음
- `stop()` 메서드에서 존재하지 않는 `self._telegram_updater` 참조

### 4. ROS2 환경 설정 문제

로봇에서 ROS2 환경이 제대로 설정되지 않았을 수 있습니다.

**확인 방법:**
```bash
source /opt/ros/humble/setup.bash
source ~/py_bt_ros/install/setup.bash
ros2 pkg list | grep telegram
```

## 진단 스크립트 실행

```bash
cd /home/wego/py_bt_ros
./check_telegram_env.sh
```

이 스크립트는 다음을 확인합니다:
1. ROS2 환경 설정
2. 텔레그램 패키지 설치 여부
3. Python 패키지 의존성
4. 네트워크 연결
5. 설정 파일
6. Python 버전
7. 모듈 임포트 테스트

## 빠른 해결 방법

1. **Python 패키지 설치:**
   ```bash
   pip3 install python-telegram-bot
   ```

2. **네트워크 확인:**
   ```bash
   ping api.telegram.org
   ```

3. **텔레그램 브리지 실행 및 에러 확인:**
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/py_bt_ros/install/setup.bash
   ros2 launch telegram_ros2 telegram_bridge.launch.py
   ```

4. **에러 로그 확인:**
   - `ModuleNotFoundError: No module named 'telegram'` → Python 패키지 미설치
   - `ConnectionError` 또는 `TimeoutError` → 네트워크 문제
   - `Invalid token` → API 토큰 문제

