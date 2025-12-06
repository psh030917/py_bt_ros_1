#!/bin/bash

# 텔레그램 패키지 환경 진단 스크립트

echo "=========================================="
echo "텔레그램 패키지 환경 진단"
echo "=========================================="
echo ""

# 1. ROS2 환경 확인
echo "[1] ROS2 환경 확인"
if command -v ros2 &> /dev/null; then
    echo "✓ ROS2 명령어 사용 가능"
    ros2 --version 2>/dev/null || echo "  ROS2 버전 확인 실패"
else
    echo "✗ ROS2 명령어를 찾을 수 없습니다"
    echo "  source /opt/ros/humble/setup.bash 실행 필요"
fi
echo ""

# 2. 텔레그램 패키지 확인
echo "[2] 텔레그램 패키지 확인"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi
if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    source "${WORKSPACE_DIR}/install/setup.bash"
fi

if command -v ros2 &> /dev/null; then
    if ros2 pkg list 2>/dev/null | grep -q telegram_ros2; then
        echo "✓ telegram_ros2 패키지 설치됨"
    else
        echo "✗ telegram_ros2 패키지를 찾을 수 없습니다"
        echo "  colcon build --packages-select telegram_ros2 실행 필요"
    fi
else
    echo "? ROS2 환경이 설정되지 않아 확인 불가"
fi
echo ""

# 3. Python 패키지 의존성 확인
echo "[3] Python 패키지 의존성 확인"
python3 -c "import telegram" 2>/dev/null && echo "✓ python-telegram-bot 설치됨" || echo "✗ python-telegram-bot 미설치 (pip3 install python-telegram-bot 필요)"
python3 -c "import cv2" 2>/dev/null && echo "✓ opencv-python 설치됨" || echo "✗ opencv-python 미설치"
python3 -c "import numpy" 2>/dev/null && echo "✓ numpy 설치됨" || echo "✗ numpy 미설치"
python3 -c "from cv_bridge import CvBridge" 2>/dev/null && echo "✓ cv_bridge 설치됨" || echo "✗ cv_bridge 미설치"
python3 -c "import rclpy" 2>/dev/null && echo "✓ rclpy 설치됨" || echo "✗ rclpy 미설치"
echo ""

# 4. 네트워크 연결 확인
echo "[4] 네트워크 연결 확인"
if ping -c 1 -W 2 api.telegram.org &> /dev/null; then
    echo "✓ api.telegram.org 접근 가능"
elif ping -c 1 -W 2 8.8.8.8 &> /dev/null; then
    echo "? 인터넷 연결은 되지만 api.telegram.org 접근 불가 (방화벽 문제 가능)"
else
    echo "✗ 인터넷 연결 불가"
fi
echo ""

# 5. 설정 파일 확인
echo "[5] 설정 파일 확인"
CONFIG_FILE="${WORKSPACE_DIR}/src/telegram_ros2/config/example_param.yaml"
if [ -f "$CONFIG_FILE" ]; then
    echo "✓ 설정 파일 존재: $CONFIG_FILE"
    API_TOKEN=$(grep -E "^\s*api_token:" "$CONFIG_FILE" | sed 's/.*api_token:\s*//' | tr -d "'\"")
    if [ -n "$API_TOKEN" ] && [ "$API_TOKEN" != "" ]; then
        echo "✓ API 토큰 설정됨 (길이: ${#API_TOKEN})"
    else
        echo "✗ API 토큰이 설정되지 않았습니다"
    fi
else
    echo "✗ 설정 파일을 찾을 수 없습니다: $CONFIG_FILE"
fi
echo ""

# 6. Python 버전 확인
echo "[6] Python 버전 확인"
PYTHON_VERSION=$(python3 --version 2>&1)
echo "  $PYTHON_VERSION"
echo ""

# 7. 텔레그램 브리지 직접 실행 테스트
echo "[7] 텔레그램 브리지 모듈 임포트 테스트"
python3 -c "
import sys
try:
    sys.path.insert(0, '${WORKSPACE_DIR}/src/telegram_ros2')
    from telegram_ros2.telegram_ros2_bridge import TelegramBridge
    print('✓ 텔레그램 브리지 모듈 임포트 성공')
except ImportError as e:
    print(f'✗ 텔레그램 브리지 모듈 임포트 실패: {e}')
except Exception as e:
    print(f'? 모듈 임포트 중 오류: {e}')
" 2>&1
echo ""

echo "=========================================="
echo "진단 완료"
echo "=========================================="

