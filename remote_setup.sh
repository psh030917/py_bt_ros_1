#!/bin/bash
# 원격 접속 후 환경 설정을 위한 스크립트

echo "=== ROS2 환경 설정 ==="
export WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

echo "=== 현재 디렉토리: $(pwd) ==="
echo "=== ROS2 환경 설정 완료 ==="
echo ""
echo "사용 가능한 명령어:"
echo "  ./run.sh           - 행동 트리 실행"
echo "  python3 main.py    - 직접 실행 (환경 설정 필요)"
echo "  ros2 node list     - 실행 중인 노드 확인"
echo "  ros2 topic list    - 토픽 목록 확인"

# 인터랙티브 셸이면 현재 셸에 적용
if [ -t 0 ]; then
    exec bash
fi

