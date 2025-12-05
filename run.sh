#!/bin/bash

# ROS2 워크스페이스 디렉토리
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS2 환경 설정
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

# Python 스크립트 실행
python3 "${WORKSPACE_DIR}/main.py" "$@"

