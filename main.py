import asyncio
import argparse
import cProfile
import subprocess
import signal
import sys
import os

from modules.utils import set_config

# 텔레그램 브리지 프로세스 관리
telegram_process = None

def start_telegram_bridge():
    """텔레그램 브리지를 백그라운드로 시작"""
    global telegram_process
    try:
        # telegram_ros2 패키지가 설치되어 있는지 확인
        result = subprocess.run(
            ['ros2', 'pkg', 'list'],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        if 'telegram_ros2' not in result.stdout:
            print("[WARNING] telegram_ros2 패키지를 찾을 수 없습니다. 텔레그램 브리지를 시작하지 않습니다.")
            return None
        
        print("[INFO] 텔레그램 브리지를 시작합니다...")
        
        # ros2 launch를 백그라운드로 실행
        telegram_process = subprocess.Popen(
            ['ros2', 'launch', 'telegram_ros2', 'telegram_bridge.launch.py'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid  # 새 프로세스 그룹 생성 (종료 시 모든 자식 프로세스 함께 종료)
        )
        
        print(f"[INFO] 텔레그램 브리지 시작됨 (PID: {telegram_process.pid})")
        return telegram_process
        
    except FileNotFoundError:
        print("[WARNING] ros2 명령어를 찾을 수 없습니다. 텔레그램 브리지를 시작하지 않습니다.")
        return None
    except Exception as e:
        print(f"[WARNING] 텔레그램 브리지 시작 실패: {e}")
        return None

def stop_telegram_bridge():
    """텔레그램 브리지 프로세스 종료"""
    global telegram_process
    if telegram_process is not None:
        try:
            print("[INFO] 텔레그램 브리지를 종료합니다...")
            # 프로세스 그룹 전체 종료
            os.killpg(os.getpgid(telegram_process.pid), signal.SIGTERM)
            telegram_process.wait(timeout=5)
            print("[INFO] 텔레그램 브리지 종료됨")
        except ProcessLookupError:
            print("[WARNING] 텔레그램 브리지 프로세스를 찾을 수 없습니다 (이미 종료됨)")
        except subprocess.TimeoutExpired:
            print("[WARNING] 텔레그램 브리지 종료 타임아웃, 강제 종료...")
            os.killpg(os.getpgid(telegram_process.pid), signal.SIGKILL)
        except Exception as e:
            print(f"[WARNING] 텔레그램 브리지 종료 중 오류: {e}")
        finally:
            telegram_process = None

def signal_handler(sig, frame):
    """시그널 핸들러 (Ctrl+C 등)"""
    print("\n[INFO] 종료 신호 수신, 정리 중...")
    stop_telegram_bridge()
    sys.exit(0)

# Parse command line arguments
parser = argparse.ArgumentParser(description='py_bt_ros')
parser.add_argument('--config', type=str, default='config.yaml', help='Path to the configuration file (default: --config=config.yaml)')
parser.add_argument('--no-telegram', action='store_true', help='텔레그램 브리지를 시작하지 않음')
args = parser.parse_args()

# Load configuration and initialize the environment
set_config(args.config)
from modules.utils import config
from modules.bt_runner import BTRunner

# 시그널 핸들러 등록
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# 텔레그램 브리지 시작 (옵션이 없으면)
if not args.no_telegram:
    telegram_enabled = config.get('telegram', {}).get('enabled', True)
    if telegram_enabled:
        start_telegram_bridge()

bt_runner = BTRunner(config)

async def loop():
    try:
        while bt_runner.running:
            bt_runner.handle_keyboard_events()
            if not bt_runner.paused:
                await bt_runner.step()
            bt_runner.render()
    finally:
        bt_runner.close()
        stop_telegram_bridge()

if __name__ == "__main__":
    try:
        if config['bt_runner']['profiling_mode']:
            cProfile.run('main()', sort='cumulative')
        else:
            asyncio.run(loop())
    except KeyboardInterrupt:
        print("\n[INFO] 프로그램 종료 중...")
    finally:
        stop_telegram_bridge()