```bash
# 빌드 
## 한줄씩 실행해 주세요.
pip3 install -r requirements.txt
pip3 install -r src/yolo_ros/requirements.txt
cd ~/py_bt_ros_1
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

```bash
# 실행
## 한줄씩 실행해 주세요.
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 main.py
```

```bash
# 텔레그램 접속 방법
1. 텔레그램 앱 접속
2. 상단의 검색 버튼 클릭
3. LIMO_LIMO_LIMO_bot 검색 후 나오는 내용 클릭
```
