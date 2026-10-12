#!/bin/bash
# C-WANG GCS (지상 관제 노트북) 설치 — Ubuntu 24.04 + ROS 2 Jazzy (로봇과 같은 배포판).
# ⚠ 데이터 약 1.5–2 GB 를 받는다: 핫스팟 말고 일반 Wi-Fi 에서. 다시 실행해도 안전 (이미 된 단계는 건너뜀).
#   bash ~/setup_gcs.sh            (중간에 sudo 비밀번호를 물음)
# 끝나면: 새 터미널 → cw check → cw up      (자세한 건 ~/perseverance/tools/gcs/README.md)
set -e
ROBOT_IP=192.168.50.124
say() { printf '\n\033[1m== %s\033[0m\n' "$*"; }

say "1/7 기본 도구"
sudo apt-get update
sudo apt-get install -y curl git wget software-properties-common python3-venv python3-opencv sshpass
sudo add-apt-repository -y universe

say "2/7 ROS 2 Jazzy 저장소"
if [ ! -f /etc/apt/sources.list.d/ros2.sources ] && [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
  V=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | awk -F'"' '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-$VERSION_CODENAME})_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt-get update
fi

say "3/7 ROS 패키지 (데스크톱 전체가 아니라 필요한 것만 — RAM 3 GB)"
sudo apt-get install -y ros-jazzy-ros-base ros-jazzy-rviz2 ros-jazzy-rviz-default-plugins ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-joy ros-jazzy-image-transport-plugins ros-jazzy-rqt-image-view ros-jazzy-slam-toolbox ros-jazzy-nav2-msgs \
  ros-jazzy-diagnostic-msgs ros-jazzy-tf2-ros python3-colcon-common-extensions

say "4/7 코드 (perseverance) 와 빌드"
[ -d ~/perseverance/.git ] || git clone https://github.com/AkimaraYouki/perseverance.git ~/perseverance
git -C ~/perseverance pull --ff-only || true
mkdir -p ~/gen2_desk
( cd ~/gen2_desk && source /opt/ros/jazzy/setup.bash && \
  colcon build --base-paths ~/perseverance/src/gen2_msgs ~/perseverance/src/gen2_tools ~/perseverance/src/gen2_bringup \
               --packages-select gen2_msgs gen2_tools gen2_bringup )

say "5/7 DDS (Cyclone, 도메인 42, 로봇 $ROBOT_IP 를 peer 로)"
IF=$(ls /sys/class/net | grep -m1 '^wl' || true)
cat > ~/gen2_desk/cyclonedds.xml <<XML
<?xml version="1.0" encoding="UTF-8"?>
<!-- GCS 노트북 → 로봇. Wi-Fi $IF, 로봇 $ROBOT_IP 를 peer 로 (멀티캐스트가 막힌 망에서도 찾게). -->
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces><NetworkInterface name="$IF"/></Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>60</MaxAutoParticipantIndex>
      <Peers><Peer address="$ROBOT_IP"/></Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
XML
cat > ~/gen2_desk/env.sh <<'ENV'
# 로봇 조종·관제용 환경: source ~/gen2_desk/env.sh
source /opt/ros/jazzy/setup.bash
source ~/gen2_desk/install/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$HOME/gen2_desk/cyclonedds.xml
ENV

say "6/7 관제 도구 (cw, wifi, 상태판, HUD, RViz 설정)"
mkdir -p ~/bin
T=~/perseverance/tools/desktop
cp $T/cw $T/wifi $T/slam_reset ~/bin/ && chmod +x ~/bin/cw ~/bin/wifi ~/bin/slam_reset
cp $T/desk_dash.py $T/desk_dash_robot.py $T/hud.py $T/gen2_desktop.rviz ~/gen2_desk/
grep -q 'HOME/bin' ~/.bashrc || echo 'export PATH="$HOME/bin:$PATH"' >> ~/.bashrc

say "7/7 손짓 인식 (MediaPipe 가상환경 + 모델)"
if [ ! -x ~/gen2_desk/venv_vision/bin/python ]; then
  python3 -m venv --system-site-packages ~/gen2_desk/venv_vision
fi
~/gen2_desk/venv_vision/bin/pip install -q mediapipe "numpy<2" "opencv-contrib-python<4.11"
mkdir -p ~/gen2_ws/models
[ -s ~/gen2_ws/models/gesture_recognizer.task ] || wget -q -O ~/gen2_ws/models/gesture_recognizer.task \
  https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/latest/gesture_recognizer.task

say "끝"
echo "남은 것 (ROBOT 망에서, 한 번):"
echo "  1) ssh-keygen -t ed25519 (Enter 세 번) → ssh-copy-id parksudo@$ROBOT_IP   (로봇 비밀번호)"
echo "  2) 새 터미널 → cw check → cw up"
echo "  3) 로봇 dds_peers 에 이 노트북 IP ($(hostname -I | awk '{print $1}')) 추가 요청 (데스크톱 에이전트가 comms 로 보냄)"
