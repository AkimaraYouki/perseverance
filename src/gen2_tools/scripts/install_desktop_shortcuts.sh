#!/bin/bash
# Creates Desktop launchers for the Gen2 bench tools.
set -e
DESK="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
WS="${GEN2_WS:-$HOME/gen2_ws}"
mk() {  # file, name, icon, command
  cat > "$DESK/$1" <<EOD
[Desktop Entry]
Type=Application
Name=$2
Icon=$3
Terminal=false
Exec=gnome-terminal --title="$2" --geometry=150x40 -- bash -ic '$4; echo; read -p "[Enter] to close"'
EOD
  chmod +x "$DESK/$1"
  gio set "$DESK/$1" metadata::trusted true 2>/dev/null || true
}
mk gen2-motor-test.desktop "Gen2 Motor Test CLI" utilities-terminal \
  "source $WS/install/setup.bash; ros2 run gen2_hardware motor_cli --ros-args --params-file $WS/install/gen2_hardware/share/gen2_hardware/config/motors.yaml"
mk gen2-motor-test-ui.desktop "Gen2 Motor Test UI" applications-engineering \
  "source $WS/install/setup.bash; ros2 launch gen2_hardware motor_test.launch.py"
mk gen2-sensor-hub-test.desktop "Gen2 Sensor Hub Test" utilities-system-monitor \
  "source $WS/install/setup.bash; ros2 run gen2_tools hub_cli"
mk gen2-bench-check.desktop "Gen2 Bench Check" emblem-default \
  "source $WS/install/setup.bash; ros2 run gen2_bringup gen2_bench_check.py"
echo "Created launchers in $DESK"
