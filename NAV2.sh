#!/usr/bin/env bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_SCRIPT="$WORKSPACE_DIR/install/setup.bash"
WORLD_NAME="${1:-home}"
USE_RVIZ="${USE_RVIZ:-True}"
RVIZ_FORCE_SOFTWARE="${RVIZ_FORCE_SOFTWARE:-0}"
PID_FILE="$WORKSPACE_DIR/.ros/nav2_sh.pid"

export ROS_HOME="$WORKSPACE_DIR/.ros"
export ROS_LOG_DIR="$ROS_HOME/log"
cd "$WORKSPACE_DIR" || exit 1

declare -a commands=(
  "ros2 launch pb2025_sentry_bringup bringup.launch.py world:=$WORLD_NAME slam:=False use_rviz:=$USE_RVIZ rviz_force_software:=$RVIZ_FORCE_SOFTWARE"
)

start_command() {
  local cmd="$1"
  mkdir -p "$(dirname "$PID_FILE")"
  gnome-terminal -- bash -lc "export ROS_HOME=\"$ROS_HOME\"; export ROS_LOG_DIR=\"$ROS_LOG_DIR\"; echo \$\$ > \"$PID_FILE\"; cd \"$WORKSPACE_DIR\"; source \"$SETUP_SCRIPT\"; $cmd; rm -f \"$PID_FILE\"; exec bash"
}

is_running() {
  if [[ ! -f "$PID_FILE" ]]; then
    return 1
  fi

  local pid
  pid="$(<"$PID_FILE")"
  [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null
}

start_commands() {
  for cmd in "${commands[@]}"; do
    start_command "$cmd"
    sleep 3
  done
}

watch_commands() {
  while true; do
    for cmd in "${commands[@]}"; do
      if ! is_running; then
        echo "$cmd 未在运行, 重新启动"
        start_command "$cmd"
        sleep 3
      fi
    done
    sleep 5
  done
}

start_vision() {
  # 视觉通过 ROS 话题复用导航串口，只启动主程序。
  mkdir -p "$WORKSPACE_DIR/.ros"
  gnome-terminal --title="ATS 视觉" -- bash -lc '
    WORKSPACE_DIR="$1"
    cd "$WORKSPACE_DIR" || exit 1

    # 独立锁防止本脚本重复启动第二份视觉、占用相机和网页端口。
    exec 9>"$WORKSPACE_DIR/.ros/nav2_vision.lock"
    if ! flock -n 9; then
      echo "视觉已经由本脚本启动，跳过重复启动。"
      exit 0
    fi

    source /opt/ros/humble/setup.bash || exit 1
    source "$WORKSPACE_DIR/install/setup.bash" || exit 1
    export HIKROBOT_SDK_ROOT=/opt/MVS
    export ROS_HOME="$WORKSPACE_DIR/.ros"
    export ROS_LOG_DIR="$ROS_HOME/log"
    config="$WORKSPACE_DIR/install/sp_vision25/share/sp_vision25/configs/standard3.yaml"

    trap "exit 0" INT TERM
    echo "视觉网页：http://127.0.0.1:8090/"
    while true; do
      ros2 run sp_vision25 auto_aim_debug_mpc "$config"
      echo "视觉程序已退出，3 秒后重新启动；关闭本终端可停止视觉。"
      sleep 3
    done
  ' ats-vision "$WORKSPACE_DIR"
}

start_commands
start_vision
watch_commands
