#!/bin/bash
###
 # @Description: This is a ros-based project!
 # @Author: Liu Biao
 # @Date: 2023-01-02 22:20:13
 # @LastEditTime: 2023-05-05 15:23:45
 # @FigtPath: /TUP-Vision-2023-Based/WatchDog.sh
### 


WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_HOME="${WS_ROOT}"
cd "${WS_ROOT}"
# 璁剧疆闇€瑕佸惎鍔ㄥ拰鐩戣鐨剅os鍛戒护
declare -a commands=("ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
slam:=True \
use_robot_state_pub:=False")

# 瀹氫箟鍑芥暟鏉ュ惎鍔╮os鍛戒护
start_commands() {
  for cmd in "${commands[@]}"
  do
    gnome-terminal -- bash -c "source install/setup.bash;$cmd;exec bash;"
  done
}

# 瀹氫箟鍑芥暟鏉ョ洃瑙唕os杩涚▼鏄惁鍦ㄨ繍琛?watch_commands() {
  while true
  do
    for cmd in "${commands[@]}"
    do
      process_name=$(echo $cmd | awk '{print $1}') # 鎻愬彇杩涚▼鍚?      if ! pgrep -f "$cmd" > /dev/null
      then
        echo "$cmd 鏈湪杩愯, 閲嶆柊鍚姩"
    gnome-terminal -- bash -c "source install/setup.bash;$cmd;exec bash;"
      fi
    done
    sleep 1 # 璁剧疆鐫＄湢鏃堕棿锛屼互渚垮湪涓嬩竴娆℃鏌ヤ箣鍓嶇粰杩涚▼瓒冲鐨勬椂闂磋繍琛?  done
}

# 鍚姩ros鍛戒护鍜岀洃瑙嗚繘绋?start_commands
watch_commands
