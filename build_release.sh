#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ros_setup="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
build_workers="${BUILD_WORKERS:-2}"

if [[ ! -f "${ros_setup}" ]]; then
  printf 'ROS 2 setup file not found: %s\n' "${ros_setup}" >&2
  exit 2
fi

# ROS setup scripts read optional variables that may be unset. Disable nounset
# while sourcing them, then restore it for the rest of this script.
set +u
# shellcheck disable=SC1090
source "${ros_setup}"
set -u

for command_name in colcon rosdep; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    printf 'Required command not found: %s\n' "${command_name}" >&2
    exit 2
  fi
done

if [[ -z "${OpenVINO_DIR:-}" ]]; then
  openvino_candidates=(
    "${HOME}/openvino_toolkit_ubuntu22_2025.4.1.20426.82bbf0292c5_x86_64/runtime/cmake"
    "/opt/intel/openvino_2025/runtime/cmake"
    "/opt/intel/openvino/runtime/cmake"
  )
  for candidate in "${openvino_candidates[@]}"; do
    if [[ -f "${candidate}/OpenVINOConfig.cmake" ]]; then
      OpenVINO_DIR="${candidate}"
      break
    fi
  done
fi

if [[ -z "${OpenVINO_DIR:-}" || ! -f "${OpenVINO_DIR}/OpenVINOConfig.cmake" ]]; then
  printf 'OpenVINO 2025 CMake package not found. Set OpenVINO_DIR to its runtime/cmake directory.\n' >&2
  exit 2
fi

if [[ ! -f /usr/local/include/small_gicp/registration/registration.hpp &&
      ! -f /usr/include/small_gicp/registration/registration.hpp ]]; then
  printf 'small_gicp headers not found. Install small_gicp before building this workspace.\n' >&2
  exit 2
fi

if [[ -z "${HIKROBOT_SDK_ROOT:-}" ]]; then
  embedded_hikrobot="${workspace_root}/src/sp_vision25/io/hikrobot"
  if [[ -f "${embedded_hikrobot}/include/MvCameraControl.h" ]]; then
    HIKROBOT_SDK_ROOT="${embedded_hikrobot}"
  elif [[ -f /opt/MVS/include/MvCameraControl.h ]]; then
    HIKROBOT_SDK_ROOT=/opt/MVS
  elif [[ -f /usr/local/MVS/include/MvCameraControl.h ]]; then
    HIKROBOT_SDK_ROOT=/usr/local/MVS
  fi
fi
if [[ -n "${HIKROBOT_SDK_ROOT:-}" ]]; then
  export HIKROBOT_SDK_ROOT
fi

vision_hikrobot="${SP_VISION_ENABLE_HIKROBOT:-ON}"
if [[ "${vision_hikrobot}" == "ON" && -z "${HIKROBOT_SDK_ROOT:-}" ]]; then
  printf 'HikRobot SDK not found; disabling the HikRobot backend for this build.\n'
  vision_hikrobot=OFF
fi

cd "${workspace_root}"
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y

colcon build \
  --symlink-install \
  --parallel-workers "${build_workers}" \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    "-DOpenVINO_DIR=${OpenVINO_DIR}" \
    "-DSP_VISION_ENABLE_HIKROBOT=${vision_hikrobot}"

printf '\nRelease build complete. Load the workspace with:\nsource %s/install/setup.bash\n' "${workspace_root}"
