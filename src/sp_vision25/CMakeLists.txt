cmake_minimum_required(VERSION 3.16.3)

project(sp_vision25)

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
# Keep an overridable default for the OpenVINO archive used by the lab machines.
if(NOT OpenVINO_DIR)
  set(_SP_VISION_OPENVINO_DEFAULT
    "$ENV{HOME}/openvino_toolkit_ubuntu22_2025.4.1.20426.82bbf0292c5_x86_64/runtime/cmake")
  if(EXISTS "${_SP_VISION_OPENVINO_DEFAULT}/OpenVINOConfig.cmake")
    set(OpenVINO_DIR "${_SP_VISION_OPENVINO_DEFAULT}")
  endif()
endif()
find_package(OpenVINO REQUIRED)

find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(fastrtps_cmake_module QUIET)
find_package(fastcdr QUIET CONFIG)
find_package(rmw QUIET)
find_package(sp_msgs QUIET)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)

####################src###################
add_executable(auto_aim_debug_mpc src/auto_aim_debug_mpc.cpp)
target_link_libraries(auto_aim_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools debug_visualization io)

set(SP_VISION_RUNTIME_TARGETS
  auto_aim_debug_mpc
)

##################calibration################
add_executable(capture calibration/capture.cpp)
add_executable(calibrate_camera calibration/calibrate_camera.cpp)
add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
add_executable(split_video calibration/split_video.cpp)

target_link_libraries(capture ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_robotworld_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(split_video ${OpenCV_LIBS} fmt::fmt tools)

list(APPEND SP_VISION_RUNTIME_TARGETS
  capture
  calibrate_camera
  calibrate_handeye
  calibrate_robotworld_handeye
  split_video
)

##################tests##################
add_executable(camera_detect_test tests/camera_detect_test.cpp)
add_executable(camera_test tests/camera_test.cpp)
add_executable(fire_test tests/fire_test.cpp)
add_executable(gimbal_test tests/gimbal_test.cpp)
add_executable(detector_video_test tests/detector_video_test.cpp)
add_executable(handeye_test tests/handeye_test.cpp)
add_executable(planner_test tests/planner_test.cpp)
add_executable(planner_test_offline tests/planner_test_offline.cpp)

target_link_libraries(camera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim io)
target_link_libraries(camera_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(fire_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(gimbal_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
target_link_libraries(detector_video_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim)
target_link_libraries(handeye_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim io)
target_link_libraries(planner_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
target_link_libraries(planner_test_offline ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

list(APPEND SP_VISION_RUNTIME_TARGETS
  camera_detect_test
  camera_test
  fire_test
  gimbal_test
  detector_video_test
  handeye_test
  planner_test
  planner_test_offline
)

if(ament_cmake_FOUND)
    # Runtime executables are installed to lib/${PROJECT_NAME};
    # configs/assets are installed to share/${PROJECT_NAME}.
    install(
      TARGETS ${SP_VISION_RUNTIME_TARGETS}
      DESTINATION lib/${PROJECT_NAME}
    )

    install(
      DIRECTORY configs assets
      DESTINATION share/${PROJECT_NAME}
    )

    install(
      FILES README.md run_with_watchdog.sh debug_mpc_watchdog.sh
      DESTINATION share/${PROJECT_NAME}
    )

    ament_package()
endif()
