cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)

add_subdirectory(serial)

option(SP_VISION_ENABLE_HIKROBOT "Build the HikRobot camera backend" ON)

set(IO_SOURCES
  camera.cpp
  gimbal/gimbal.cpp
)

if(SP_VISION_ENABLE_HIKROBOT)
  set(HIKROBOT_SDK_HINT_PATHS
    $ENV{HIKROBOT_SDK_ROOT}
    $ENV{MVCAM_SDK_ROOT}
    $ENV{MVCAM_COMMON_RUNENV}
    /opt/MVS
    /usr/local/MVS
    /opt/hikrobot/MVS
    /opt/HikRobot/MVS
    ${CMAKE_SOURCE_DIR}/../dependencies/hik_camera_ros2_driver/hikSDK
    ${CMAKE_CURRENT_SOURCE_DIR}/hikrobot
  )

  find_path(HIKROBOT_INCLUDE_DIR
    NAMES MvCameraControl.h
    PATHS ${HIKROBOT_SDK_HINT_PATHS}
    PATH_SUFFIXES include MvImport/include
  )

  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(HIKROBOT_LIB_SUFFIXES lib lib/64 lib/amd64 MvImport/lib/64 MvImport/lib/amd64)
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(HIKROBOT_LIB_SUFFIXES lib lib/arm64 MvImport/lib/arm64)
  else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
  endif()

  find_library(HIKROBOT_MVCAMERA_LIBRARY
    NAMES MvCameraControl
    PATHS ${HIKROBOT_SDK_HINT_PATHS}
    PATH_SUFFIXES ${HIKROBOT_LIB_SUFFIXES}
  )

  if(NOT HIKROBOT_INCLUDE_DIR OR NOT HIKROBOT_MVCAMERA_LIBRARY)
    message(FATAL_ERROR
      "HikRobot SDK not found. Set HIKROBOT_SDK_ROOT or configure with "
      "-DSP_VISION_ENABLE_HIKROBOT=OFF for ROS Image camera input.")
  endif()

  get_filename_component(HIKROBOT_LIBRARY_DIR "${HIKROBOT_MVCAMERA_LIBRARY}" DIRECTORY)
  message(STATUS "Using HikRobot SDK include dir: ${HIKROBOT_INCLUDE_DIR}")
  message(STATUS "Using HikRobot SDK library dir: ${HIKROBOT_LIBRARY_DIR}")
  list(APPEND IO_SOURCES hikrobot/hikrobot.cpp)
endif()

# io
add_library(io STATIC ${IO_SOURCES})

target_link_libraries(io yaml-cpp serial)
if(SP_VISION_ENABLE_HIKROBOT)
  target_compile_definitions(io PRIVATE SP_VISION_HAS_HIKROBOT)
  target_include_directories(io PUBLIC "${HIKROBOT_INCLUDE_DIR}")
  target_link_directories(io PUBLIC "${HIKROBOT_LIBRARY_DIR}")
  target_link_libraries(io "${HIKROBOT_MVCAMERA_LIBRARY}" usb-1.0)
endif()

################## ROS 2 ##################
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(example_interfaces QUIET)
find_package(geometry_msgs QUIET)
find_package(sensor_msgs QUIET)
find_package(pb_rm_interfaces QUIET)
find_package(tf2 QUIET)
find_package(tf2_ros QUIET)
find_package(tf2_geometry_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(fastrtps_cmake_module QUIET)
find_package(fastcdr QUIET CONFIG)
find_package(rmw QUIET)
find_package(sp_msgs QUIET)

# If the ROS 2 environment is unavailable, skip the ROS-specific build.
if(NOT ament_cmake_FOUND OR NOT rclcpp_FOUND OR NOT std_msgs_FOUND OR NOT example_interfaces_FOUND OR NOT geometry_msgs_FOUND OR NOT sensor_msgs_FOUND OR NOT pb_rm_interfaces_FOUND OR NOT tf2_FOUND OR NOT tf2_ros_FOUND OR NOT tf2_geometry_msgs_FOUND OR NOT rosidl_typesupport_cpp_FOUND OR NOT fastrtps_cmake_module_FOUND OR NOT fastcdr_FOUND OR NOT rmw_FOUND OR NOT sp_msgs_FOUND)
    message(WARNING "ROS2 not found, skipping ROS2 specific code.")
else()
    message(STATUS "ROS2 environment found, compiling ROS2-related code.")
    target_compile_definitions(io PRIVATE SP_VISION_HAS_ROS2)

    # sp_msgs rosidl interfaces; also gimbal topic bridge uses only rclcpp/std_msgs.
    target_sources(io PRIVATE
      ros2/publish2nav.cpp
      ros2/ros2.cpp
      ros2/subscribe2nav.cpp
      ros2/gimbal_topic_bridge.cpp
    )

    include_directories(${std_msgs_INCLUDE_DIRS})
    ament_target_dependencies(
      io rclcpp std_msgs example_interfaces geometry_msgs sensor_msgs pb_rm_interfaces
      tf2 tf2_ros tf2_geometry_msgs sp_msgs)
endif()
