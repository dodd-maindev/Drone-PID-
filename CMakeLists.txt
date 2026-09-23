cmake_minimum_required(VERSION 3.16)
project(drone_control_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Tìm các gói thư viện Gazebo Transport và Gazebo Messages
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Thư viện Core Controller (Giao tiếp trực tiếp với Gazebo Sim & QGroundControl)
add_library(drone_lib STATIC
    src/comm/gz_bridge.cpp
    src/comm/mavlink_bridge.cpp
    src/controllers/altitude_controller.cpp
    src/core/vehicle_commander.cpp
    src/utils/data_logger.cpp
)

target_include_directories(drone_lib PUBLIC 
    include 
    third_party/mavlink
)
target_link_libraries(drone_lib PUBLIC
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    pthread
)
target_compile_options(drone_lib PUBLIC -Wno-address-of-packed-member)

# Bài 2: PID Altitude Hold (Trọng tâm dự án)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/experiments/02_pid_altitude_hold.cpp")
    add_executable(02_pid_altitude_hold experiments/02_pid_altitude_hold.cpp)
    target_link_libraries(02_pid_altitude_hold PRIVATE drone_lib)
endif()


if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/experiments/04_keyboard_teleop.cpp")
    add_executable(04_keyboard_teleop experiments/04_keyboard_teleop.cpp)
    target_link_libraries(04_keyboard_teleop PRIVATE drone_lib)
endif()

# Bài 5: Tự quay quanh trục đứng Yaw (Trái / Phải theo input)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/experiments/05_yaw_rotation_control.cpp")
    add_executable(05_yaw_rotation_control experiments/05_yaw_rotation_control.cpp)
    target_link_libraries(05_yaw_rotation_control PRIVATE drone_lib)
endif()

# Bài 6: Hardware-In-The-Loop (HITL) Gateway kết nối ESP32 với Gazebo Sim
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/experiments/06_hitl_esp32_bridge.cpp")
    add_executable(06_hitl_esp32_bridge experiments/06_hitl_esp32_bridge.cpp)
    target_link_libraries(06_hitl_esp32_bridge PRIVATE drone_lib)
endif()

