cmake_minimum_required(VERSION 3.16)
project(drone_control_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Tìm các gói thư viện Gazebo Transport và Gazebo Messages
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Thư viện Core Controller (Giao tiếp trực tiếp với Gazebo Sim)
add_library(drone_lib STATIC
    src/comm/gz_bridge.cpp
    src/controllers/altitude_controller.cpp
    src/core/vehicle_commander.cpp
)

target_include_directories(drone_lib PUBLIC include)
target_link_libraries(drone_lib PUBLIC
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    pthread
)

# Bài 2: PID Altitude Hold (Trọng tâm dự án)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/experiments/02_pid_altitude_hold.cpp")
    add_executable(02_pid_altitude_hold experiments/02_pid_altitude_hold.cpp)
    target_link_libraries(02_pid_altitude_hold PRIVATE drone_lib)
endif()

# Các bài mở rộng khác (nếu có trong thư mục cục bộ)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/experiments/01_open_loop_thrust.cpp")
    add_executable(01_open_loop_thrust experiments/01_open_loop_thrust.cpp)
    target_link_libraries(01_open_loop_thrust PRIVATE drone_lib)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/experiments/03_attitude_maneuver.cpp")
    add_executable(03_attitude_maneuver experiments/03_attitude_maneuver.cpp)
    target_link_libraries(03_attitude_maneuver PRIVATE drone_lib)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/experiments/04_keyboard_teleop.cpp")
    add_executable(04_keyboard_teleop experiments/04_keyboard_teleop.cpp)
    target_link_libraries(04_keyboard_teleop PRIVATE drone_lib)
endif()
