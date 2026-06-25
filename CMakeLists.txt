cmake_minimum_required(VERSION 3.8)
project(drokck)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 필요한 패키지 로드 (하드웨어 의존성 제거)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# -----------------------------------------------------------------
# 2-1. [1~4구간 노드] - 미션 매니저 (간단한 버전)
# -----------------------------------------------------------------
add_executable(track_1_4_node track_1_4_node.cpp)

ament_target_dependencies(track_1_4_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
)

# -----------------------------------------------------------------
# 2-2. [5구간 노드] - 5구간용이 따로 있다면 동일하게 처리
# -----------------------------------------------------------------
# 만약 track_5_node.cpp도 단순화된 로직을 사용한다면 아래처럼 수정하세요.
add_executable(track_5_node track_5_node.cpp)

ament_target_dependencies(track_5_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
)

# -----------------------------------------------------------------
# 3. 설치 경로 지정
# -----------------------------------------------------------------
install(TARGETS
  track_1_4_node
  track_5_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
