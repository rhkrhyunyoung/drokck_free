cmake_minimum_required(VERSION 3.8)
project(drokck)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 필요한 패키지 로드
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED) # OpenCV 로드
find_package(message_filters REQUIRED)

# 의존성 리스트 정리
set(dependencies
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  cv_bridge
  message_filters
)

# -----------------------------------------------------------------
# 2. 각 계절별 노드 빌드 설정
# -----------------------------------------------------------------

# 봄 노드
add_executable(spring_node spring_node.cpp)
ament_target_dependencies(spring_node ${dependencies})
target_link_libraries(spring_node ${OpenCV_LIBRARIES})

# 여름 노드
add_executable(summer_node summer_node.cpp)
ament_target_dependencies(summer_node ${dependencies})
target_link_libraries(summer_node ${OpenCV_LIBRARIES})

# 가을 노드
add_executable(autumn_node autumn_node.cpp)
ament_target_dependencies(autumn_node ${dependencies})
target_link_libraries(autumn_node ${OpenCV_LIBRARIES})

# 겨울 노드
add_executable(winter_node winter_node.cpp)
ament_target_dependencies(winter_node ${dependencies})
target_link_libraries(winter_node ${OpenCV_LIBRARIES})

# (기존) 5구간 노드가 있다면 추가
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/track_5_node.cpp")
  add_executable(track_5_node track_5_node.cpp)
  ament_target_dependencies(track_5_node ${dependencies})
  target_link_libraries(track_5_node ${OpenCV_LIBRARIES})
endif()

# -----------------------------------------------------------------
# 3. 설치 경로 지정
# -----------------------------------------------------------------
install(TARGETS
  spring_node
  summer_node
  autumn_node
  winter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
