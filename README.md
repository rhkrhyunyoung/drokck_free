# drokck_free

Navigation 명령(cmd_vel_nav)을 받아 최종 로봇 속도(cmd_vel)로 제어하는 Behavior Tree 기반 미들웨어 패키지입니다.

![alt text](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)

![alt text](https://img.shields.io/badge/BehaviorTree-CPP-green)

![alt text](https://img.shields.io/badge/License-MIT-yellow.svg)

```
ros2 run drokck track_1_4_node
```
# Behavior Tree

graph TD
A[Start] --> B{Is System OK?}
B -- Yes --> C[Get cmd_vel_nav]
C --> D{Apply Constraints}
D --> E[Publish cmd_vel]
B -- No --> F[Stop Robot]


* /cmd_vel_nav	geometry_msgs/msg/Twist	(Input) 네비게이션 스택에서 생성된 속도

* /cmd_vel	geometry_msgs/msg/Twist	(Output) 로봇 베이스로 전달되는 최종 속도

drokck_free/
├── behavior_trees/     # BT XML 파일 정의
├── include/            # C++ 헤더 파일
├── src/                # BT Action/Condition 노드 구현 및 메인 로직
├── launch/             # 실행을 위한 Launch 파일
├── CMakeLists.txt
└── package.xml
