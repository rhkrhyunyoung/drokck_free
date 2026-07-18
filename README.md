# drokck_free

Navigation 명령(cmd_vel_nav)을 받아 최종 로봇 속도(cmd_vel)로 제어하는 Behavior Tree 기반 미들웨어 패키지입니다.

<p align="center">
  <img src="https://img.shields.io/github/stars/rhkrhyunyoung/drokck_free?style=for-the-badge&logo=github" alt="Stars">
  <img src="https://img.shields.io/github/forks/rhkrhyunyoung/drokck_free?style=for-the-badge&logo=github" alt="Forks">
  <img src="https://img.shields.io/github/license/rhkrhyunyoung/drokck_free?style=for-the-badge&logo=opensourceinitiative" alt="License">
  <img src="https://img.shields.io/github/issues/rhkrhyunyoung/drokck_free?style=for-the-badge&logo=github" alt="Issues">
</p>


```
ros2 run drokck track_1_4_node
```
# Behavior Tree
```
graph TD
A[Start] --> B{Is System OK?}
B -- Yes --> C[Get cmd_vel_nav]
C --> D{Apply Constraints}
D --> E[Publish cmd_vel]
B -- No --> F[Stop Robot]
```

* /cmd_vel_nav	geometry_msgs/msg/Twist	(Input) 네비게이션 스택에서 생성된 속도

* /cmd_vel	geometry_msgs/msg/Twist	(Output) 로봇 베이스로 전달되는 최종 속도

drokck_free/
-behavior_trees/
-include/
-src/
-launch/
-CMakeLists.txt
-package.xml
