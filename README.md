# drokck_free

Lane Tracer (Python): 카메라 영상을 분석하여 최적의 주행 경로(의견)를 /cmd_vel_nav 토픽으로 보냅니다.

Mission Manager (C++): YOLO 인식 결과와 IMU 데이터를 기반으로 주행 신호를 필터링하여 최종 제어권을 행사하고 /cmd_vel 토픽을 통해 차체를 구동합니다.

```
ros2 run drokck track_1_4_node
```

/cmd_vel_nav	geometry_msgs/Twist	라인 트레이서의 주행 제안 속도

/yolo_detected_object	std_msgs/String	YOLO 객체 인식 결과 (STOP, GO, cargo_zone)

/imu/data	sensor_msgs/Imu	슬립 보정을 위한 차체 각속도 데이터

/cmd_vel	geometry_msgs/Twist	최종 결정된 차체 구동 속도

