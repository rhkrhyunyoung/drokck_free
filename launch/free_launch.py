import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, LogInfo
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 환경 설정 및 인자
    season = LaunchConfiguration('season')
    season_arg = DeclareLaunchArgument('season', default_value='spring')
    
    home_path = "/home/kudos"
    
    # --- 정확한 파일 경로 설정 ---
    three_cam_path = os.path.join(home_path, 'drokck_free/src/camera/three_cameras_launch.py')
    yolo_autumn_path = os.path.join(home_path, 'drokck_free/src/yolo_xyz_publisher/yolo_xyz_publisher/yolo_bbox_node.py')
    realsense_launch_path = os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')

    # 2. 공통 명령 (시리얼 권한)
    chmod_tty = ExecuteProcess(cmd=['sudo', 'chmod', '666', '/dev/ttyACM0'], output='screen')

    # 3. RealSense - 가을/겨울용 (Node 방식)
    rs_basic = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="camera",
        output="screen",
        parameters=[{
            "serial_no": "336222300167",
            "enable_color": True,
            "enable_depth": True,
            "enable_infra1": False,
            "enable_infra2": False,
            "enable_gyro": False,
            "enable_accel": False,
            "align_depth.enable": True,
            "enable_sync": True,
            "initial_reset": True,
            "rgb_camera.color_profile": "848x480x15",
            "depth_module.depth_profile": "848x480x15",
        }],
        condition=IfCondition(PythonExpression(["'", season, "' in ['autumn', 'winter']"]))
    )

    # 4. RealSense - 여름용
    rs_summer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'false', 
            'serial_no': "'336222300167'",
            'rgb_camera.color_profile': '848x480x15'
        }.items(),
        condition=LaunchConfigurationEquals('season', 'summer')
    )

    # 6. 봄 전용 3캠 런치
    three_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(three_cam_path),
        condition=LaunchConfigurationEquals('season', 'spring')
    )

    # 7. 가을 전용 주행 제어 노드
    autumn_control_node = Node(
        package='drokck',
        executable='autumn_node',
        name='autumn_node',
        condition=LaunchConfigurationEquals('season', 'spring'), # 기존 spring에서 autumn으로 수정 권장
        output='screen'
    )

    # 8. YOLO (q 노드) - 가을, 여름, 
    yolo_q = Node(
        package='yolo_xyz_publisher', 
        executable='q',
        condition=IfCondition(PythonExpression(["'", season, "' in ['autumn', 'summer', 'winter']"]))
    )

    # 9. YOLO (Python 스크립트) - 봄 전용
    yolo_autumn = ExecuteProcess(
        cmd=['python3', yolo_autumn_path],
        condition=LaunchConfigurationEquals('season', 'spring'),
        output='screen'
    )

    # 10. Linetracing (계절별 분기)
    # 가을: main1_node 실행
    linetracing_autumn = Node(
        package='linetracing_cpp', 
        executable='main1_node', 
        name='linetracing_node',
        condition=LaunchConfigurationEquals('season', 'autumn'),
        output='screen'
    )

    # 봄, 여름, 겨울: main2_node 실행
    linetracing_others = Node(
        package='linetracing_cpp', 
        executable='main2_node', 
        name='linetracing_node',
        condition=IfCondition(PythonExpression(["'", season, "' in ['spring', 'summer', 'winter']"])),
        output='screen'
    )

    return LaunchDescription([
        LogInfo(msg=["Starting Integrated Launch for Season: ", season]),
        season_arg,
        chmod_tty,
        rs_basic,
        rs_summer,
        # imu_node, # 정의되지 않은 변수이므로 필요시 정의 후 주석 해제하세요.
        three_cam_launch,
        autumn_control_node,
        yolo_q,
        yolo_autumn,
        linetracing_autumn,
        linetracing_others
    ])
