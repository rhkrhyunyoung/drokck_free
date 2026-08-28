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

    # 3. RealSense - 봄/겨울용 (Depth X)
    rs_basic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            'enable_color': 'true', 
            'enable_depth': 'false', 
            'serial_no': "'234322302402'",
            'rgb_camera.color_profile': '848x480x15',
            'initial_reset': 'true'
        }.items(),
        condition=IfCondition(PythonExpression(["'", season, "' in ['spring', 'winter']"]))
    )

    # 4. RealSense - 여름용 (Depth O + 시리얼 번호 추가)
    # 시리얼 번호는 보통 문자열로 전달하면 됩니다.
    rs_summer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true', 
            'serial_no': "'234322302402'", # 작은따옴표 안에 큰따옴표를 넣어 문자열임을 명시
            'rgb_camera.color_profile': '848x480x15', 
            'depth_module.depth_profile': '848x480x15',
            'align_depth.enable': 'true',
            'initial_reset': 'true'
        }.items(),
        condition=LaunchConfigurationEquals('season', 'summer')
    )

    # 6. 가을 전용 3캠 런치
    three_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(three_cam_path),
        condition=LaunchConfigurationEquals('season', 'autumn')
    )

    # 7. 가을 전용 주행 제어 노드 (직접 켜지 않아도 되게 추가)
    autumn_control_node = Node(
        package='drokck',
        executable='autumn_node',
        name='autumn_node',
        condition=LaunchConfigurationEquals('season', 'autumn'),
        output='screen'
    )

    # 8. YOLO (q 노드) - 봄, 여름, 겨울
    yolo_q = Node(
        package='yolo_xyz_publisher', 
        executable='q',
        condition=IfCondition(PythonExpression(["'", season, "' in ['spring', 'summer', 'winter']"]))
    )

    # 9. YOLO (Python 스크립트) - 가을 전용
    yolo_autumn = ExecuteProcess(
        cmd=['python3', yolo_autumn_path],
        condition=LaunchConfigurationEquals('season', 'autumn'),
        output='screen'
    )


    return LaunchDescription([
        LogInfo(msg=["Starting Integrated Launch for Season: ", season]),
        season_arg,
        chmod_tty,
        rs_basic,
        rs_summer,
        three_cam_launch,
        autumn_control_node,
        yolo_q,
        yolo_autumn,
    ])
