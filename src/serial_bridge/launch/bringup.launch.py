from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # ---------- 1. Serial Bridge (Arduino + odom + TF) ----------
    serial_bridge = Node(
        package='serial_bridge',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen',
        parameters=[
            {'port': '/dev/ttyACM0'},
            {'baudrate': 115200},
            {'simulate': False},
        ],
    )

    # ---------- 2. LiDAR driver (LD14P) ----------
    ldlidar_dir = get_package_share_directory('ldlidar_sl_ros2')
    ldlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ldlidar_dir, 'launch', 'ld14p.launch.py')
        )
    )

    # ---------- 3. Scan Republisher ----------
    scan_republisher = ExecuteProcess(
        cmd=['python3', '/home/airon/ros2_ws_lidarSlam/republish_scan.py'],
        output='screen',
        name='scan_republisher',
    )

    # ---------- 4. SLAM Toolbox (online async) ----------
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'scan_topic': '/scan_fixed',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'transform_timeout': 10.0,
            'minimum_time_interval': 0.0,
            'minimum_travel_distance': 0.0,
            'minimum_travel_heading': 0.0,
        }],
    )

    # ---------- 5. Rosbridge (for mobile app) ----------
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090}],
    )

    # ---------- 6. Web Video Server (camera streaming) ----------
    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{'port': 8080}],
    )

    # ---------- 7. USB Camera Node for dashboard ----------
    usb_camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{'image_width': 640, 'image_height': 480}],
    )
    # ---------- 8. HTTP server for dashboard ----------
    http_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8081',
             '--directory', '/home/airon/ros2_ws_lidarSlam/web'],
        output='screen',
        name='http_server',
    )

    return LaunchDescription([
        serial_bridge,
        ldlidar_launch,
        scan_republisher,
        slam_toolbox,
        rosbridge,
        usb_camera,
        web_video_server,
        http_server,
    ])
