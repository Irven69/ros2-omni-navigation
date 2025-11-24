from launch.actions import TimerAction
from launch.actions import OpaqueFunction
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import os
import time
try:
    import serial
except Exception:
    serial = None



urdf_path = os.path.join(get_package_share_directory('smoothop'),'urdf', 'smoothop.urdf')
with open(urdf_path, 'r') as file:
    robot_description = file.read()



def generate_launch_description():
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')
    max_distance = LaunchConfiguration('max_distance', default='6.0')

    nav2_launch_file = os.path.join(get_package_share_directory('nav2_bringup'), 
                                    'launch', 'bringup_launch.py')
    # SLAM launch (used when enable_slam is true)
    slam_launch_file = os.path.join(get_package_share_directory('slam_toolbox'),
                                    'launch', 'online_sync_launch.py')
    map_file = os.path.join(get_package_share_directory('smoothop'), 'map', 'BIC2.yaml')
    config_file = os.path.join(get_package_share_directory('smoothop'), 'config', 'nav2_params.yaml')

    def wait_for_serial_device(context, timeout_s: float = 10.0):
        port = LaunchConfiguration('serial_port').perform(context)
        waited = 0.0
        poll_interval = 0.5
        while waited < timeout_s:
            if os.path.exists(port):
                print(f"found serial device {port} after {waited:.1f}s")
                return []
            print(f"waiting for {port} ({waited:.1f}s)...")
            time.sleep(poll_interval)
            waited += poll_interval
        print(f"warning: device {port} not found after {timeout_s}s, continuing launch")
        return []

    def reset_serial(context):
        if serial is None:
            print("pyserial not installed; skipping serial reset")
            return []
        port = LaunchConfiguration('serial_port').perform(context)
        baud_s = LaunchConfiguration('serial_baudrate').perform(context)
        try:
            baud = int(baud_s) if baud_s else 115200
        except Exception:
            baud = 115200
        try:
            s = serial.Serial(port, baud, timeout=0.5)
            s.setDTR(False)
            time.sleep(0.05)
            s.setDTR(True)
            s.close()
            print(f"serial reset on {port} ok")
        except Exception as e:
            print(f"serial reset failed: {e}")
        return []

    return LaunchDescription([
        Node(
            package='smoothop',
            executable='motionControl',
            name='Motion_controller_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'ERROR']  # Changed from WARN
        ),
        Node(
            package='smoothop',
            executable='odom',
            name='Odometry_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'ERROR']  # Changed from WARN
        ),
        Node(
            package='smoothop',
            executable='motorControl',
            name='Motor_controller_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'ERROR']  # Changed from WARN
        ),
        # Node(
        #     package='joy',
        #     executable='joy_node',
        #     name='joy_node'
        # ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            arguments=['--ros-args', '--log-level', 'ERROR']  # Changed from WARN
        ),  
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),
        
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),
            
        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),
        
        DeclareLaunchArgument(
            'max_distance',
            default_value=max_distance,
            description='Maximum lidar distance in meters'),

        DeclareLaunchArgument(
            'enable_localization',
            default_value='false',
            description='Enable Nav2 localization (AMCL). Set true to start localization'),

        DeclareLaunchArgument(
            'enable_slam',
            default_value='false',
            description='Enable SLAM (slam_toolbox). Set true to start online SLAM'),

        OpaqueFunction(function=wait_for_serial_device),
         TimerAction(
              period=3.0,
              actions=[
                  Node(
                      package='sllidar_ros2',
                      executable='sllidar_node',
                      name='sllidar_node',
                      parameters=[{'channel_type': channel_type,
                                   'serial_port': serial_port,
                                   'serial_baudrate': serial_baudrate,
                                   'frame_id': frame_id,
                                   'inverted': inverted,
                                   'angle_compensate': angle_compensate,
                                   'max_distance': max_distance}],
                      output='screen',
                      respawn=False,
                      arguments=['--ros-args', '--log-level', 'INFO']  # Changed from DEBUG
                  )
              ]
          ),
        TimerAction(
            period=10.0,
            condition=IfCondition(LaunchConfiguration('enable_localization')),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch_file),
                    launch_arguments={
                        'use_sim_time':'False',
                        'map':map_file,
                        'params_file':config_file
                    }.items()
                )
            ]
        ),
        TimerAction(
            period=10.0,
            condition=IfCondition(LaunchConfiguration('enable_slam')),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(slam_launch_file),
                    launch_arguments={
                        'use_sim_time':'False'
                    }.items()
                )
            ]
        ),

        # If SLAM is enabled also start nav2 (navigation) after a short delay so SLAM can initialize.
        TimerAction(
            period=20.0,  # wait for slam_toolbox to initialize and start publishing /map
            condition=IfCondition(LaunchConfiguration('enable_slam')),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch_file),
                    launch_arguments={
                        # For SLAM-based navigation we still load nav2 params.
                        # We do not provide a static map when using online SLAM; nav2 will operate using map updates.
                        'use_sim_time':'False',
                        'map': '',               # empty -> no static map file
                        'params_file': config_file
                    }.items()
                )
            ]
        ),
    ])
"""
Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen'
),

"""
"""
Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{'robot_description': robot_description}]
),
"""
'''
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '1', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'smoothop']
        ),
'''
