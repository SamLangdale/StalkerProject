import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_robot_description(urdf_path):
    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    # Gazebo's spawn_entity expects XML content without an XML declaration.
    return robot_description.removeprefix('<?xml version="1.0" encoding="UTF-8"?>\n').strip()


def generate_launch_description():
    package_share = get_package_share_directory('3d_lidar')
    gazebo_model_root = os.path.dirname(package_share)
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    existing_resource_path = os.environ.get('GAZEBO_RESOURCE_PATH', '')
    lidar_product_name = LaunchConfiguration('lidar_product_name')
    lidar_port_name = LaunchConfiguration('lidar_port_name')
    lidar_port_baudrate = LaunchConfiguration('lidar_port_baudrate')
    lidar_scan_dir = LaunchConfiguration('lidar_scan_dir')
    urdf_path = f'{package_share}/model/robot.urdf'
    robot_description = load_robot_description(urdf_path)

    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_product_name',
            default_value='LDLiDAR_LD06',
            description='LDLiDAR product name',
        ),
        DeclareLaunchArgument(
            'lidar_port_name',
            default_value='/dev/ttyUSB0',
            description='Serial port used by the LiDAR driver',
        ),
        DeclareLaunchArgument(
            'lidar_port_baudrate',
            default_value='230400',
            description='Serial baudrate used by the LiDAR driver',
        ),
        DeclareLaunchArgument(
            'lidar_scan_dir',
            default_value='true',
            description='Set true for counterclockwise scan, false for clockwise',
        ),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=os.pathsep.join(
                path for path in [gazebo_model_root, existing_model_path] if path
            ),
        ),
        SetEnvironmentVariable(
            name='GAZEBO_RESOURCE_PATH',
            value=os.pathsep.join(
                path for path in [gazebo_model_root, existing_resource_path] if path
            ),
        ),
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='3d_lidar',
            executable='joint_spinner',
            name='lidar_joint_spinner',
            parameters=[{
                'joint_name': 'base_to_lidar_rotator_joint',
                'speed_rad_s': 0.5,
                'publish_rate_hz': 50.0,
            }],
        ),
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='ldlidar_driver',
            output='screen',
            parameters=[{
                'product_name': lidar_product_name,
                'topic_name': 'scan',
                'frame_id': 'laser',
                'port_name': lidar_port_name,
                'port_baudrate': lidar_port_baudrate,
                'laser_scan_dir': lidar_scan_dir,
                'enable_angle_crop_func': False,
                'angle_crop_min': 135.0,
                'angle_crop_max': 225.0,
            }],
        ),
        Node(
            package='3d_lidar',
            executable='scan_3d',
            name='scan_3d',
            output='screen',
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'cart_model',
                '-topic', 'robot_description',
            ],
            output='screen',
        ),
    ])
