from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_robot_description(urdf_path):
    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    return robot_description.removeprefix('<?xml version="1.0" encoding="UTF-8"?>\n').strip()


def generate_launch_description():
    package_share = get_package_share_directory('3d_lidar')
    # debug param for simulated motion
    spin_speed_rad_s = LaunchConfiguration('spin_speed_rad_s')
    spin_publish_rate_hz = LaunchConfiguration('spin_publish_rate_hz')
    lidar_product_name = LaunchConfiguration('lidar_product_name')
    lidar_port_name = LaunchConfiguration('lidar_port_name')
    lidar_port_baudrate = LaunchConfiguration('lidar_port_baudrate')
    lidar_scan_dir = LaunchConfiguration('lidar_scan_dir')
    urdf_path = f'{package_share}/model/robot.urdf'
    robot_description = load_robot_description(urdf_path)

    return LaunchDescription([
        DeclareLaunchArgument(
            'spin_speed_rad_s',
            default_value='1.0',
            description='Rotating lidar joint speed in radians per second',
        ),
        DeclareLaunchArgument(
            'spin_publish_rate_hz',
            default_value='250.0',
            description='Joint state publish rate for the rotating lidar',
        ),
        DeclareLaunchArgument(
            'lidar_product_name',
            default_value='LDLiDAR_STL27L',
            description='LDLiDAR product name',
        ),
        DeclareLaunchArgument(
            'lidar_port_name',
            default_value='/dev/ttyUSB0',
            description='Serial port used by the LiDAR driver',
        ),
        DeclareLaunchArgument(
            'lidar_port_baudrate',
            default_value='921600',
            description='Serial baudrate used by the LiDAR driver',
        ),
        DeclareLaunchArgument(
            'lidar_scan_dir',
            default_value='false',
            description='Set true for counterclockwise scan, false for clockwise',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_base_link_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        ),
        Node(
            package='3d_lidar',
            executable='joint_spinner',
            name='lidar_joint_spinner',
            parameters=[{
                'joint_name': 'base_to_lidar_rotator_joint',
                'speed_rad_s': spin_speed_rad_s,
                'publish_rate_hz': spin_publish_rate_hz,
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
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
        ),
    ])
