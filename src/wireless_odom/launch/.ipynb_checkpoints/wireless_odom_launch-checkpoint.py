from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    wireles_odom = Node(
            package='wireless_odom',
            executable='wireless_odom',
            name='wireless_odom',
        )
    
    i2c_read = Node(
            package='wireless_odom',
            executable='i2c_read',
            name='i2c_read',
        )
    
    base_move_jetracer = Node(
            package='base_move_jetracer',
            executable='base_move',
            name='base_move_jetracer',
        )
    
    tf1 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
        )
    tf2 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        )
    realsens = IncludeLaunchDescription(PythonLaunchDescriptionSource([FindPackageShare("realsens_speed_sens"), '/realsens_laser_launch.py']))
    

    return LaunchDescription([base_move_jetracer,realsens,i2c_read,wireles_odom,tf1,tf2])