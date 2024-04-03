from launch import LaunchDescription
from launch_ros.actions import Node



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


    return LaunchDescription([i2c_read,wireles_odom,tf1,tf2])
