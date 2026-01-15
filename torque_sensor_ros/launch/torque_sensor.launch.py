from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyUSB0',
            description='Serial port for the torque sensor'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='256000',
            description='Baudrate for serial communication'
        ),
        DeclareLaunchArgument(
            'sensor_type',
            default_value='RANGE_30NM',
            description='Sensor type: RANGE_30NM or RANGE_100NM'
        ),
        DeclareLaunchArgument(
            'zero_voltage',
            default_value='5.0',
            description='Voltage corresponding to 0 Nm'
        ),
        DeclareLaunchArgument(
            'max_voltage',
            default_value='10.0',
            description='Voltage corresponding to max range'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='100.0',
            description='Rate at which to publish torque data (Hz)'
        ),

        Node(
            package='torque_sensor_ros',
            executable='torque_sensor_ros_node',
            name='torque_sensor',
            output='screen',
            parameters=[{
                'port_name': LaunchConfiguration('port_name'),
                'baudrate': LaunchConfiguration('baudrate'),
                'sensor_type': LaunchConfiguration('sensor_type'),
                'zero_voltage': LaunchConfiguration('zero_voltage'),
                'max_voltage': LaunchConfiguration('max_voltage'),
                'publish_rate': LaunchConfiguration('publish_rate')
            }]
        )
    ])
