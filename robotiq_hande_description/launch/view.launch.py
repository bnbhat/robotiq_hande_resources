from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': open('/home/balachandra/thesis_ws/src/ur/robotiq_hande_resources/robotiq_hande_description/urdf/robotiq_hande_world.urdf.xacro', 'r').read()}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='both'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='both'
        ),
    ])
