from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
        'gui', default_value='false', 
        description='Flag to enable GUI'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
        'description_package', 
        default_value='robotiq_hande_description', 
        description='Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
        'description_file', 
        default_value='robotiq_hande_world.urdf.xacro', 
        description='URDF/XACRO description file'
        )
    )
    
    #General Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    rviz_config_file =  PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "robotiq_hande.rviz"]
    )

    #Parameter
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), 
        " ", 
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file])])
    
    robot_description = {"robot_description": robot_description_content}

    # Debug print or logging
    #print(f"robot_description_content: {robot_description_content.perform(None)}")

    gui = {'use_gui': LaunchConfiguration('gui')}

    # nodes to launch
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',  # In ROS 2, you might want to use `joint_state_publisher_gui` for GUI support
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        parameters=[gui]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    nodes_to_start = [ 
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,]
    
    return LaunchDescription(declared_arguments + nodes_to_start)
