# Nome do arquivo: navigation_launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
            
    # Gazebo
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    gazebo_launch_args = {
        'verbose': 'false',
        'pause': 'false',
    }
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments = gazebo_launch_args.items(),
    )
    
    robot_name = 'waffle'

    # Use the path to the URDF file of the TurtleBot3
    robot_description_path = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', 'turtlebot3_waffle.urdf')

    # Node to load the URDF description into the ROS parameter server
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', robot_description_path, ' robot_name:=', robot_name])}],
        output='screen',
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=["-entity", robot_name, '-x', '-3.0', '-y', '1.0', '-z', '0.0',
                    '-topic', '/robot_description']
    )

    # Nó para iniciar o nó de navegação
    navigation = Node(
        package='ros_alvw_navigation',  # Substitua pelo nome do seu pacote
        executable='navigation',
        name='navigation',
        output='screen',
        emulate_tty=True,  # Permite a exibição de mensagens de log no terminal
    )
    return LaunchDescription([
        spawn_robot,
        gazebo,
        robot_state_publisher,
        navigation,
       ])
