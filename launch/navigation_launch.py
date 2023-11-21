# Nome do arquivo: navigation_launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
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
        gazebo,
        spawn_robot,
        navigation,
       ])
