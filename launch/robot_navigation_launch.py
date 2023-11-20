from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Nó para iniciar o Gazebo
        Node(
            package='gazebo_ros',
            executable='gazebo',
            name='gazebo',
            output='screen',
            arguments=['-s', 'libgazebo_ros_factory.so']
        ),
        
        # Nó para iniciar o mundo do TurtleBot3 no Gazebo
        Node(
            package='turtlebot3_gazebo',
            executable='turtlebot3_fake_node',
            name='turtlebot3_fake_node',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'x_pos': '-3.0'},  # Valor padrão
                {'y_pos': '1.0'},   # Valor padrão
                {'z_pos': '0.0'},   # Valor padrão
            ],
            arguments=['waffle'],  # Valor padrão para o modelo
        ),

        # Nó para iniciar o nó de navegação
        Node(
            package='ros_alvw_navigation',
            executable='navigation.py',
            name='navigation',
            output='screen',
        ),
    ])
