from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Nó para iniciar o nó de navegação
    navigation = Node(
        package='ros_alvw_navigation',  # Substitua pelo nome do seu pacote
        executable='navigation',
        name='navigation',
        output='screen',
        emulate_tty=True,  # Permite a exibição de mensagens de log no terminal
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(navigation)

    return ld