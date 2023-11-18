# ros_alvw_navigation

Pacote ROS referente ao segundo trabalho de Robôs Autônomos.

## Procedimentos para a instalação do pacote

* No terminal, abra a area de trabalho ROS (ros2_ws), e em seguida o diretório src:

cd ~/ros2_ws/src

* Faça o clone do pacote do GitHub:

git clone git@github.com:viniciusbaltoe/ros_alvw_navigation.git

* Retorne ao diretório do WorkSpace (ros2_ws) e faça a atualização do colcon:

cd ~/ros2_ws && colcon build && source install/setup.bash


## Debug

* Exemplo de publicação de velocidade em um tópico direto do terminal:

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
