# ros_alvw_navigation

Pacote ROS referente ao segundo trabalho de Robôs Autônomos.

## Procedimentos para a instalação do pacote

* No terminal, abra a area de trabalho ROS (ros2_ws), e em seguida o diretório src:
```
cd ~/ros2_ws/src
```
* Faça o clone do pacote do GitHub:
```
git clone git@github.com:viniciusbaltoe/ros_alvw_navigation.git
```
* Retorne ao diretório do WorkSpace (ros2_ws) e faça a atualização do colcon:
```
cd ~/ros2_ws && colcon build && source install/setup.bash
```

## Tutoriais de utilização do pacote

* Inicialização direta do Node de navegação:
```
ros2 run ros_alvw_navigation navigation
```
* Inicialização de um dos mundos construídos no gazebo e do node de navegação utilizando launch'file:
```
ros2 launch ros_alvw_navigation robot_navigation_launch.py map_file:=/caminho/do/seu/mapa.yaml initial_pose:="x:=1.0 y:=2.0 z:=0.0"
```

## Debug

* Exemplo de publicação de velocidade em um tópico direto do terminal:
```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```
* Exemplo de publicação de uma /scan:
```
ros2 topic pub /scan sensor_msgs/msg/LaserScan '{ranges: [2.0, 2.0, 3.0], angle_min: 0.0, angle_max: 1.0}'
```