# ros_alvw_navigation

[![ROS2](https://img.shields.io/badge/ROS2-Humble-green)](https://docs.ros.org/en/humble/index.html)
[![Python](https://img.shields.io/badge/Python-v3.7-blue)](https://www.python.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-v22.04.4-red)](https://ubuntu.com/download)
[![Gazebo](https://img.shields.io/badge/Gazebo-vX-orange)](https://gazebosim.org/docs)

## Sobre

Este projeto investiga os fundamentos da percepção ao empregar um sistema robótico para a execução de uma tarefa específica: identificar e contar obstáculos em um ambiente.

## Motivação

O impulso central deste empreendimento reside na aplicação prática dos conhecimentos teóricos adquiridos durante as aulas da disciplina de Robôs Autônomos ministradas pelo Prof. Dr. Ricardo Carminati de Mello, na Universidade Federal do Espírito Santo (UFES), visando a solução de problemas no campo da robótica. Além disso, o projeto em desenvolvimento tem o potencial de ser implementado em um robô real, aprimorando seu processo de percepção e ampliando a extração de informações do ambiente, como a contagem precisa de obstáculos. Essas informações podem ser aplicadas para verificar a densidade de obstáculos no ambiente, bem como estimar suas posições no referencial global.
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
* Inicialização de um dos mundos construídos no gazebo:
```
ros2 launch ros_alvw_navigation gazebo_launch.py headless:=False x_pose:=2.0 y_pose:=1.0
```
* Inicialização do node de navegação e identificação de obstáculos:
```
ros2 launch ros_alvw_navigation navigation_launch.py
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
