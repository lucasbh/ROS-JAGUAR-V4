# drrobotV2_player
The project is for Jaguar V2/V4 robot from Dr Robot Inc. This project is also need DrRobotMotionSensorDriver project. Please fork or copy these 2 poject to your catkin working space under ROS.

**drrobotV2_player**
    Este node tem o objetivo de realizar a leitura de sensores e controlar os motores do jaguar. Toda a comunicação é realizada em tcp/IP por sockets, e os comandos são todos enviados a partir deste node.

**drrobotV2_keyboard_teleop**
    Este node atua no controle de movimentação do jaguar. Os comandos digitados no teclado atuam na msg cmd_vel, especificamente em cmd_vel.linear_x e angular_x. Essas msg é então publicada e lida pelo player, que fará o controle da placa.
    
**drrobotV2_joy_teleop**
    Este node tem a mesma função do anterior, porém, seu controle é realizado por um joystick. Foi implementado ainda uma função de marchas, onde o usuário pode controlar melhor a velocidade do jaguar. 
    
**drrobotV2_imu**
    Node que faz a leitura do acelerometro e giroscópio do sensor "IMU 9DOF Razor". As leituras são publicadas em uma string e tratadas. Ainda não consegui colocá-lo para funcionar, a leitura retorna valores insconsistentes. 
