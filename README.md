# Sera Olgunluk Tarayıcısı Robotu

## Ekip
    |      Üye       | Öğrenci No |               Görev                |
    |----------------|------------|------------------------------------|
    | Hacer Cansevdi | 1306220064 | Lidar, robot gövdesi, launch       |
    | Edanur Çınar   | 1306220037 | Kamera, sera dünyası               |
    | Hicran Demir   | 1306220039 | IMU, bumper, davranış entegrasyonu |

## Gereksinimler
Ubuntu 26.04, ROS 2 Lyrical, Gazebo Jetty

## Kurulum
ROS 2 Lyrical'ı resmi sayfadaki adımlarla kurun (apt deposu + ros-lyrical-desktop), sonra:

    sudo apt install ros-lyrical-ros-gz ros-lyrical-xacro \
      ros-lyrical-robot-state-publisher ros-lyrical-joint-state-publisher-gui \
      ros-lyrical-teleop-twist-keyboard python3-colcon-common-extensions

## Derleme ve Çalıştırma
    mkdir -p ~/ws/src && cd ~/ws/src
    git clone https://github.com/Robotik-Projesi/Sera-Projesi.git my_robot_pkg
    cd ~/ws && colcon build --symlink-install
    source install/setup.bash
    ros2 launch my_robot_pkg sim.launch.py

Robotu klavyeyle sürmek için ayrı terminalde:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

