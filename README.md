# Sera Olgunluk Tarayıcısı Robotu

## Ekip
    |      Üye       | Öğrenci No |               Görev                |
    |----------------|------------|------------------------------------|
    | Hacer Cansevdi | 1306220064 | Lidar, robot gövdesi, launch       |
    | Edanur Çınar   | 1306220037 | Kamera, sera dünyası ,domates sayımı|
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

`sim.launch.py` içindeki `behavior_node` robotu otomatik sürer (`/cmd_vel`'in tek yayıncısı).
`lidar_standalone_tester` yalnızca LiDAR'ı tek başına denemek içindir; `behavior_node` ile
aynı anda çalıştırmayın (iki yayıncı `/cmd_vel`'i çakıştırır).

## Topic'ler
| Topic | Tip | Yayınlayan | Dinleyen |
|-------|-----|------------|----------|
| `/scan` | LaserScan | Gazebo (LiDAR) | behavior_node |
| `/odom` | Odometry | Gazebo | behavior_node |
| `/camera/image_raw` | Image | Gazebo (kamera) | tomato_counter |
| `/imu` | Imu | Gazebo | behavior_node |
| `/bumper` | ros_gz_interfaces/Contacts | Gazebo | behavior_node |
| `/ripe_count` | Int32 | tomato_counter | izleme |
| `/behavior_state` | String | behavior_node | izleme |
| `/cmd_vel` | Twist | yalnızca behavior_node | Gazebo |

## Davranış Mantığı
`behavior_node` her turda şu sırayla bakar, ilk komut veren kazanır:
1. `bumper_recovery`: çarpma olursa geri çekilir, sonra LiDAR şerit takibine yanal ofset verir
2. `tilt_guard`: IMU eğimi eşiği aşarsa durur; eğimde yavaşlatır (rampada durdurmaz)
3. `lidar_navigation`: şerit takibi ve engelden kaçış
4. varsayılan: ileri git

Eşikler `config/behavior_params.yaml` içindedir. Aktif davranışı canlı görmek için:

    ros2 topic echo /behavior_state

