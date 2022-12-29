# installs 
https://cyberbotics.com/doc/guide/installation-procedure

wget -qO- https://cyberbotics.com/Cyberbotics.asc | sudo apt-key add -
sudo apt-add-repository 'deb https://cyberbotics.com/debian/ binary-amd64/'
sudo apt update


# besoin d'une version specifique de R2022b https://github.com/cyberbotics/webots/releases/tag/R2022a
wget https://github.com/cyberbotics/webots/releases/download/R2022a/webots_2022a_amd64.deb
sudo apt install ./webots_2022a_amd64.deb
<!--> sudo apt-get install webots:R2022b-ubuntu20.04 <!-->
<!-->docker run -it cyberbotics/webots:R2022b-ubuntu20.04<!-->

## test sous webots

### rajouter un objet 3D :
* creer un modele cadshape
* dans le champs url rajouter le obj ou dae (apres avoir exporte le materiel en mode copie) et l'avoir verifié sous https://3dviewer.net/
https://cyberbotics.com/doc/reference/cadshape

### essayer de simuler un lidar 
sudo apt install ros-rolling-webots-ros2-epuck
sudo apt install ros-rolling-webots-ros2-msgs
sudo ap install ros-rolling-webots-ros2-tiago
sudo apt install ros-rolling-webots-ros2-mavic
## sourcing ros 2 source /opt/ros/rolling/setup.bash
source /opt/ros/rolling/setup.bash
ros2 launch /opt/ros/rolling/share/webots_ros2_epuck/launch/robot_launch.py
/opt/ros/rolling/lib/python3.8/site-packages/webots_ros2_epuck

ros2 launch /opt/ros/rolling/share/webots_ros2_tiago/launch/robot_launch.py
# sinsiper de https://github.com/cyberbotics/webots_ros2/wiki/Tutorial-E-puck-for-ROS2-Beginners


# faire un plugin specifique ros2 
https://github.com/cyberbotics/webots_ros2/wiki/Tutorial-Creating-a-Custom-Python-Plugin
https://github.com/cyberbotics/webots_ros2/blob/master/webots_ros2_mavic/webots_ros2_mavic/mavic_driver.py
/opt/ros/rolling/lib/python3.8/site-packages/webots_ros2_mavic

## ce sur qu'oi sinspirer 
https://github.com/cyberbotics/webots_ros2
#prendre un example (du epuck par exemple, essayer de lexecuter)


# install ros 2 from https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html
cd ~/webotsRA
mkdir -p ros2/ros2_custom
cd ros2/ros2_custom



* source ros
source /opt/ros/rolling/setup.bash
sudo apt install ros-rolling-webots-ros2-driver
sudo apt install ros-rolling-webots-ros2-universal-robot

<!-->
ros2 pkg create --build-type ament_python --node-name robot_interface robot_interface
https://github.com/cyberbotics/webots_ros2/wiki/Tutorial-Creating-a-Custom-Package

<!-->

cd 
git clone https://github.com/cyberbotics/webots_ros2.git
cd webotsRA
cp -rf /home/user/webots_ros2/webots_ros2_turtlebot ./webots_ros2_custom
cd webots_ros2_custom
colcon build

ros2 launch ./launch/robot_launch.py

## https://github.com/cyberbotics/webots_ros2/wiki/Linux-Installation-Guide
<!-->
mkdir -p ./ros2_ws/src
source /opt/ros/rolling/setup.bash
cd ros2_ws
git clone --recurse-submodules https://github.com/cyberbotics/webots_ros2.git src/webots_ros2
sudo apt install python3-pip python3-rosdep python3-colcon-common-extensions
sudo rosdep init && rosdep update
sudo apt install ros-rolling-ackermann-msgs
sudo apt install ros-rolling-xacro
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# Building packages
colcon build

#PYTHON_LIBRARY='/usr/lib/python3.9/config-3.9-aarch64-linux-gnu/libpython3.9.so' PYTHON_INCLUDE_DIR='/usr/lib/python3.9'
#PYTHON_LIBRARY='/usr/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so' 
#PYTHON_INCLUDE_DIR='/usr/lib/python3.8'
# Source this workspace (careful when also sourcing others)
source install/local_setup.bash

# To start a specific Webots installation, set WEBOTS_HOME
export WEBOTS_HOME=/usr/local/webots
# If installed from sources, source the ROS2 workspace
source install/local_setup.bash
# Start demo package (e.g. webots_ros2_epuck)

ros2 launch webots_ros2_tiago robot_launch.py 
ros2 launch webots_ros2_epuck robot_launch.py

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
<!-->

cd /home/user/webotsRA/webots_ros2_custom
source /opt/ros/rolling/setup.bash
export WEBOTS_HOME=/usr/local/webots
source install/local_setup.bash
cd ..

ros2 launch webots_ros2_custom robot_launch.py
<!-->
ros2 launch  /opt/ros/rolling/share/webots_ros2_turtlebot/launch/robot_launch.py
<!-->

export WEBOTS_HOME=/home/username/webots
source /opt/ros/rolling/setup.bash
#ros2 launch webots_ros2_tiago robot_launch.py rviz:=true slam:=true
ros2 launch webots_ros2_tiago robot_launch.py rviz:=true slam:=false
ros2 launch webots_ros2_tiago robot_launch.py rviz:=false slam:=false

source /opt/ros/rolling/setup.bash
#ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"

# 

#essai a partir d'un docker :
## https://cyberbotics.com/doc/guide/installation-procedure#installing-the-docker-image
docker run -it cyberbotics/webots:latest
<!-->docker run -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw cyberbotics/webots:latest webots<!-->

xhost +local:root > /dev/null 2>&1
<!-->docker run -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw cyberbotics/webots:latest<!-->
docker run --gpus=all -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw cyberbotics/webots:latest

# install ros rolling https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html
*puis sourcing du ros : source /opt/ros/rolling/setup.bash

# install de webot ros https://github.com/cyberbotics/webots_ros2/wiki/Linux-Installation-Guide
mkdir -p ~/ros2_ws/src
git clone --recurse-submodules https://github.com/cyberbotics/webots_ros2.git src/webots_ros2
apt install python3-pip python3-rosdep python3-colcon-common-extensions
rosdep init 
rosdep update

apt install ros-rolling-ackermann-msgs
apt install ros-rolling-xacro
apt install ros-rolling-controller-manager
apt install ros-rolling-vision-msgs
apt install ros-rolling-joint-state-broadcaster
apt install ros-rolling-joint-trajectory-controller
apt install ros-rolling-diff-drive-controller
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO

colcon build

nano -w src/webots_ros2/webots_ros2_driver/CMakeLists.txt et forcer la python3.8
<!-->
--- stderr: webots_ros2_driver                           
CMake Error at /usr/share/cmake-3.16/Modules/FindPackageHandleStandardArgs.cmake:146 (message):
  Could NOT find PythonLibs (missing: PYTHON_LIBRARIES PYTHON_INCLUDE_DIRS)
  (Required is exact version "3.10")

https://docs.ros.org/en/rolling/Tutorials/Advanced/Simulators/Webots.html
<!-->
# install python3.10 https://computingforgeeks.com/how-to-install-python-on-ubuntu-linux-system/
sudo apt install software-properties-common -y
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt install python3.10

rm /usr/bin/python3 # a logirine pointe cers 3.8
ln -s /usr/bin/python3.10 /usr/bin/python3

cd webots_ros2/
git branch 2022.1.4

# lient super interessant https://docs.ros.org/en/rolling/Tutorials/Advanced/Simulators/Webots.html
source /opt/ros/rolling/setup.bash
sudo apt install ros-rolling-webots-ros2-driver
apt install ros-rolling-webots-ros2-driver-dbgsym
cd ~/webotsRA
mkdir -p ros2_ws/src
ros2 pkg create --build-type ament_python --node-name my_robot_driver my_package --dependencies rclpy geometry_msgs webots_ros2_driver
cd my_package
mkdir launch
mkdir worlds
* recupration d'un worlds
cd worlds
wget https://docs.ros.org/en/rolling/_downloads/c0d343388f672b7ea879394d1f179ca1/my_world.wbt
<!--> create a robot https://github.com/cyberbotics/webots_ros2/wiki/Tutorial-Create-Webots-Robot <!-->
cd ..

cd ~/webotsRA/ros2_ws/src
colcon build
source install/local_setup.bash
ros2 launch my_package robot_launch.py

<!-->
export WEBOTS_HOME=/usr/local/webots
export PYTHONIOENCODING=UTF-8
export PYTHONPATH=$PYTHONPATH:$WEBOTS_HOME/lib/controller/python38
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$WEBOTS_HOME/lib/controller

/opt/ros/rolling/lib/python3.8/site-packages/webots_ros2_driver ne contient pas 
cannot import name 'Ros2SupervisorLauncher' from 'webots_ros2_driver.webots_launcher' (/opt/ros/rolling/lib/python3.8/site-packages/webots_ros2_driver/webots_launcher.py)

<!-->

# comme ca marche pas, essauer ros-foxy-webots-ros2-tests ou ros-galactic-webots-ros2
https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
sudo apt install ros-foxy-desktop python3-argcomplete
sudo apt install ros-foxy-webots-ros2-driver

source /opt/ros/foxy/setup.bash

cd ~/webotsRA/ros2_ws/src
colcon build
<!-->--symlink-install<!-->
source install/local_setup.bash
ros2 launch my_package robot_launch.py


#install dans /home/user/.ros

