#ROS

ros_release="melodic"

echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

apt-get update
apt-get install ros-${ros_release}-ros-base -y

#mavros
apt-get install ros-${ros_release}-mavros python-prettytable -y

#catkin
apt-get install python-wstool python-catkin-tools -y

#clean
apt-get clean

#init
rosdep init
