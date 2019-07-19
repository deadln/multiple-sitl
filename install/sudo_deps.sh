#repos

echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

apt-get update

#gazebo
apt-get install gazebo9 libgazebo9-dev -y

#sitl_gazebo
apt-get install libopencv-dev libeigen3-dev protobuf-compiler libprotobuf-dev libprotoc-dev -y

#px4
apt-get install python-argparse git-core wget zip python-empy cmake build-essential genromfs -y

#start script
apt-get install ruby xterm -y

#clean
apt-get clean
