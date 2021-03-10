pkill gz
pkill gzclient
pkill gzserver
pkill gazebo
pkill roslaunch
pkill roscore
pkill -f "xterm -T mavros"
pkill -f "xterm -T gazebo"
pkill -f "xterm -T gz"
`dirname $0`/kill_px4.sh
