pkill gzclient
pkill gzserver
pkill gazebo
pkill roslaunch
pkill -f "xterm -T mavros"
pkill -f "xterm -T gazebo"
`dirname $0`/kill_px4.sh
