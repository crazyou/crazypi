#!/bin/bash
if [ "$1" != "" ];then
    APP_ROOT=$1
else
   APP_ROOT=/home/crazypi/apps
fi

if [ "$2" != "" ];then
    DID=$2
else
    DID=0ffffffff
fi

APP_DIR=$APP_ROOT
LOG_DIR=$APP_DIR/logs/
DAEMON_SH=$APP_DIR/daemon.sh
source /opt/ros/kinetic/setup.bash

#itest=$(fping 120.76.126.248| grep alive)
#while [ "$itest" == "" ] 
#do
#        sleep 5
#        itest=$(fping 120.76.126.248| grep alive)
#done
#echo "Online"

hot_ip=192.168.99.1
ros_ip=$hot_ip
cip=""

pushd $APP_DIR/sound
echo "Start Crazy Daemon...."
bash $DAEMON_SH "voiceRecog" "./voiceRecog" $LOG_DIR start > $LOG_DIR/voiceRcog.log 2>&1 &
popd
while true
do
    cip=$(ifconfig  | grep 'inet addr:'| grep -v '127.0.0.1' | cut -d: -f2 | awk '{ print $1}')
    if [ "$cip" != "" ] && [ "$cip" != "$ros_ip" ];then
        ros_ip=$cip
        export ROS_IP=$cip
        echo "current ip :"
        echo $cip

        ps -ef | grep "ros"| grep -v  "grep" | awk '{print $2}' | xargs kill -9
        ps -ef | grep "node_bridge"| grep -v  "grep" | awk '{print $2}' | xargs sudo  kill -9
        ps -ef | grep "call_node_arm"| grep -v  "grep" | awk '{print $2}' | xargs sudo  kill -9
        ps -ef | grep "robot_ctrl_node"| grep -v  "grep" | awk '{print $2}' | xargs sudo  kill -9
        ps -ef | grep "node_lidar"| grep -v  "grep" | awk '{print $2}' | xargs sudo  kill -9
        ps -ef | grep "monitor_btn"| grep -v  "grep" | awk '{print $2}' | xargs sudo  kill -9
        
        /usr/bin/python /opt/ros/kinetic/bin/roslaunch  $APP_DIR/slam/slam_tutorial.launch > $LOG_DIR/slam.log 2>&1 &

        pushd $APP_DIR/bridge
        echo "Start Bridge...."
        bash $DAEMON_SH "node_bridge" "python ./node_bridge.py" $LOG_DIR start > $LOG_DIR/node_bridge.log 2>&1 &
        popd

        pushd $APP_DIR/call
        echo "Start Call...."
        bash $DAEMON_SH "node_call" "./call_node_arm" $LOG_DIR start > $LOG_DIR/start_call.log 2>&1 &
        popd

        pushd $APP_DIR/robot_ctrl
        echo "Start Crazy Daemon...."
        bash $DAEMON_SH "robot_ctrl_node" "./robot_ctrl_node" $LOG_DIR start > $LOG_DIR/robot_ctrl.log 2>&1 &
        popd

#        pushd $APP_DIR/terminal
#        echo "Start Crazy Daemon...."
#        bash $DAEMON_SH "terminal_node" "./terminal_node" $LOG_DIR start > $LOG_DIR/start_terminal.log 2>&1 &
#        popd

        pushd $APP_DIR/lidar
        echo "Start Crazy Daemon...."
        bash $DAEMON_SH "node_lidar" "./node_lidar" $LOG_DIR start > $LOG_DIR/lidar.log 2>&1 &
        popd

        pushd $APP_DIR/btn
        echo "Start Crazy Daemon...."
        bash $DAEMON_SH "monitor_btn" "python ./monitor_btn.py" $LOG_DIR start > $LOG_DIR/start_monitor_btn.log 2>&1 &
        popd

        rosrun robot_pose_publisher robot_pose_publisher &

        sleep 1
    else
        #echo "ip no change"
        sleep 1
    fi
done

#source /home/crazypi/catkin_ws/devel/setup.bash
#rosrun odom2base_link tf_broadcast &
#/home/crazypi/temp_benny/nav_config/slam_tutorial.launch &
#/home/crazypi/temp_benny/nav_config/hecotr_navigation/nav.launch &

#bash $DAEMON_SH "rplidarNode_usb0_default_stop" "./rplidarNode_usb0_default_stop" $LOG_DIR start > $LOG_DIR/lidar.log 2>&1 &
#bash $DAEMON_SH "rplidarNode" "./rplidarNode" $LOG_DIR start > $LOG_DIR/lidar.log 2>&1 &
 
#roscore > $LOG_DIR/roscore.log 2>&1 &
#/usr/bin/python /opt/ros/kinetic/bin/roslaunch  /opt/ros/kinetic/share/hector_slam_launch/launch/bred.launch > /home/crazypi/temp.log 2>&1 &
#/usr/bin/python /opt/ros/kinetic/bin/roslaunch  /home/crazypi/temp_benny/nav_config/slam_tutorial.launch > /home/crazypi/temp.log 2>&1 &


