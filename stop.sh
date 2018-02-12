#!/bin/bash
if [ "$1" != "" ];then
    APP_ROOT=$1
else
    APP_ROOT=/home/CrazyPi/apps
fi

if [ "$2" != "" ];then
    DID=$2
else
    DID=0ffffffff
fi

APP_DIR=$APP_ROOT
LOG_DIR=$APP_DIR/logs/
DAEMON_SH=$APP_DIR/daemon.sh

pushd $APP_DIR/bridge
echo "Start Bridge...."
bash $DAEMON_SH "node_crazyou_bridge" "python ./node_crazyou_bridge.py" $DID $LOG_DIR stop > $LOG_DIR/start_bridge.log 2>&1 &
popd

pushd $APP_DIR/call
echo "Start Call...."
bash $DAEMON_SH "call" "./call_node_arm" $DID $LOG_DIR stop > $LOG_DIR/start_call.log 2>&1 &
popd

pushd $APP_DIR/crazd
echo "Start Crazy Daemon...."
bash $DAEMON_SH "crazd" "./crazd" $DID $LOG_DIR stop > $LOG_DIR/start_crazd.log 2>&1 &
popd

pushd $APP_DIR/robot_ctrl
echo "Start Crazy Daemon...."
bash $DAEMON_SH "robot_ctrl_node" "./robot_ctrl_node" $DID $LOG_DIR stop > $LOG_DIR/start_crazd.log 2>&1 &
popd

pushd $APP_DIR/lidar
echo "Start Crazy Daemon...."
bash $DAEMON_SH "rplidarNode_usb0_default_stop" "./rplidarNode_usb0_default_stop" $DID $LOG_DIR stop > $LOG_DIR/lidar.log 2>&1 &
popd


