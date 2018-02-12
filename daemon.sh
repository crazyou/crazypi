#!/bin/bash
source /opt/ros/kinetic/setup.bash

NAME=$1
CMD=$2
LOG_ROOT=$3
EXE_TYPE=$4

LOG_DIR=$LOG_ROOT/$NAME
mkdir -p $LOG_DIR
LOG_NAME=$LOG_DIR/$NAME".log"
DAEMON_LOG_NAME=$LOG_DIR/$NAME"_daemon.log"
PID_FILE=$LOG_DIR/$NAME".pid"
SHELL_PID_FILE=$LOG_DIR/$NAME"_daemon.pid"
echo $$ > $SHELL_PID_FILE

function start {
    $CMD > $LOG_NAME 2>&1 &
    CMD_PID=$!
    echo $CMD_PID > $PID_FILE
    echo "start $CMD [ok]"
    echo "`date '+%Y/%m/%d %H:%M:%S'` START" >> $DAEMON_LOG_NAME
}

function stop {
	ps_pid=`ps -ef | grep $NAME | grep -v grep | awk '{print $2}'`
	if [ -f $PID_FILE ];then
		file_pid=`cat $PID_FILE` 
		sudo kill -9 $file_pid
		sudo rm $PID_FILE
		echo "stop $CMD, kill $ps_pid"
		echo "`date '+%Y/%m/%d %H:%M:%S'` STOP" >> $DAEMON_LOG_NAME
	elif [ "$ps_pid" != "" ];then
		sudo kill -9 $ps_pid
		echo "stop $CMD, kill $ps_pid"
		echo "`date '+%Y/%m/%d %H:%M:%S'` STOP PS" >> $DAEMON_LOG_NAME
	else
		echo "`date '+%Y/%m/%d %H:%M:%S'` STOP FILED, NOT STARTED." >> $DAEMON_LOG_NAME
	fi
}

function restart {
    if [ -f $PID_FILE ];then
        stop
        sleep 1
    fi
    start
    echo "`date '+%Y/%m/%d %H:%M:%S'` RESTART" >> $DAEMON_LOG_NAME
}

case "$EXE_TYPE" in
start)
    start
;;
restart)
    restart
;;
stop)
    stop
    exit 0
;;
esac

#check daemon
for ((c=0;;c++))
do
    if [ -f $PID_FILE ];then
        pid=`cat $PID_FILE`
        psrtn=`ps uh -p $pid`
        if [ -z "$psrtn" ];then
            #need restart
            echo "`date '+%Y/%m/%d %H:%M:%S'` FATALERROR RESTART SERVICE" >> $DAEMON_LOG_NAME
            start
        elif((c%20 == 0 ));then
            #record runtime info
            echo "`date '+%Y/%m/%d %H:%M:%S'` PSINFO $psrtn" >> $DAEMON_LOG_NAME
            c=0
        fi
    	sleep 5
    else
        echo "`date '+%Y/%m/%d %H:%M:%S'` DAEMON SCRIPT EXIT" >> $DAEMON_LOG_NAME
	break;
    fi
done
