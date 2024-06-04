#!/bin/bash
if [ $# -ge 1 ]; then
    if [ "$1" = "master" ]; then
        MASTER_IP_ADDRESS=$2
        export ROS_MASTER_URI=http://${MASTER_IP_ADDRESS}:11311
        export ROS_IP=${MASTER_IP_ADDRESS}
        
    elif [ "$1" = "client" ]; then
        MASTER_IP_ADDRESS=$2
        CLIENT_IP_ADDRESS=$3
        export ROS_MASTER_URI=http://${MASTER_IP_ADDRESS}:11311
        export ROS_IP=${CLIENT_IP_ADDRESS}
    fi
fi

echo "ROS_MASTER_URI set to: $ROS_MASTER_URI"
echo "ROS_IP set to: $ROS_IP"