#!/bin/sh

ls /dev/ttyACM*
if [ $? -eq 1 ]; then
echo "error yei"
exit 0
else
echo "yei port exist"
fi

OUTPUT=$(hostname -I) 
echo "${OUTPUT}" 
 
export ROS_IP=`hostname -I`
 . /home/milad/humanoid/surena4/devel/setup.sh
rosrun yei_imu yei_imu ttyACM0 &
rosrun yei_imu yei_imu ttyACM1 &
rosrun yei_imu yei_imu ttyACM2 &
rosrun yei_imu yei_imu ttyACM3 &
rosrun surena_usb surena_usb &
roslaunch xsens_driver xsens.launch &

read -p " " domain_name
rosnode kill --all


