#include "mainwindow.h"
#include <QApplication>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include<std_msgs/Int32MultiArray.h>
#include<std_msgs/Int32.h>
#include<std_msgs/Float64MultiArray.h>
#include <stdlib.h>
#include "ft_sensor.h"



/// ==========================================================================================
/// \brief main
/// \param argc
/// \param argv
/// \return
/// ==========================================================================================
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "ft_node");
    ros::NodeHandle nh;

    ROS_INFO("start app");
    ros::Rate loop_rate(100);
    ft_sensor ft;
    ros::Publisher pub=nh.advertise<std_msgs::Float64MultiArray>("FtSensor",100);
    qDebug()<<"hid connect "<< ft.Init(0xc251,0x1c01);
    QVector<double> data;

std_msgs::Float64MultiArray msg;
std_msgs::MultiArrayDimension msg_dim;
msg_dim.label = "ft_sensors";
msg_dim.size = 1;
msg.layout.dim.clear();
msg.layout.dim.push_back(msg_dim);

    while (ros::ok()) {

        ft.Read(data);
       // qDebug()<<data[0]<<" "<<data[2]<<" "<<data[4]<<" "<<data[6]<<" "<<data[8]<<" "<<data[10]<<" "<<data[1]<<" "<<data[3]<<" "<<data[5]<<" "<<data[7]<<" "<<data[9]<<" "<<data[11];
        msg.data.clear();
       for(int i = 0; i < data.count(); i++)
        msg.data.push_back(data[i]);

        pub.publish(msg);
        //ROS_INFO("time");
           ros::spinOnce();
        //loop_rate.sleep();

    }

    ros::spin();
    return 0;
    // return a.exec();
}
