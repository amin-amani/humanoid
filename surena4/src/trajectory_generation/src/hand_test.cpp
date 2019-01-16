#include "ros/ros.h"
#include "std_msgs/String.h"
#include"Eigen/Dense"
#include <vector>
#include <iostream>
#include <QString>
#include <QList>
#include "Robot.h"
//#include"TaskSpace.h"
#include"taskspaceoffline.h"
#include <qmath.h>
#include <cstring>
#include<qdebug.h>
#include <Eigen/Geometry>
#include <cstdlib>
//#include <link.h>
#include "Eigen/eiquadprog.h"
#include "Eigen/Core"
#include "Eigen/Cholesky"
#include "Eigen/LU"
#include<std_msgs/Int32MultiArray.h>
#include<std_msgs/Float32MultiArray.h>
#include<math.h>
#include<sensor_msgs/Imu.h>
#include<std_msgs/Float64.h>
#include "qcgenerator.h"
#include <gazebo_msgs/LinkStates.h>


using namespace  std;

int main(int argc, char **argv)
{

int count = 0;
double t=0;
double dt=.005;

ros::init(argc, argv, "myNode");
ros::NodeHandle nh;
ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",1000);
ros::Rate loop_rate(200);

std_msgs::Int32MultiArray msg;
std_msgs::MultiArrayDimension msg_dim;

msg_dim.label = "joint_position";
msg_dim.size = 1;
msg.layout.dim.clear();
msg.layout.dim.push_back(msg_dim);


while (ros::ok())
{

 msg.data.clear();
int p;
p=int(1000*sin(t*2*M_PI/4));

for(int  i = 0;i < 12;i++)
{
    msg.data.push_back(p);
}

int q;
q=int(80*sin(t*2*M_PI/4));

for(int i= 12;i < 16;i++){

    msg.data.push_back(q);
}

for(int  i = 16;i < 20;i++)
{
    msg.data.push_back(0);
}

for(int i= 20;i < 24;i++){

    msg.data.push_back(q);
}

for(int  i = 24;i < 28;i++)
{
    msg.data.push_back(0);
}

chatter_pub.publish(msg);


ros::spinOnce();
loop_rate.sleep();
++count;
t+=dt;
}


return 0;
}
