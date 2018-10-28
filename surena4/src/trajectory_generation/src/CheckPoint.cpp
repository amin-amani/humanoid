#include "ros/ros.h"
#include "std_msgs/String.h"
#include"Eigen/Dense"
#include <vector>
#include <iostream>
#include <QString>
#include <QList>
#include "Robot.h"
#include"TaskSpace.h"
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
#include "pidcontroller.h"
#include<rosgraph_msgs/Clock.h>

#include "std_msgs/String.h"
#include<gazebo_msgs/ApplyBodyWrench.h>

#include<gazebo_msgs/ApplyBodyWrenchRequest.h>
#include <gazebo_msgs/GetLinkState.h>
#include<gazebo_msgs/LinkStates.h>
#include <nav_msgs/Odometry.h>

using namespace  std;
using namespace  Eigen;

ros::Publisher pub1  ;
ros::Publisher pub2  ;
ros::Publisher pub3  ;
ros::Publisher pub4  ;
ros::Publisher pub5  ;
ros::Publisher pub6  ;
ros::Publisher pub7  ;
ros::Publisher pub8  ;
ros::Publisher pub9  ;
ros::Publisher pub10 ;
ros::Publisher pub11 ;
ros::Publisher pub12 ;
ros::Publisher pub13 ;
ros::Publisher pub14 ;
ros::Publisher pub15 ;
ros::Publisher pub16 ;
ros::Publisher pub17 ;
ros::Publisher pub18 ;
ros::Publisher pub19 ;
ros::Publisher pub20 ;
ros::Publisher pub21 ;
ros::Publisher pub22 ;
ros::Publisher pub23 ;
ros::Publisher pub24 ;
ros::Publisher pub25 ;
ros::Publisher pub26 ;
ros::Publisher pub27 ;
ros::Publisher pub28 ;

//ros::Subscriber quaternions;
double teta=0;
double phi=0;



double quaternion2ankle_pitch(double q0,double q1,double q2,double q3){
    double R11,R32,R33,R31,theta;
    //    R11=q0*q0+q1*q1-q2*q2-q3*q3;
    R31=2*(q1*q3-q0*q2);
    //    theta=atan2(-R31,R11);
    R32=2*(q0*q1+q2*q3);
    R33=q0*q0-q1*q1-q2*q2+q3*q3;
    theta=atan2(-R31,sqrt(R32*R32+R33*R33));

    return theta;
}
double quaternion2ankle_roll(double q0,double q1,double q2,double q3){
    double R23,R22,phi,R33,R32;
    //R23=2*(q2*q3-q0*q1);
    //R22=q0*q0-q1*q1+q2*q2-q3*q3;
    //phi=atan2(-R23,R22);
    R32=2*(q0*q1+q2*q3);
    R33=q0*q0-q1*q1-q2*q2+q3*q3;
    phi=atan2(R32,R33);
    return phi;
}
geometry_msgs::Pose GetLinkPosition(QString linkName ,const gazebo_msgs::LinkStates::ConstPtr linkStates)
{
  geometry_msgs::Pose result;
const std::vector<std::string> &names = linkStates->name;
const std::vector<geometry_msgs::Pose> positions = linkStates->pose;


for(int i=0;i<names.size();i++)
{


  if(QString::fromStdString( names[i]).contains(linkName))
  {

    return  positions[i];

  }
}

return result;
}
void chatterCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
//const std::vector<std::string> &names = msg->name;
//const std::vector<geometry_msgs::Pose> psitions = msg->pose;
//  double distance=0;

 geometry_msgs::Pose parentPosition=GetLinkPosition("robot::LLeg_Foot_Link",msg);
qDebug()<<"we are here:"<< parentPosition.position.x;
 //qDebug()<<QString::fromStdString( names[9])<<psitions[9].position.x<<psitions[9].position.y<<psitions[9].position.z;
//ROS_INFO("I heard");

}


PIDController teta_pid;
PIDController phi_pid;
double p_teta,i_teta,d_teta,p_phi,i_phi,d_phi,dt,rate;
double teta_motor=0;
double phi_motor=0;
double timestep=.01;
double time_=0;

//teta_pid.Init(0,0,0,0,0,0);
//teta_pid.Init(dt,1,-1,p_teta,i_teta,d_teta);
//phi_pid.Init(dt,1,-1,p_teta,i_teta,d_teta);

void  SendGazebo(){

    std_msgs::Float64 data;
    data.data=0;
    pub1.publish(data);
    pub2.publish(data);
    pub3.publish(data);
    pub4.publish(data);
    pub5.publish(data);
    pub6.publish(data);
    pub7.publish(data);
    pub8.publish(data);
    pub9.publish(data);
    pub10.publish(data);
    pub11.publish(data);
    pub12.publish(data);
    pub13.publish(data);
    pub14.publish(data);
    pub15.publish(data);
    pub16.publish(data);
    pub17.publish(data);
    pub18.publish(data);
    pub19.publish(data);
    pub20.publish(data);
    pub21.publish(data);
    pub22.publish(data);
    pub23.publish(data);
    pub24.publish(data);
    pub25.publish(data);
    pub26.publish(data);
    pub27.publish(data);
    pub28.publish(data);
}

bool imuok=false;
void RecievIMULeft(const sensor_msgs::Imu & msg)
{
    imuok=true;
    ROS_INFO("Left:[%f] [%f] [%f] [%f]",  msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z);
    teta= quaternion2ankle_pitch( msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z);
    phi=quaternion2ankle_roll( msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z);
    SendGazebo();
}

void RecievTime(const rosgraph_msgs::Clock & msg)
{
    time_ =double(msg.clock.nsec)*1e-9 + double(msg.clock.sec);
}


void RecievIMURight(const sensor_msgs::Imu & msg)
{
    ROS_INFO("Right:[%f] [%f] [%f] [%f]", msg.orientation.w, msg.orientation.x,msg.orientation.y,msg.orientation.z);
}

void RecievIMUCenter(const sensor_msgs::Imu & msg)
{
    ROS_INFO("Center:[%f] [%f] [%f] [%f]", msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z);
}

int main(int argc, char **argv)
{
    int count = 0;
    ros::init(argc, argv, "myNode1");
    ros::NodeHandle nh;
   // ros::Subscriber  IMULeft = nh.subscribe("/yei2000154", 1, RecievIMULeft);
    ros::Subscriber  time_sub = nh.subscribe("/clock", 1, RecievTime);
     ros::Subscriber sub = nh.subscribe("/gazebo/link_states", 1, &chatterCallback);

    pub1  = nh.advertise<std_msgs::Float64>("rrbot/joint1_position_controller/command",1);
    pub2  = nh.advertise<std_msgs::Float64>("rrbot/joint2_position_controller/command",1);
    pub3  = nh.advertise<std_msgs::Float64>("rrbot/joint3_position_controller/command",1);
    pub4  = nh.advertise<std_msgs::Float64>("rrbot/joint4_position_controller/command",1);
    pub5  = nh.advertise<std_msgs::Float64>("rrbot/joint5_position_controller/command",1);
    pub6  = nh.advertise<std_msgs::Float64>("rrbot/joint6_position_controller/command",1);
    pub7  = nh.advertise<std_msgs::Float64>("rrbot/joint7_position_controller/command",1);
    pub8  = nh.advertise<std_msgs::Float64>("rrbot/joint8_position_controller/command",1);
    pub9  = nh.advertise<std_msgs::Float64>("rrbot/joint9_position_controller/command",1);
    pub10 = nh.advertise<std_msgs::Float64>("rrbot/joint10_position_controller/command",1);
    pub11 = nh.advertise<std_msgs::Float64>("rrbot/joint11_position_controller/command",1);
    pub12 = nh.advertise<std_msgs::Float64>("rrbot/joint12_position_controller/command",1);
    pub13 = nh.advertise<std_msgs::Float64>("rrbot/joint13_position_controller/command",1);
    pub14 = nh.advertise<std_msgs::Float64>("rrbot/joint14_position_controller/command",1);
    pub15 = nh.advertise<std_msgs::Float64>("rrbot/joint15_position_controller/command",1);
    pub16 = nh.advertise<std_msgs::Float64>("rrbot/joint16_position_controller/command",1);
    pub17 = nh.advertise<std_msgs::Float64>("rrbot/joint17_position_controller/command",1);
    pub18 = nh.advertise<std_msgs::Float64>("rrbot/joint18_position_controller/command",1);
    pub19 = nh.advertise<std_msgs::Float64>("rrbot/joint19_position_controller/command",1);
    pub20 = nh.advertise<std_msgs::Float64>("rrbot/joint20_position_controller/command",1);
    pub21 = nh.advertise<std_msgs::Float64>("rrbot/joint21_position_controller/command",1);
    pub22 = nh.advertise<std_msgs::Float64>("rrbot/joint22_position_controller/command",1);
    pub23 = nh.advertise<std_msgs::Float64>("rrbot/joint23_position_controller/command",1);
    pub24 = nh.advertise<std_msgs::Float64>("rrbot/joint24_position_controller/command",1);
    pub25 = nh.advertise<std_msgs::Float64>("rrbot/joint25_position_controller/command",1);
    pub26 = nh.advertise<std_msgs::Float64>("rrbot/joint26_position_controller/command",1);
    pub27 = nh.advertise<std_msgs::Float64>("rrbot/joint27_position_controller/command",1);
    pub28 = nh.advertise<std_msgs::Float64>("rrbot/joint28_position_controller/command",1);

    rate=100.0;
    ros::Rate loop_rate(rate);
    dt=1/rate;

    std_msgs::Int32MultiArray msg;
    std_msgs::MultiArrayDimension msg_dim;

    msg_dim.label = "joint_position";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);

  //  SendGazebo();

//    while (~imuok){
//        ROS_INFO("No IMU data recieved! %i",IMULeft.getNumPublishers());
//        SendGazebo();
//        ROS_INFO("initialized!");
//        continue;
//    }

    while (ros::ok())
    {
        //ROS_INFO("In loop %i",IMULeft.getNumPublishers());
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}

