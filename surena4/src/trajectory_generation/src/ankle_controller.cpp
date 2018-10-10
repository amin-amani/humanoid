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

ros::Subscriber quaternions;
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

PIDController teta_pid;
PIDController phi_pid;
double p_teta,i_teta,d_teta,p_phi,i_phi,d_phi,dt,rate;
double teta_motor=0;
double phi_motor=0;
double timestep=.01;
double time_;

//teta_pid.Init(0,0,0,0,0,0);
//teta_pid.Init(dt,1,-1,p_teta,i_teta,d_teta);
//phi_pid.Init(dt,1,-1,p_teta,i_teta,d_teta);

void  SendGazebo(double t){

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


    data.data=min(.1*t,1.0);
    pub10.publish(data);

    data.data=-data.data/2;
    pub9.publish(data);

double maximum=.5;
teta_motor=teta_motor+teta_pid.Calculate(0,teta);
    data.data=teta_motor;
    if (data.data<-maximum){data.data=-maximum;}
    if (data.data>maximum){data.data=maximum;}
    pub11.publish(data);
    phi_motor=phi_motor+phi_pid.Calculate(0,phi);
    data.data=phi_motor;
    if (data.data<-maximum){data.data=-maximum;}
    if (data.data>maximum){data.data=maximum;}
    pub12.publish(data);

//    double maximum=.5;
//        data.data=teta;
//        if (data.data<-maximum){data.data=-maximum;}
//        if (data.data>maximum){data.data=maximum;}
//        pub11.publish(data);
//        data.data=phi;
//        if (data.data<-maximum){data.data=-maximum;}
//        if (data.data>maximum){data.data=maximum;}
//        pub12.publish(data);
}





void RecievIMULeft(const sensor_msgs::Imu & msg)
{
    ROS_INFO("Left:[%f] [%f] [%f] [%f]",  msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z);

    teta= quaternion2ankle_pitch( msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z);
    phi=quaternion2ankle_roll( msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z);
    time_=time_+timestep;
    //if (IMULeft.getTopic()=="\0"){ time=0; }

            SendGazebo(time_);

    //DoController(teta,phi);
    //  ROS_INFO("I heard");
}

void RecievIMURight(const sensor_msgs::Imu & msg)
{
    ROS_INFO("Right:[%f] [%f] [%f] [%f]", msg.orientation.w, msg.orientation.x,msg.orientation.y,msg.orientation.z);
    //  ROS_INFO("I heard");
}

void RecievIMUCenter(const sensor_msgs::Imu & msg)
{
    ROS_INFO("Center:[%f] [%f] [%f] [%f]", msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z);
    //  ROS_INFO("I heard");
}

//QVector<double> get_quaternions(const sensor_msgs::Imu & msg){
//    QVector<double> q(4);
//    q(0)=msg.orientation.w;
//    q(1)=msg.orientation.x;
//    q(2)=msg.orientation.y;
//    q(3)=msg.orientation.z;
//    return q;
//}



int main(int argc, char **argv)
{
    //check _timesteps


    //*******************This part of code is for initialization of joints of the robot for walking**********************************
    int count = 0;

    ros::init(argc, argv, "myNode");

    ros::NodeHandle nh;
    ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",10);

    ros::Subscriber  IMULeft = nh.subscribe("/yei2000154", 100, RecievIMULeft);

    //ros::Subscriber  IMURight = nh.subscribe("/yei200015B", 100, RecievIMURight);
    //ros::Subscriber  IMUCenter = nh.subscribe("/mti/sensor/imu", 100, RecievIMUCenter);



    //double theta,phi;
    //theta=quaternion2ankle_pitch(quaternions(0),quaternions(1),quaternions(2),quaternions(3));
    //phi=quaternion2ankle_roll(quaternions(0),quaternions(1),quaternions(2),quaternions(3));

    pub1  = nh.advertise<std_msgs::Float64>("rrbot/joint1_position_controller/command",10);
    pub2  = nh.advertise<std_msgs::Float64>("rrbot/joint2_position_controller/command",10);
    pub3  = nh.advertise<std_msgs::Float64>("rrbot/joint3_position_controller/command",10);
    pub4  = nh.advertise<std_msgs::Float64>("rrbot/joint4_position_controller/command",10);
    pub5  = nh.advertise<std_msgs::Float64>("rrbot/joint5_position_controller/command",10);
    pub6  = nh.advertise<std_msgs::Float64>("rrbot/joint6_position_controller/command",10);
    pub7  = nh.advertise<std_msgs::Float64>("rrbot/joint7_position_controller/command",10);
    pub8  = nh.advertise<std_msgs::Float64>("rrbot/joint8_position_controller/command",10);
    pub9  = nh.advertise<std_msgs::Float64>("rrbot/joint9_position_controller/command",10);
    pub10 = nh.advertise<std_msgs::Float64>("rrbot/joint10_position_controller/command",10);
    pub11 = nh.advertise<std_msgs::Float64>("rrbot/joint11_position_controller/command",10);
    pub12 = nh.advertise<std_msgs::Float64>("rrbot/joint12_position_controller/command",10);
    pub13 = nh.advertise<std_msgs::Float64>("rrbot/joint13_position_controller/command",10);
    pub14 = nh.advertise<std_msgs::Float64>("rrbot/joint14_position_controller/command",10);
    pub15 = nh.advertise<std_msgs::Float64>("rrbot/joint15_position_controller/command",10);
    pub16 = nh.advertise<std_msgs::Float64>("rrbot/joint16_position_controller/command",10);
    pub17 = nh.advertise<std_msgs::Float64>("rrbot/joint17_position_controller/command",10);
    pub18 = nh.advertise<std_msgs::Float64>("rrbot/joint18_position_controller/command",10);
    pub19 = nh.advertise<std_msgs::Float64>("rrbot/joint19_position_controller/command",10);
    pub20 = nh.advertise<std_msgs::Float64>("rrbot/joint20_position_controller/command",10);
    pub21 = nh.advertise<std_msgs::Float64>("rrbot/joint21_position_controller/command",10);
    pub22 = nh.advertise<std_msgs::Float64>("rrbot/joint22_position_controller/command",10);
    pub23 = nh.advertise<std_msgs::Float64>("rrbot/joint23_position_controller/command",10);
    pub24 = nh.advertise<std_msgs::Float64>("rrbot/joint24_position_controller/command",10);
    pub25 = nh.advertise<std_msgs::Float64>("rrbot/joint25_position_controller/command",10);
    pub26 = nh.advertise<std_msgs::Float64>("rrbot/joint26_position_controller/command",10);
    pub27 = nh.advertise<std_msgs::Float64>("rrbot/joint27_position_controller/command",10);
    pub28 = nh.advertise<std_msgs::Float64>("rrbot/joint28_position_controller/command",10);

    rate=100.0;
    ros::Rate loop_rate(rate);
    dt=1/rate;
    p_teta=100;
    p_phi=0;
    i_teta=0;i_phi=0;
    d_teta=0;d_phi=0;
    teta_pid.Init(dt,1,-1,p_teta,i_teta,d_teta);
    phi_pid.Init(dt,1,-1,p_phi,i_phi,d_phi);
time_=0;
    std_msgs::Int32MultiArray msg;
    std_msgs::MultiArrayDimension msg_dim;

    msg_dim.label = "joint_position";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);

    //sensor imu----------------------------------------------sensor imu
    //sensor imu----------------------------------------------sensor imu
    //  timer.start();
    //  ros::Subscriber sub = nh.subscribe("/mti/sensor/imu", 1000, chatterCallback);
    //sensor imu----------------------------------------------sensor imu
    //sensor imu----------------------------------------------sensor imu

SendGazebo(0);
    while (ros::ok())
    {//time=timestep+time;
//if (IMULeft.getTopic()=="\0"){ time=0; }

        //SendGazebo(time);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}

