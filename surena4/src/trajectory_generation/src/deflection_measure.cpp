#include "ros/ros.h"
#include "std_msgs/String.h"
#include"Eigen/Dense"
#include <vector>
#include <iostream>
#include <QString>
#include <QList>
#include <QTime>
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
#include <sensor_msgs/JointState.h>

double AbsEncoder[13];
double IncEncoder[13];

void abs_enc_receive(const  sensor_msgs::JointState &msg)
{
    for (int i = 0; i < 13; ++i) {
        AbsEncoder[i]=msg.position[i];
    }
}

void inc_enc_receive(const  sensor_msgs::JointState &msg)
{
    for (int i = 0; i < 13; ++i) {
       IncEncoder[i]=msg.position[i];
    }
}



int main(int argc, char *argv[])
{

    QByteArray data;


    QTime pc_time;
    QString name;
    name="/home/cast/Desktop/deflection_data_save/data_";
    name+=QString::number(pc_time.currentTime().hour())+"_";
    name+=QString::number(pc_time.currentTime().minute())+"_";
    name+=QString::number(pc_time.currentTime().second())+"_";
    name+=".txt";
    QFile myfile(name);

    vector<double> cntrl(13);
    QCgenerator QC;
    //check _timesteps



    ros::init(argc, argv, "myNode");

    ros::NodeHandle nh;
    ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",1000);
//    ros::Subscriber abs_enc = nh.subscribe("surena/abs_joint_state", 100,abs_enc_receive );
//    ros::Subscriber inc_enc = nh.subscribe("surena/inc_joint_state", 100,inc_enc_receive );

    ros::Rate loop_rate(200);
    std_msgs::Int32MultiArray msg;
    std_msgs::MultiArrayDimension msg_dim;

    msg_dim.label = "joint_position";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);


    double dt=.005;
    double time;

    double  d_theta=dt*M_PI/2/10;
    double hip_roll_angle=0;
    bool increase=true;
    while (ros::ok())
    {
        ROS_INFO("hip_roll_angle=%f",hip_roll_angle);

        if(increase){hip_roll_angle+=d_theta;}
        if(time<2){hip_roll_angle=0;}
        if(hip_roll_angle>M_PI/9){hip_roll_angle=M_PI/9;}
//        else{hip_roll_angle-=d_theta;}
//        if(hip_roll_angle>M_PI/3 || hip_roll_angle<0){increase=!increase;}

        //commands in radian
        cntrl[0]=0.0;
        cntrl[1]=0.0;//RHY
        cntrl[2]=0.0-hip_roll_angle;//RHR(<0 to avoid collision)
        cntrl[3]=0.0;//RHP
        cntrl[4]=0.0;//RK
        cntrl[5]=0.0;//RAP
        cntrl[6]=0.0;//RAR
        cntrl[7]=0.0;//LHY
        cntrl[8]=0.0;//LHR(>0 to avoid collision)
        cntrl[9]=0.0;//LHP
        cntrl[10]=0.0;//LK
        cntrl[11]=0.0;//LAP
        cntrl[12]=0.0;//LAR



    vector<int> qref(12);
    qref=QC.ctrldata2qc(cntrl);

msg.data.clear();
    for(int  i = 0;i < 12;i++)
    {
        msg.data.push_back(qref[i]);
        //msg.data.push_back(0);
    }


    chatter_pub.publish(msg);
    //  ROS_INFO("t={%d} c={%d}",timer.elapsed(),count);
    time+=dt;
//    data.append(QString::number(time)+","+QString::number(hip_roll_angle)+","+QString::number(AbsEncoder[9+1])+","+QString::number(IncEncoder[9+1])+"\n");
//    myfile.open(QFile::ReadWrite);
//    myfile.write(data);
//    myfile.close();

    ros::spinOnce();
    loop_rate.sleep();



}



    return 0;
}

