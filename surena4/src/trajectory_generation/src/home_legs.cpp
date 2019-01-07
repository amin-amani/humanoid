#include "ros/ros.h"
#include "std_msgs/String.h"
#include"Eigen/Dense"
#include <vector>
#include <iostream>
#include <QString>
#include <QList>
#include <QThread>
#include "Robot.h"
#include"taskspaceoffline.h"
#include <qmath.h>
#include <cstring>
#include<qdebug.h>
#include <Eigen/Geometry>
#include <cstdlib>
#include<std_msgs/Float64.h>
#include "qcgenerator.h"
#include "Eigen/eiquadprog.h"
#include "Eigen/Core"
#include "Eigen/Cholesky"
#include "Eigen/LU"
#include<std_msgs/Int32MultiArray.h>
#include<std_msgs/Float32MultiArray.h>
#include<math.h>
#include<sensor_msgs/Imu.h>
#include<sensor_msgs/JointState.h>
#include<geometry_msgs/WrenchStamped.h>
#include<rosgraph_msgs/Clock.h>
#include<dynamic_reconfigure/Config.h>
#include<gazebo_msgs/ApplyBodyWrenchRequest.h>
#include <gazebo_msgs/GetLinkState.h>
#include<gazebo_msgs/LinkStates.h>
#include <nav_msgs/Odometry.h>

using namespace  std;
using namespace  Eigen;

int32_t incEncoder[13];
double AbsEncoder[13];
int32_t diffs[12];

void abs_enc_receive(const  sensor_msgs::JointState &msg)
{
    for (int i = 0; i < 13; ++i) {
        AbsEncoder[i]=msg.position[i];
    }
}

void inc_enc_receive(const  sensor_msgs::JointState &msg)
{
    for (int i = 0; i < 13; ++i) {
        incEncoder[i]=msg.position[i];
    }
}






int main(int argc, char **argv)
{
    QMap<QString,int> MotorNames;
    QMap<int,int> HomeOrder;
    int MotorOffset[12];
    double GearRatio[12];
    ///init robot motors map
    MotorNames.insert("R1",0);
    MotorNames.insert("R2",1);
    MotorNames.insert("R3",2);
    MotorNames.insert("R4",3);
    MotorNames.insert("R5",9);
    MotorNames.insert("R6",8);

    MotorNames.insert("L1",5);
    MotorNames.insert("L2",4);
    MotorNames.insert("L3",6);
    MotorNames.insert("L4",7);
    MotorNames.insert("L5",10);
    MotorNames.insert("L6",11);
    MotorNames.insert("ALL",255);

    HomeOrder.insert(0,0);
    HomeOrder.insert(1,1);
    HomeOrder.insert(2,2);
    HomeOrder.insert(3,3);
    HomeOrder.insert(4,8);
    HomeOrder.insert(5,9);
    HomeOrder.insert(6,5);
    HomeOrder.insert(7,4);
    HomeOrder.insert(8,6);
    HomeOrder.insert(9,7);
    HomeOrder.insert(10,10);
    HomeOrder.insert(11,11);

     QMap<int,double> Direction;
    Direction.insert(0,1);
    Direction.insert(1,-1);
    Direction.insert(2,1);
    Direction.insert(3,-1);
    Direction.insert(4,1);
    Direction.insert(5,1);
    Direction.insert(6,-1);
    Direction.insert(7,1);
    Direction.insert(8,-1);
    Direction.insert(9,1);
    Direction.insert(10,1);
    Direction.insert(11,-1);




    // double GearRatio[12]={28.125,28.125,14.0625,22.5,28.125,28.125,14.0625,-22.5,33.75,33.75,33.75,33.75};
    GearRatio[MotorNames["R1"]]=56.25;
    GearRatio[MotorNames["R2"]]=56.25;
    GearRatio[MotorNames["R3"]]=28.125;
    GearRatio[MotorNames["R4"]]=45;
    GearRatio[MotorNames["R5"]]=67.5;
    GearRatio[MotorNames["R6"]]=67.5;
    GearRatio[MotorNames["L1"]]=56.25;
    GearRatio[MotorNames["L2"]]=56.25;
    GearRatio[MotorNames["L3"]]=28.125;
    GearRatio[MotorNames["L4"]]=45;
    GearRatio[MotorNames["L5"]]=67.5;
    GearRatio[MotorNames["L6"]]=67.5;



    MotorOffset[MotorNames["R1"]]=-42;
    MotorOffset[MotorNames["R2"]]=-77;
    MotorOffset[MotorNames["R3"]]=156;
    MotorOffset[MotorNames["R4"]]=0;//104;
    MotorOffset[MotorNames["R5"]]=16;
    MotorOffset[MotorNames["R6"]]=-149;
    MotorOffset[MotorNames["L1"]]=-127;
    MotorOffset[MotorNames["L2"]]=337;
    MotorOffset[MotorNames["L3"]]=-1;
    MotorOffset[MotorNames["L4"]]=203;
    MotorOffset[MotorNames["L5"]]=-970;
    MotorOffset[MotorNames["L6"]]=143;

    for (int i = 0; i < 12; ++i) {
        AbsEncoder[i+1]=MotorOffset[i];
    }





    vector<double> cntrl(13);
    int count = 0;
    ros::init(argc, argv, "myNode");

    ros::NodeHandle nh;
    ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",1000);

    ros::Subscriber abs_enc = nh.subscribe("surena/abs_joint_state", 100,abs_enc_receive );
    ros::Subscriber inc_enc = nh.subscribe("surena/inc_joint_state", 100,inc_enc_receive );





    ros::Rate loop_rate(200);
    std_msgs::Int32MultiArray msg;
    std_msgs::MultiArrayDimension msg_dim;

    msg_dim.label = "joint_position";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);

    //************************ massage must be sent to
    //    for (int var = 0; var < 100; ++var) {


    //    msg.data.clear();
    //    for(int  k = 0;k < 12;k++)
    //    {
    //        msg.data.push_back(10);
    //    }

    //    chatter_pub.publish(msg);
    //    ros::spinOnce();
    //   loop_rate.sleep();
    // }
    //*************************


    while (inc_enc.getNumPublishers()==0) {
        ROS_INFO("Robot data is not available!");
        QThread::msleep(50);
    }


    for (int i = 0; i < 12; ++i) {
        diffs[i]=incEncoder[i+1];
    }

    double max=20.0;
    double kp=1000;//230400/2/3.14/2;
    int d=100;
    int start=0;
    while(ros::ok()){

        if(start<5){

            msg.data.clear();
            for(int  k = 0;k < 12;k++)
            {
                msg.data.push_back(0);
                //msg.data.push_back(100);
            }

            chatter_pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();

            start++;
        }



        else{


            for (int i = 0; i <12; ++i) {

                int j=HomeOrder[i];

                while (abs(-AbsEncoder[j+1])>M_PI/4096) {
                    //while (true) {

                    if(abs((-AbsEncoder[j+1])*kp*Direction[j])<=max){diffs[j]+= (-AbsEncoder[j+1])*kp*Direction[j];}
                    if(((-AbsEncoder[j+1])*kp*Direction[j])<-max){diffs[j]-=(max);}
                    if(((-AbsEncoder[j+1])*kp*Direction[j])>max){diffs[j]+=(max);}
                    ROS_INFO("abs:R1=%d,R2=%d,R3=%d,R4=%d,L2=%d,L1=%d,L3=%d,L4=%d,R6=%d,R5=%d,L5=%d,L6=%d",int(AbsEncoder[1]*4096/2/M_PI),int(AbsEncoder[2]*4096/2/M_PI),int(AbsEncoder[3]*4096/2/M_PI),int(AbsEncoder[4]*4096/2/M_PI),int(AbsEncoder[5]*4096/2/M_PI),int(AbsEncoder[6]*4096/2/M_PI),int(AbsEncoder[7]*4096/2/M_PI),int(AbsEncoder[8]*4096/2/M_PI),int(AbsEncoder[9]*4096/2/M_PI),int(AbsEncoder[10]*4096/2/M_PI),int(AbsEncoder[11]*4096/2/M_PI),int(AbsEncoder[12]*4096/2/M_PI));
                    ROS_INFO("ID=%d  ,ABS=%f   ,command=%d",j ,AbsEncoder[j+1] ,diffs[j]);

                                    msg.data.clear();
                                    for(int  k = 0;k < 12;k++)
                                    {
                                        msg.data.push_back(diffs[k]);
                                        //msg.data.push_back(100);
                                    }

                                    chatter_pub.publish(msg);
                                    ros::spinOnce();
                                    loop_rate.sleep();

                    //QThread::msleep(d);

                }

            }
        }


//        msg.data.clear();
//        for(int  k = 0;k < 12;k++)
//        {
//            msg.data.push_back(diffs[k]);
//            //msg.data.push_back(100);
//        }

//        chatter_pub.publish(msg);

//        ros::spinOnce();
//        loop_rate.sleep();


    }

    return 0;
}
