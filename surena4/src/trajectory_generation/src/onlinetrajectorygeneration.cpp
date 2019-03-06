#include "ros/ros.h"
#include "std_msgs/String.h"
#include"Eigen/Dense"
#include <vector>
#include <iostream>
#include <QString>
#include <QList>
#include "Robot.h"
#include"taskspaceoffline.h"
#include <qmath.h>
#include <cstring>
#include<qdebug.h>
#include <Eigen/Geometry>
#include <cstdlib>
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
#include <std_msgs/Empty.h>
#include "std_srvs/Empty.h"
#include "qcgenerator.h"
#include <qthread.h>
#include<sensor_msgs/Imu.h>
#include<gazebo_msgs/ApplyBodyWrenchRequest.h>
#include <gazebo_msgs/GetLinkState.h>
#include<gazebo_msgs/LinkStates.h>
#include <nav_msgs/Odometry.h>
#include "std_srvs/Empty.h"
#include "qcgenerator.h"
#include "surenastateestimation.h"
#include<sensor_msgs/JointState.h>
#include<std_msgs/Float64.h>
#include<termios.h>

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

double q0, q1, q2, q3; // x~q1 y~q2 z~q3 w~q0
double Roll, Pitch, Yaw;
bool QcInitialBool;

void RecieveIMUOrientation(const sensor_msgs::Imu & msg)
{
    ROS_INFO("%f",msg.orientation.w);
    q0 = msg.orientation.w;
    q1 = msg.orientation.x;
    q2 = msg.orientation.y;
    q3 = msg.orientation.z;

    Roll = msg.orientation.x;
    Pitch = msg.orientation.y;
    Yaw = msg.orientation.z;
}

void QcInitial(const sensor_msgs::JointState & msg){
    if (QcInitialBool){
        ROS_INFO("Initialized!");
        QcInitialBool=false;
    }

}

int getch()
{
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

    int c = getchar();  // read character (non-blocking)

    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return c;
}

int main(int argc, char **argv)
{
    QcInitialBool=true;
    SurenaStateEstimation StateEstimation;
    ros::init(argc, argv, "myNode");
    ros::NodeHandle nh;
    ros::Publisher SendJointData  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",1000);
    ros::Publisher SendDeltaPosition = nh.advertise<std_msgs::Float32MultiArray>("/surena/PelvisTilt",1000);
    ros::Subscriber qcinit = nh.subscribe("/surena/inc_joint_state", 1000, QcInitial);
    ros::Subscriber  IMU_xsens = nh.subscribe("/surena/imu_state", 1000, RecieveIMUOrientation);

    ros::Rate loop_rate(200);
    std_msgs::Int32MultiArray msg;
    std_msgs::Float32MultiArray msgTilt;
    ROS_INFO("Press Any Key to Start!");
    getch();
    ROS_INFO("started!");
    while (ros::ok())
    {
//        if (QcInitialBool) {
//            ROS_INFO("qc is initializing!");
//            ros::spinOnce();
//            continue;
//        }
        //        q0 = 0.25;
        //        q1 = 0.25;
        //        q2 = 0.25;
        //        q3 = 0.25;
        MatrixXd RotationMatrix = StateEstimation.MatrixRotationPelvisRPY(Roll,Pitch,0); //yaw is zero intentionally because the real value contains drift.
        MatrixXd LocalPosition(3,1);
//        LocalPosition << 113-30, 115, 0;
        LocalPosition << 0, 0, 10;
        MatrixXd DeltaPosition;
        DeltaPosition = StateEstimation.PelvisLocalChange(RotationMatrix,LocalPosition);
        msg.data.clear();
        msgTilt.data.clear();
        for(int  i = 0;i < 28;i++)
        {
            msg.data.push_back(0);
        }
        for(int j=1;j<4;j++)
        {
            msgTilt.data.push_back(DeltaPosition(j-1,0));
        }
        SendJointData.publish(msg);
        SendDeltaPosition.publish(msgTilt);
        //ROS_INFO("%f%f%f",msgTilt(0,0),msgTilt(1,0),msgTilt(2,0));
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
