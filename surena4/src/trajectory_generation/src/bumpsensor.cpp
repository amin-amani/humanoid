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

using namespace  std;
using namespace  Eigen;
//data of left foot sensor
double a;
double b;
double c;
double d;



void receiveFootSensor(const std_msgs::Int32MultiArray& msg)
{
    if (msg.data.size()!=4) {
        qDebug("the size of sensor data is in wrong");
        return;
    }

    //ROS_INFO("I heard: [%d  %d %d %d]", (int)msg.data[0],(int)msg.data[1],(int)msg.data[2],(int)msg.data[3]);
    double temp[4];
    int tempInt[4];
    temp[0]=msg.data[0];
    temp[1]=msg.data[1];
    temp[2]=msg.data[2];
    temp[3]=msg.data[3];
    //normalizing data of sensors
    temp[0]=temp[0]*(100.0/105);
    temp[1]=temp[1]*(100.0/99);
    temp[2]=temp[2]*(100.0/116);
    temp[3]=temp[3]*(100.0/116);

    tempInt[0]=temp[0];
    tempInt[1]=temp[1];
    tempInt[2]=temp[2];
    tempInt[3]=temp[3];

    a=tempInt[0];
    b=tempInt[1];
    c=tempInt[2];
    d=tempInt[3];

   // ROS_INFO("I heard: [%d  %d %d %d]", tempInt[0],tempInt[1],tempInt[2],tempInt[3]);
}



int main(int argc, char **argv)
{


    QCgenerator QC;
    QElapsedTimer timer;
    double dt;

    double k1;
    double k2;
    double k3;
    double k4;

    bool aState=false;
    bool bState=false;
    bool cState=false;
    bool dState=false;

    double teta_motor_L=0;
    double teta_motor_R=0;
    double phi_motor_L=0;
    double phi_motor_R=0;


    double footSensorSaturation=90;
    //*******************This part of code is for initialization of joints of the robot for walking**********************************
    int count = 0;

    ros::init(argc, argv, "bumpsensorcontroller");

    ros::NodeHandle nh;
    ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",1000);

    ros::Subscriber sub = nh.subscribe("/foot", 1000, receiveFootSensor);


    ros::Rate loop_rate(100);
    std_msgs::Int32MultiArray msg;
    std_msgs::MultiArrayDimension msg_dim;

    msg_dim.label = "joint_position";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);
    k1=0.00009;
    k2=0.00009;
    k3=0.00001;
    k4=0.00001;


    while (ros::ok())
    {



        if ((a)>=footSensorSaturation && (b)>=footSensorSaturation && (c)>=footSensorSaturation && (d)>=footSensorSaturation){
            qDebug("swing foot landing is successful");
            aState=true;
            bState=true;
            cState=true;
            dState=true;

            //do nothing all sensors are on the ground
            teta_motor_L=teta_motor_L;
            phi_motor_L=phi_motor_L;

            teta_motor_L=teta_motor_R;
            phi_motor_L=phi_motor_R;

            exit(0);

        }
       else {//when the four sensors are not saturated


//            if ((a)>=footSensorthreshold ){aState=true;}else {aState=false;}
//            if ((b)>=footSensorthreshold ){bState=true;}else {bState=false;}
//            if ((c)>=footSensorthreshold ){cState=true;}else {cState=false;}
//            if ((d)>=footSensorthreshold ){dState=true;}else {dState=false;}





//        minimum={-14.0,-55.0,0.0,-50.0,-55.0,-14.0,0.0,-50.0,-23.0,-15.0,-15.0,-23.0};
//        maximum={14.0,26.0,75.0,35.0,26.0,14.0,75.0,35.0,23.0,10.0,10.0,23.0};



        //Pitch left ankle motor
        if (abs(b-a)>=abs(c-d)) {

            if (abs(a-b)<100) {
                teta_motor_L=1*teta_motor_L+k1*(a-b);
            }
            else {
                teta_motor_L=teta_motor_L;
            }

        }

        else {
            if (abs(d-c)<100) {
                teta_motor_L=1*teta_motor_L+k1*(d-c);
            }
            else {
                teta_motor_L=teta_motor_L;
            }


        }



        //Roll left ankle motor
        if (abs(c-b)>=abs(d-a)) {

            if (abs(c-b)<100) {
                phi_motor_L=1*phi_motor_L+k1*(c-b);
            }
            else {
                phi_motor_L=phi_motor_L;
            }

        }

        else {

            if (abs(a-d)<100) {
                phi_motor_L=1*phi_motor_L+k1*(d-a);
            }
            else {
                phi_motor_L=phi_motor_L;
            }

        }
}
//saturation of ankle motors

        if ((abs(phi_motor_L))>0.9) {
            phi_motor_L=0.9;
        }

        if ((abs(teta_motor_L))>0.9) {
            teta_motor_L=0.9;
        }


        if ((abs(phi_motor_R))>0.9) {
            phi_motor_L=0.9;
        }

        if ((abs(teta_motor_R))>0.9) {
            teta_motor_R=0.9;
        }


ROS_INFO("I heard data of sensors : [%f %f %f %f]",a,b,c,d);
//        ROS_INFO("I heard motor pitch: [%f]",teta_motor_L);
//        ROS_INFO("I heard motor roll: [%f]",phi_motor_L);
        msg.data.clear();
        vector<double> cntrl(13);
        cntrl[0]=0.0;
        cntrl[1]=0;
        cntrl[2]=0;
        cntrl[3]=0;
        cntrl[4]=0;
        cntrl[5]=teta_motor_R;//pitch
        cntrl[6]=phi_motor_R;//roll
        cntrl[7]=0;
        cntrl[8]=0;
        cntrl[9]=0;
        cntrl[10]=0;
        cntrl[11]=1*teta_motor_L;
        cntrl[12]=1*phi_motor_L;

        vector<int> qref(12);
        qref=QC.ctrldata2qc(cntrl);

        for(int  i = 0;i < 12;i++)
        {
            msg.data.push_back(qref[i]);

        }

        chatter_pub.publish(msg);
        // ROS_INFO("t={%d} c={%d}",timer.elapsed(),count);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}

