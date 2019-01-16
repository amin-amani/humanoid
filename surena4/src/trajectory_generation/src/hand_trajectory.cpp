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
#include "right_hand.h"
#include "hand_gui.h"


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

void  SendGazebo(vector<double> q){

    std_msgs::Float64 data;
    data.data=q[1];
    pub1.publish(data);
    data.data=q[2];
    pub2.publish(data);
    data.data=q[3];
    pub3.publish(data);
    data.data=q[4];
    pub4.publish(data);
    data.data=q[5];
    pub5.publish(data);
    data.data=q[6];
    pub6.publish(data);
    data.data=q[7];
    pub7.publish(data);
    data.data=q[8];
    pub8.publish(data);
    data.data=q[9];
    pub9.publish(data);
    data.data=q[10];
    pub10.publish(data);
    data.data=q[11];
    pub11.publish(data);
    data.data=q[12];
    pub12.publish(data);
    data.data=q[13];
    pub13.publish(data);
    data.data=q[14];
    pub14.publish(data);
    data.data=q[15];
    pub15.publish(data);
    data.data=q[16];
    pub16.publish(data);
    data.data=q[17];
    pub17.publish(data);
    data.data=q[18];
    pub18.publish(data);
    data.data=q[19];
    pub19.publish(data);
    data.data=q[20];
    pub20.publish(data);
    data.data=q[21];
    pub21.publish(data);
    data.data=q[22];
    pub22.publish(data);
    data.data=q[23];
    pub23.publish(data);
    data.data=q[24];
    pub24.publish(data);
    data.data=q[25];
    pub25.publish(data);
    data.data=q[26];
    pub26.publish(data);
    data.data=q[27];
    pub27.publish(data);
    data.data=q[28];
    pub28.publish(data);

}



int main(int argc, char **argv)
{
//hand_gui my_hand_gui;
//my_hand_gui.show();
    ros::init(argc, argv, "myNode");

    ros::NodeHandle nh;
    ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",1000);

{

    pub1  = nh.advertise<std_msgs::Float64>("rrbot/joint1_position_controller/command",1000);
    pub2  = nh.advertise<std_msgs::Float64>("rrbot/joint2_position_controller/command",1000);
    pub3  = nh.advertise<std_msgs::Float64>("rrbot/joint3_position_controller/command",1000);
    pub4  = nh.advertise<std_msgs::Float64>("rrbot/joint4_position_controller/command",1000);
    pub5  = nh.advertise<std_msgs::Float64>("rrbot/joint5_position_controller/command",1000);
    pub6  = nh.advertise<std_msgs::Float64>("rrbot/joint6_position_controller/command",1000);
    pub7  = nh.advertise<std_msgs::Float64>("rrbot/joint7_position_controller/command",1000);
    pub8  = nh.advertise<std_msgs::Float64>("rrbot/joint8_position_controller/command",1000);
    pub9  = nh.advertise<std_msgs::Float64>("rrbot/joint9_position_controller/command",1000);
    pub10 = nh.advertise<std_msgs::Float64>("rrbot/joint10_position_controller/command",1000);
    pub11 = nh.advertise<std_msgs::Float64>("rrbot/joint11_position_controller/command",1000);
    pub12 = nh.advertise<std_msgs::Float64>("rrbot/joint12_position_controller/command",1000);
    pub13 = nh.advertise<std_msgs::Float64>("rrbot/joint13_position_controller/command",1000);
    pub14 = nh.advertise<std_msgs::Float64>("rrbot/joint14_position_controller/command",1000);
    pub15 = nh.advertise<std_msgs::Float64>("rrbot/joint15_position_controller/command",1000);
    pub16 = nh.advertise<std_msgs::Float64>("rrbot/joint16_position_controller/command",1000);
    pub17 = nh.advertise<std_msgs::Float64>("rrbot/joint17_position_controller/command",1000);
    pub18 = nh.advertise<std_msgs::Float64>("rrbot/joint18_position_controller/command",1000);
    pub19 = nh.advertise<std_msgs::Float64>("rrbot/joint19_position_controller/command",1000);
    pub20 = nh.advertise<std_msgs::Float64>("rrbot/joint20_position_controller/command",1000);
    pub21 = nh.advertise<std_msgs::Float64>("rrbot/joint21_position_controller/command",1000);
    pub22 = nh.advertise<std_msgs::Float64>("rrbot/joint22_position_controller/command",1000);
    pub23 = nh.advertise<std_msgs::Float64>("rrbot/joint23_position_controller/command",1000);
    pub24 = nh.advertise<std_msgs::Float64>("rrbot/joint24_position_controller/command",1000);
    pub25 = nh.advertise<std_msgs::Float64>("rrbot/joint25_position_controller/command",1000);
    pub26 = nh.advertise<std_msgs::Float64>("rrbot/joint26_position_controller/command",1000);
    pub27 = nh.advertise<std_msgs::Float64>("rrbot/joint27_position_controller/command",1000);
    pub28 = nh.advertise<std_msgs::Float64>("rrbot/joint28_position_controller/command",1000);

}

    right_hand hand_funcs;





    VectorXd r_target(3);

    MatrixXd R_target(3,3);
    //shaking

//    r_target<<.5,
//            .05,
//            -.35;

        r_target<<.5,
                0.2,
                -.2;
R_target=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,-20*M_PI/180+atan2(r_target(1),r_target(0)),3);
//    r_target<<0,
//            -.550,
//            0;


//from base to end


  // R_target=hand_funcs.rot(3,-90*M_PI/180,3)*hand_funcs.rot(2,-90*M_PI/180,3);


 //R_target=hand_funcs.rot(2,90*M_PI/180,3)*hand_funcs.rot(3,-90*M_PI/180,3);
//R_target=hand_funcs.rot(2,90*M_PI/180,3)*hand_funcs.rot(3,90*M_PI/180,3);



VectorXd q0(7);
//q0<<0,
//        0,
//        0,
//        -10*M_PI/180,
//        0,
//        0,
//        0;
q0<<10*M_PI/180,
        4*M_PI/180,
        0,
        -20*M_PI/180,
        0,
        0,
        0;

double v0=0;
double v_target =.4;
right_hand hand0(q0,r_target,R_target,0,0);
double d0=hand0.dist;
double d=d0;
double d_des=hand0.d_des;
double theta=hand0.theta;
double theta_target=hand0.theta_target;
double sai=hand0.sai;
double sai_target=hand0.sai_target;
double phi=hand0.phi;
double phi_target=hand0.phi_target;


QVector<double> qr1;
QVector<double> qr2;
QVector<double> qr3;
QVector<double> qr4;
QVector<double> qr5;
QVector<double> qr6;
QVector<double> qr7;


int i=0;
VectorXd q_ra;
q_ra=q0;

qr1.append(q_ra(0));
qr2.append(-q_ra(1));
qr3.append(-q_ra(2));
qr4.append(q_ra(3));
qr5.append(-q_ra(4));
qr6.append(-q_ra(5));
qr7.append(q_ra(6));

vector<double> q_init(30);
for (int i = 0; i < 30; ++i) {
    q_init[i]=0;
}
q_init[15]=qr1[0];
q_init[16]=qr2[0];
q_init[17]=qr3[0];
q_init[18]=qr4[0];
q_init[19]=qr5[0];
q_init[20]=qr6[0];
q_init[21]=qr7[0];


SendGazebo(q_init);






vector<double> q(30);


ros::Rate loop_rate(int(1.0/hand0.T));
int count = 0;

    while (ros::ok())
    {
//        for (int k = 0; k < 7; ++k) {
//            if (k%2==1){r_target=r_shaking;}
//            else{r_target=r_shaking2;}
//right_hand hand00(q_ra,r_target,R_target,i,d0);
//d=hand00.dist;
        while ((d>d0/2)  || (abs(theta-theta_target)>.02)   || (abs(sai-sai_target)>.02)  || (abs(phi-phi_target)>.02)  ) {
//while (1){
        right_hand hand(q_ra,r_target,R_target,i,d0);



        hand.doQP(q_ra);
        q_ra=hand.q_next;

        qr1.append(q_ra(0));
        qr2.append(-q_ra(1));
        qr3.append(-q_ra(2));
        qr4.append(q_ra(3));
        qr5.append(-q_ra(4));
        qr6.append(-q_ra(5));
        qr7.append(q_ra(6));

        d=hand.dist;
        theta=hand.theta;
        sai=hand.sai;
        phi=hand.phi;

        i++;

        for (int i = 0; i < 30; ++i) {
            q[i]=0;
        }

//         q[15]=-M_PI/2;
//         q[16]=-10*M_PI/180;
//         q[17]=-10*M_PI/180;

        q[15]=qr1[count];
        q[16]=qr2[count];
        q[17]=qr3[count];
        q[18]=qr4[count];


        q[19]=qr5[count];
        q[20]=qr6[count];
        q[21]=qr7[count];

       ROS_INFO("q1=%f\tq2=%f\tq3=%f\tq4=%f\tq5=%f\tq6=%f\tq7=%f\t",180/M_PI*q[15],180/M_PI*q[16],180/M_PI*q[17],180/M_PI*q[18],180/M_PI*q[19],180/M_PI*q[20],180/M_PI*q[21]);
       ROS_INFO("x=%f\ty=%f\tz=%f\t",hand.r_right_palm(0),hand.r_right_palm(1),hand.r_right_palm(2));
        SendGazebo(q);

        //chatter_pub.publish(msg);


        ros::spinOnce();
        loop_rate.sleep();
        ++count;
}
        


           }
    return 0;
}
