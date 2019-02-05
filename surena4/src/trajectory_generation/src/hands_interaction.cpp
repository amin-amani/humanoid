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
#include "left_hand.h"
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
ros::Publisher pub29 ;
ros::Publisher pub30 ;
ros::Publisher pub31 ;


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
    data.data=q[29];
    pub29.publish(data);
    data.data=q[30];
    pub30.publish(data);
    data.data=q[31];
    pub31.publish(data);

}



int main(int argc, char **argv)
{
//hand_gui my_hand_gui;
//my_hand_gui.show();
    ros::init(argc, argv, "myNode");

    ros::NodeHandle nh;
    ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",1000);

    std_msgs::Int32MultiArray msg;
    std_msgs::MultiArrayDimension msg_dim;

    msg_dim.label = "joint_position";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);



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
    pub29 = nh.advertise<std_msgs::Float64>("rrbot/joint29_position_controller/command",1000);
    pub30 = nh.advertise<std_msgs::Float64>("rrbot/joint30_position_controller/command",1000);
    pub31 = nh.advertise<std_msgs::Float64>("rrbot/joint31_position_controller/command",1000);



    right_hand hand_funcs;


    VectorXd r_target(3);
    MatrixXd R_target(3,3);
    VectorXd r_target_l(3);
    MatrixXd R_target_l(3,3);
        r_target<<.4,
                0.05,
                -0.2;
R_target=hand_funcs.rot(2,-120*M_PI/180,3)*hand_funcs.rot(3,-40*M_PI/180,3);

r_target_l<<.4,
        -0.05,
        -0.2;
R_target_l=hand_funcs.rot(2,-120*M_PI/180,3)*hand_funcs.rot(3,40*M_PI/180,3);


VectorXd q0(7);
VectorXd q0_l(7);
q0<<10*M_PI/180,
        -4*M_PI/180,
        0,
        -20*M_PI/180,
        0,
        0,
        0;

q0_l<<10*M_PI/180,
        4*M_PI/180,
        0,
        -20*M_PI/180,
        0,
        0,
        0;


right_hand hand0(q0,r_target,R_target,0,0);
left_hand hand0_l(q0_l,r_target_l,R_target_l,0,0);
double d0=hand0.dist;
double d=d0;
double d_des=hand0.d_des;
double theta=hand0.theta;
double theta_target=hand0.theta_target;
double sai=hand0.sai;
double sai_target=hand0.sai_target;
double phi=hand0.phi;
double phi_target=hand0.phi_target;


double d0_l=hand0.dist;
double d_l=d0;

double theta_l=hand0_l.theta;
double theta_target_l=hand0_l.theta_target;
double sai_l=hand0_l.sai;
double sai_target_l=hand0_l.sai_target;
double phi_l=hand0_l.phi;
double phi_target_l=hand0_l.phi_target;
//ROS_INFO("theta_target=%f,sai_target=%f,phi_target=%f",theta_target,sai_target,phi_target);
ROS_INFO("\nr_target=\n%f\n%f\n%f",r_target(0),r_target(1),r_target(2));
ROS_INFO("\nR_target=\n%f\t%f\t%f\n%f\t%f\t%f\n%f\t%f\t%f\n",R_target(0,0),R_target(0,1),R_target(0,2),R_target(1,0),R_target(1,1),R_target(1,2),R_target(2,0),R_target(2,1),R_target(2,2));
ROS_INFO("press any key to start!");
getch();

QVector<double> qr1;
QVector<double> qr2;
QVector<double> qr3;
QVector<double> qr4;
QVector<double> qr5;
QVector<double> qr6;
QVector<double> qr7;

QVector<double> ql1;
QVector<double> ql2;
QVector<double> ql3;
QVector<double> ql4;
QVector<double> ql5;
QVector<double> ql6;
QVector<double> ql7;

double x_r,y_r,z_r,x_l,y_l,z_l;

int i=0;
int i_l=0;
VectorXd q_ra;
VectorXd q_la;
q_ra=q0;
q_la=q0_l;

qr1.append(q_ra(0));
qr2.append(q_ra(1));
qr3.append(q_ra(2));
qr4.append(q_ra(3));
qr5.append(q_ra(4));
qr6.append(q_ra(5));
qr7.append(q_ra(6));

ql1.append(q_la(0));
ql2.append(q_la(1));
ql3.append(q_la(2));
ql4.append(q_la(3));
ql5.append(q_la(4));
ql6.append(q_la(5));
ql7.append(q_la(6));

vector<double> q_init(32);
for (int i = 0; i < 32; ++i) {
    q_init[i]=0;
}
q_init[15]=qr1[0];
q_init[16]=qr2[0];
q_init[17]=qr3[0];
q_init[18]=qr4[0];
q_init[19]=qr5[0];
q_init[20]=qr6[0];
q_init[21]=qr7[0];

q_init[22]=ql1[0];
q_init[23]=ql2[0];
q_init[24]=ql3[0];
q_init[25]=ql4[0];
q_init[26]=ql5[0];
q_init[27]=ql6[0];
q_init[28]=ql7[0];





SendGazebo(q_init);



int q_motor[16];
for (int var = 0; var < 16 ; ++var) {
    q_motor[var]=0;
}

vector<double> q(32);


ros::Rate loop_rate(200);
int count = 0;

    while (ros::ok())
    {

//        for (int k = 0; k < 7; ++k) {
//            if (k%2==1){r_target=r_shaking;}
//            else{r_target=r_shaking2;}
//right_hand hand00(q_ra,r_target,R_target,i,d0);
//d=hand00.dist;
//        while ((d>d0/2)  || (abs(theta-theta_target)>.02)   || (abs(sai-sai_target)>.02)  || (abs(phi-phi_target)>.02)  ) {
//while (1){
        if ((d>d_des)   || (abs(theta-theta_target)>.02)   || (abs(sai-sai_target)>.02)  || (abs(phi-phi_target)>.02)  )
{

        right_hand hand(q_ra,r_target,R_target,i,d0);


        hand.doQP(q_ra);
        q_ra=hand.q_next;

        qr1.append(q_ra(0));
        qr2.append(q_ra(1));
        qr3.append(q_ra(2));
        qr4.append(q_ra(3));
        qr5.append(q_ra(4));
        qr6.append(q_ra(5));
        qr7.append(q_ra(6));

        d=hand.dist;
        theta=hand.theta;
        sai=hand.sai;
        phi=hand.phi;

        i++;
//        ROS_INFO("q1=%f\tq2=%f\tq3=%f\tq4=%f\tq5=%f\tq6=%f\tq7=%f\t",180/M_PI*q[15],180/M_PI*q[16],180/M_PI*q[17],180/M_PI*q[18],180/M_PI*q[19],180/M_PI*q[20],180/M_PI*q[21]);
//        ROS_INFO("x=%f\ty=%f\tz=%f\t",hand.r_right_palm(0),hand.r_right_palm(1),hand.r_right_palm(2));
x_r=hand.r_right_palm(0);
y_r=hand.r_right_palm(1);
z_r=hand.r_right_palm(2);

        }
        else{
            //ROS_INFO("right reached!");
            qr1.append(qr1[qr1.size()-1]);
            qr2.append(qr2[qr2.size()-1]);
            qr3.append(qr3[qr3.size()-1]);
            qr4.append(qr4[qr4.size()-1]);
            qr5.append(qr5[qr5.size()-1]);
            qr6.append(qr6[qr6.size()-1]);
            qr7.append(qr7[qr7.size()-1]);

        }


    if ((d_l>d_des)   || (abs(theta_l-theta_target_l)>.02)   || (abs(sai_l-sai_target_l)>.02)  || (abs(phi-phi_target_l)>.02)  )
{

    left_hand hand_l(q_la,r_target_l,R_target_l,i_l,d0_l);


    hand_l.doQP(q_la);
    q_la=hand_l.q_next;

    ql1.append(q_la(0));
    ql2.append(q_la(1));
    ql3.append(q_la(2));
    ql4.append(q_la(3));
    ql5.append(q_la(4));
    ql6.append(q_la(5));
    ql7.append(q_la(6));

    d_l=hand_l.dist;
    theta_l=hand_l.theta;
    sai_l=hand_l.sai;
    phi_l=hand_l.phi;

    i_l++;
//    ROS_INFO("q1=%f\tq2=%f\tq3=%f\tq4=%f\tq5=%f\tq6=%f\tq7=%f\t",180/M_PI*q[15],180/M_PI*q[16],180/M_PI*q[17],180/M_PI*q[18],180/M_PI*q[19],180/M_PI*q[20],180/M_PI*q[21]);
//    ROS_INFO("x=%f\ty=%f\tz=%f\t",hand_l.r_left_palm(0),hand_l.r_left_palm(1),hand_l.r_left_palm(2));
    x_l=hand_l.r_left_palm(0);
    y_l=hand_l.r_left_palm(1);
    z_l=hand_l.r_left_palm(2);
    }
    else{

        //ROS_INFO("left reached!");
        ql1.append(ql1[ql1.size()-1]);
        ql2.append(ql2[ql2.size()-1]);
        ql3.append(ql3[ql3.size()-1]);
        ql4.append(ql4[ql4.size()-1]);
        ql5.append(ql5[ql5.size()-1]);
        ql6.append(ql6[ql6.size()-1]);
        ql7.append(ql7[ql7.size()-1]);

    }



        for (int i = 0; i < 32; ++i) {
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

        q[22]=ql1[count];
        q[23]=ql2[count];
        q[24]=ql3[count];
        q[25]=ql4[count];

        q[26]=ql5[count];
        q[27]=ql6[count];
        q[28]=ql7[count];


        q[29]=20*M_PI/180*sin(double(count)*hand0.T*2);
        q[30]=0.0;
        q[31]=15*M_PI/180;


        ROS_INFO("\nqr=%f,%f,%f,%f,%f,%f,%f\nql=%f,%f,%f,%f,%f,%f,%f",
                 q[15]*180/M_PI,q[16]*180/M_PI,q[17]*180/M_PI,q[18]*180/M_PI,q[19]*180/M_PI,
                q[20]*180/M_PI,q[21]*180/M_PI,q[22]*180/M_PI,q[23]*180/M_PI,q[24]*180/M_PI,
                q[25]*180/M_PI,q[26]*180/M_PI,q[27]*180/M_PI,q[28]*180/M_PI);
        ROS_INFO("\nx_r=%f, y_r=%f, z_r=%f\nx_l=%f, y_l=%f, z_l=%f",x_r,y_r,z_r,x_l,y_l,z_l);





    //   ROS_INFO("T=%f upper:%f %f lower:%f %f qdot=%f q=%f %f",hand.T,(hand.toRad(10)+q_ra(1))/hand.T,hand.qdot_max,(hand.toRad(-90)+q_ra(1))/hand.T,-hand.qdot_max,hand.qdot(2),180/M_PI*q[16],180/M_PI*q_ra(1));



       SendGazebo(q);



       msg.data.clear();

      for(int  i = 0;i < 12;i++)
      {
          msg.data.push_back(0);
      }

      q_motor[0]=-int(6*(qr1[count]-q0(0))*360/M_PI);
      q_motor[1]=int(6*(qr2[count]-q0(1))*360/M_PI);
      q_motor[2]=-int(6*(qr3[count]-q0(2))*300/M_PI);
      q_motor[3]=int(6*(qr4[count]-q0(3))*300/M_PI);



      //right hand epose
      msg.data.push_back(q_motor[0]);//12 -y  a,z
      msg.data.push_back(q_motor[1]);//13 +x
      msg.data.push_back(q_motor[2]);//14 -z
      msg.data.push_back(q_motor[3]);//15 +y
      //right hand dynamixel + fingers
      msg.data.push_back(0);//16
      msg.data.push_back(0);//17
      msg.data.push_back(0);//18
      msg.data.push_back(0);//19
      //left hand epose
      msg.data.push_back(q_motor[4]);//20 +y
      msg.data.push_back(q_motor[5]);//21 +x
      msg.data.push_back(q_motor[6]);//22 -z
      msg.data.push_back(q_motor[7]);//23 -y
      //left hand dynamixel + fingers
      msg.data.push_back(0);//24
      msg.data.push_back(0);//25
      msg.data.push_back(0);//26
      msg.data.push_back(0);//27

        chatter_pub.publish(msg);


        ros::spinOnce();
        loop_rate.sleep();
        ++count;



}

    return 0;
}
