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
#include<termios.h>
#include "MinimumJerkInterpolation.h"

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
    pub29 = nh.advertise<std_msgs::Float64>("rrbot/joint26_position_controller/command",1000);
    pub30 = nh.advertise<std_msgs::Float64>("rrbot/joint27_position_controller/command",1000);
    pub31 = nh.advertise<std_msgs::Float64>("rrbot/joint28_position_controller/command",1000);



    right_hand hand_funcs;

    VectorXd r_target(3);

    MatrixXd R_target(3,3);


        r_target<<.5,
                -0.1,
                -0.2;

R_target=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,-10*M_PI/180+atan2(r_target(1),r_target(0)),3);

VectorXd r_deliver(3);

MatrixXd R_deliver(3,3);



    r_deliver<<r_target(0),
            r_target(1)-.2,
            r_target(2)+.2;
            //R_deliver=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,-10*M_PI/180+atan2(r_deliver(1),r_deliver(0)),3);
    R_deliver=R_target;



VectorXd q0(7);

q0<<10*M_PI/180,
        -4*M_PI/180,
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




double theta_deliver;
double sai_deliver;
double phi_deliver;






//ROS_INFO("theta_target=%f,sai_target=%f,phi_target=%f",theta_target,sai_target,phi_target);
ROS_INFO("\nr_target=\n%f\n%f\n%f",r_target(0),r_target(1),r_target(2));
ROS_INFO("\nR_target=\n%f\t%f\t%f\n%f\t%f\t%f\n%f\t%f\t%f\n",R_target(0,0),R_target(0,1),R_target(0,2),R_target(1,0),R_target(1,1),R_target(1,2),R_target(2,0),R_target(2,1),R_target(2,2));
ROS_INFO("press any key to start!");
ROS_INFO("q0=[%f,%f,%f,%f,%f,%f,%f]",q0(0),q0(1),q0(2),q0(3),q0(4),q0(5),q0(6));
getch();

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
qr2.append(q_ra(1));
qr3.append(q_ra(2));
qr4.append(q_ra(3));
qr5.append(q_ra(4));
qr6.append(q_ra(5));
qr7.append(q_ra(6));


vector<double> q_init(31);
for (int i = 0; i < 31; ++i) {
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



int q_motor[8];
for (int var = 0; var < 8; ++var) {
    q_motor[var]=0;
}

vector<double> q(31);


ros::Rate loop_rate(200);
int count = 1;
int endofdeliver;
bool reaching=true;
bool gripping=false;
bool delivering=false;
bool getting_back=false;


    while (ros::ok())
    {
if(reaching){
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

count=qr1.size()-1;
        d=hand.dist;
        theta=hand.theta;
        sai=hand.sai;
        phi=hand.phi;
               ROS_INFO("x=%f\ty=%f\tz=%f\t",hand.r_right_palm(0),hand.r_right_palm(1),hand.r_right_palm(2));

        i++;
}
        else{
            ROS_INFO("reached to target!");
            reaching=false;
        delivering=true;
        right_hand hand(q_ra,r_deliver,R_deliver,0,0);
        double d0=hand0.dist;
         d=d0;
                    theta_deliver=hand.theta_target;
                 sai_deliver=hand.sai_target;
                 phi_deliver=hand.phi_target;
                 i=0;


        }
}

if(delivering){

    if ((d>d_des)   || (abs(theta-theta_deliver)>.02)   || (abs(sai-sai_deliver)>.02)  || (abs(phi-phi_deliver)>.02)  )
{


    right_hand hand(q_ra,r_deliver,R_deliver,i,d0);


    hand.doQP(q_ra);
    q_ra=hand.q_next;

    qr1.append(q_ra(0));
    qr2.append(q_ra(1));
    qr3.append(q_ra(2));
    qr4.append(q_ra(3));
    qr5.append(q_ra(4));
    qr6.append(q_ra(5));
    qr7.append(q_ra(6));

count=qr1.size()-1;
    d=hand.dist;
    theta=hand.theta;
    sai=hand.sai;
    phi=hand.phi;

    i++;
    ROS_INFO("x=%f\ty=%f\tz=%f\t",hand.r_right_palm(0),hand.r_right_palm(1),hand.r_right_palm(2));

    }

    else{
ROS_INFO("delivered the object!");
    delivering=false;

    getting_back=true;
  endofdeliver=count;

    }

}
 if(getting_back && (abs(q_ra(0)-q0(0))>.0001)){

    q_ra(0)+=(q0(0)-qr1[endofdeliver])/5*hand0.T;
    q_ra(1)+=(q0(1)-qr2[endofdeliver])/5*hand0.T;
    q_ra(2)+=(q0(2)-qr3[endofdeliver])/5*hand0.T;
    q_ra(3)+=(q0(3)-qr4[endofdeliver])/5*hand0.T;
    q_ra(4)+=(q0(4)-qr5[endofdeliver])/5*hand0.T;
    q_ra(5)+=(q0(5)-qr6[endofdeliver])/5*hand0.T;
    q_ra(6)+=(q0(6)-qr7[endofdeliver])/5*hand0.T;


    qr1.append(q_ra(0));
    qr2.append(q_ra(1));
    qr3.append(q_ra(2));
    qr4.append(q_ra(3));
    qr5.append(q_ra(4));
    qr6.append(q_ra(5));
    qr7.append(q_ra(6));
    count=qr1.size()-1;
}







        for (int i = 0; i < 31; ++i) {
            q[i]=0;
        }


        q[15]=qr1[count];
        q[16]=qr2[count];
        q[17]=qr3[count];
        q[18]=qr4[count];

        q[19]=qr5[count];
        q[20]=qr6[count];
        q[21]=qr7[count];

//       ROS_INFO("q1=%f\tq2=%f\tq3=%f\tq4=%f\tq5=%f\tq6=%f\tq7=%f\t",180/M_PI*q[15],180/M_PI*q[16],180/M_PI*q[17],180/M_PI*q[18],180/M_PI*q[19],180/M_PI*q[20],180/M_PI*q[21]);
//       ROS_INFO("x=%f\ty=%f\tz=%f\t",hand.r_right_palm(0),hand.r_right_palm(1),hand.r_right_palm(2));
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





           }
    return 0;
}
