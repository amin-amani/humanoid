
#include "ros/ros.h"
#include "std_msgs/String.h"
#include"Eigen/Dense"
#include <vector>
#include <iostream>
#include <QString>
#include <QList>
#include "Robot.h"
//#include"TaskSpace.h"
#include"MinimumJerkInterpolation.h"
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
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseArray.h>
#include <cstdlib>
#include<termios.h>
#include <iostream>
#include <QTime>
#include"pidcontroller.h"

using namespace  std;
using namespace  Eigen;

double X_face=320;
double Y_face=200;
PIDController head_yaw_pid;
PIDController head_pitch_pid;

VectorXd r_bottle_cam(3);
VectorXd r_bottle_shoulder(3);
double head_yaw=0;
double head_pitch=0;
double head_roll=0;
int n;
double saturate(double a, double min, double max){
    if(a<min){//ROS_INFO("subceeding!");
        return min;}
    else if(a>max){//ROS_INFO("exceeding!");
        return max;}
    else{return a;}
}

void numplot(double num,double min,double max){
    //⬛

    QString str;
    int l=100;
    int n=int((num-min)/(max-min)*l);
    if (num<min){n=0;}
    if (num>max){n=100;}
    str+=QString::number(min);
    str+="|";
    if (n<=l/2){
        for (int i = 0; i < n; ++i) {
            str+=" ";
        }
        for (int i = 0; i < l/2-n; ++i) {
            str+="|";

        }
        str+="|";
        for (int i = 0; i < l/2; ++i) {
            str+=" ";
        }
    }
    else {
        for (int i = 0; i < l/2; ++i) {
            str+=" ";
        }
        for (int i = 0; i < n-l/2; ++i) {
            str+="|";

        }
        str+="|";
        for (int i = 0; i < l-n; ++i) {
            str+=" ";
        }

    }

    str+="|";
    str+=QString::number(max);
    str+="=>";str+=QString::number(num);
    qDebug()<<str;
    qDebug()<<"";


}

void matrix_view(MatrixXd M){

    for (int i = 0; i <M.rows() ; ++i) {
        QString str;
        for (int j = 0; j <M.cols() ; ++j) {
            str+=QString::number(M(i,j));
            str+="   ";
        }
        qDebug()<<str;
    }
    qDebug()<<"";
}


void matrix_view(VectorXd M){
    QString str;
    for (int i = 0; i <M.rows() ; ++i) {str+=QString::number(M(i));str+="   ";}
    qDebug()<<str;
    qDebug()<<"";
}

double move2pose(double max,double t_local,double T_start ,double T_end){
    double T_move=T_end-T_start;
    double c3=(10*max)/pow(T_move,3);
    double c4=-(15*max)/pow(T_move,4);
    double c5=(6*max)/pow(T_move,5);
    double theta=0;
    if(t_local<T_start){theta=0;}
    else if (t_local<T_end){theta=c3*pow(t_local-T_start,3)+c4*pow(t_local-T_start,4)+c5*pow(t_local-T_start,5);}
    else{theta=max;}
    return theta;
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

int qc_offset[12];
int qra_offset[4];
int qla_offset[4];
bool qc_initial_bool;
void qc_initial(const sensor_msgs::JointState & msg){
    if (qc_initial_bool){

        for (int i = 0; i < 12; ++i) {
            qc_offset[i]=int(msg.position[i+1]);

        }

        for (int i = 12; i < 16; ++i){
            qra_offset[i-12]=int(msg.position[i+1]);
        }

        for (int i = 20; i < 24; ++i){
            qla_offset[i-20]=int(msg.position[i+1]);
        }


        qc_initial_bool=false;

        ROS_INFO("Offset_feet=%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\n",
                 qc_offset[0],qc_offset[1],qc_offset[2],qc_offset[3],qc_offset[4],
                qc_offset[5],qc_offset[6],qc_offset[7],qc_offset[8],qc_offset[9],
                qc_offset[10],qc_offset[11]);
        ROS_INFO("Offset_righthand=%d\t%d\t%d\t%d",qra_offset[0],qra_offset[1], qra_offset[2],qra_offset[3]);
        ROS_INFO("Offset_lefthand=%d\t%d\t%d\t%d",qla_offset[0],qla_offset[1], qla_offset[2],qla_offset[3]);
        ROS_INFO("Initialized!");
    }
}

VectorXd r_camera2shoulder(VectorXd r_camera)
{
    double roll2pitch=.05;
    double yaw2roll=.1;
    VectorXd r_neck(3);
    r_neck<<.03,0.3,0;

    MatrixXd T0(4,4);
    T0<<1,0,0,0,  0,cos(M_PI/12),-sin(M_PI/12),0,  0,sin(M_PI/12),cos(M_PI/12),0,  0,0,0,1;
    MatrixXd T1(4,4);
    T1<<1,0,0,r_camera(0),  0,1,0,r_camera(1),  0,0,1,r_camera(2),  0,0,0,1;
  MatrixXd T2(4,4);
  T2<<1,0,0,0,  0,cos(head_pitch),-sin(head_pitch),0,  0,sin(head_pitch),cos(head_pitch),0,  0,0,0,1;
  MatrixXd T3(4,4);
  T3<<cos(head_roll),0,sin(head_roll),0,  0,1,0,0,    -sin(head_roll),0,cos(head_roll),roll2pitch,   0,0,0,1;
  MatrixXd T4(4,4);
  T4<<cos(head_yaw),-sin(head_yaw),0,0,  sin(head_yaw),cos(head_yaw),0,0,  0,0,1,yaw2roll,  0,0,0,1;
  MatrixXd T5(4,4);
  T5<<1,0,0,r_neck(0),  0,1,0,r_neck(1),  0,0,1,r_neck(2),  0,0,0,1;
  MatrixXd T(4,4);
  T=T5*T4*T3*T2*T1*T0;
  VectorXd temp(4); temp<<r_camera,1;
  temp=T*temp;

  return temp.block(0,0,3,1);


}
void object_detect(const geometry_msgs::PoseArray & msg){
    double l=640;
    double w=480;
    double a=.51;
    double b=.385;
    double X0=.493;
    double Y0,Z0,L0,X,Y,Z;
    double x=l/2;
    double y=w/2;
    double z=0;
  int n=msg.poses.size();
  for (int i = 0; i < n; ++i) {
      if (msg.poses[i].orientation.x==39) {
         x=msg.poses[i].position.x;
         y=msg.poses[i].position.y;
         z=msg.poses[i].position.z;
        break;
      }
 }
Y0=-(x-l/2)/l*a;
Z0=-(y-w/2)/w*b;
L0=sqrt(X0*X0+Y0*Y0+Z0*Z0);
X=z/L0*X0;
Y=X*Y0/X0;
Z=X*Z0/X0;
if(X!=0){r_bottle_cam<<X,Y,Z;}
r_bottle_shoulder=r_camera2shoulder(r_bottle_cam);
matrix_view(r_bottle_shoulder);

}

void face_detect(const geometry_msgs::PoseArray & msg){



  n=msg.poses.size();

if(n!=0) {
      X_face=msg.poses[0].position.x;
      Y_face=msg.poses[0].position.y;
  }




}




// waist
double Waist2ArmZ=0.2694;
double Waist2RArmY=-0.239;
double Waist2LArmY=0.239;
//double WaistYaw=0; double WaistPitch=0;
//MatrixXd R_shoulder(3,3);
//VectorXd r_left_shoulder(3);
//VectorXd r_right_shoulder(3);
//    R_shoulder<<cos(WaistYaw)*cos(WaistPitch), -sin(WaistYaw), cos(WaistYaw)*sin(WaistPitch),
//            cos(WaistPitch)*sin(WaistYaw),  cos(WaistYaw), sin(WaistYaw)*sin(WaistPitch),
//            -sin(WaistPitch),              0,               cos(WaistPitch);
//    r_left_shoulder<< Waist2ArmZ*cos(WaistYaw)*sin(WaistPitch) - Waist2LArmY*sin(WaistYaw),
//                      Waist2LArmY*cos(WaistYaw) + Waist2ArmZ*sin(WaistYaw)*sin(WaistPitch)-Waist2LArmY,
//                                                               Waist2ArmZ*cos(WaistPitch)-Waist2ArmZ;
//    r_right_shoulder<< Waist2ArmZ*cos(WaistYaw)*sin(WaistPitch) - Waist2RArmY*sin(WaistYaw),
//                       Waist2RArmY*cos(WaistYaw) + Waist2ArmZ*sin(WaistYaw)*sin(WaistPitch)-Waist2RArmY,
//                                                                Waist2ArmZ*cos(WaistPitch)-Waist2ArmZ;
MatrixXd rightshoulder2waist(double WaistYaw, double WaistPitch){
    MatrixXd T(4,4);
    MatrixXd R(3,3);
    VectorXd P(3);
    R<<cos(WaistYaw)*cos(WaistPitch), -sin(WaistYaw), cos(WaistYaw)*sin(WaistPitch),
            cos(WaistPitch)*sin(WaistYaw),  cos(WaistYaw), sin(WaistYaw)*sin(WaistPitch),
            -sin(WaistPitch),              0,               cos(WaistPitch);

    P<< Waist2ArmZ*cos(WaistYaw)*sin(WaistPitch) - Waist2RArmY*sin(WaistYaw),
            Waist2RArmY*cos(WaistYaw) + Waist2ArmZ*sin(WaistYaw)*sin(WaistPitch)-Waist2RArmY,
            Waist2ArmZ*cos(WaistPitch)-Waist2ArmZ;

    T<<R.transpose(),-R.transpose()*P,0,0,0,1;

    return T;
}
MatrixXd leftshoulder2waist(double WaistYaw, double WaistPitch){
    MatrixXd T(4,4);
    MatrixXd R(3,3);
    VectorXd P(3);
    R<<cos(WaistYaw)*cos(WaistPitch), -sin(WaistYaw), cos(WaistYaw)*sin(WaistPitch),
            cos(WaistPitch)*sin(WaistYaw),  cos(WaistYaw), sin(WaistYaw)*sin(WaistPitch),
            -sin(WaistPitch),              0,               cos(WaistPitch);

    P<< Waist2ArmZ*cos(WaistYaw)*sin(WaistPitch) - Waist2LArmY*sin(WaistYaw),
            Waist2LArmY*cos(WaistYaw) + Waist2ArmZ*sin(WaistYaw)*sin(WaistPitch)-Waist2LArmY,
            Waist2ArmZ*cos(WaistPitch)-Waist2ArmZ;                                                                1;
    T<<R.transpose(),-R.transpose()*P,0,0,0,1;
    return T;
}

int main(int argc, char **argv)
{



    qc_initial_bool=true;


    ros::init(argc, argv, "head");
    ros::NodeHandle nh;
    ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",100);
    ros::Rate loop_rate(200);
    ros::Subscriber qcinit = nh.subscribe("/surena/inc_joint_state", 1000, qc_initial);
    ros::Subscriber object_sub=nh.subscribe("/ai/object", 1000, object_detect);
    ros::Subscriber face_sub=nh.subscribe("/ai/face", 1000, face_detect);
    std_msgs::Int32MultiArray msg;
    std_msgs::MultiArrayDimension msg_dim;

    msg_dim.label = "joint_position";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);

    head_yaw_pid.Init(.005,-45,45,.01,0,0);
    head_pitch_pid.Init(.005,-15,15,.01,0,0);
double time=0;

//QTime chronometer;
//chronometer.start();
    while (ros::ok())
    {

   // chronometer.restart();


//        if (qc_initial_bool) {
//            ROS_INFO_ONCE("qc is initializing!");
//            ros::spinOnce();
//            continue;
//        }


//head_yaw=head_yaw_pid.Calculate(320,X_face);
//head_pitch=head_pitch_pid.Calculate(200,Y_face);
        if(n!=0){
            head_yaw+=.0004*(320-X_face);
            head_pitch-=.0004*(200-Y_face);
        }
        else{
           // head_yaw-=.005*head_yaw;
            //head_pitch-=.005*head_pitch;
        }

        head_yaw=saturate(head_yaw,-45,45);
        head_pitch=saturate(head_pitch,-15,15);


//qDebug()<<"X_face:  "<<X_face<<"\tY_face:  "<<Y_face;
//numplot(head_yaw,-45,45);
//numplot(head_pitch,-15,15);


int head_yaw_motor,head_pitch_motor;
head_pitch_motor=int(head_pitch/30*342);
head_yaw_motor=int(head_yaw/30*342);

//head_pitch_motor=int(20*sin(time/10)/30*342);
//head_yaw_motor=int(20*sin(time/10)/30*342);

//int hand_yaw_motor=int((20*sin(time))*(4000-2050)/(23));


//qDebug()<<head_pitch_motor<<"\t"<<head_yaw_motor;
    msg.data.clear();
    for(int  i = 0;i < 12;i++)
    {
        msg.data.push_back(qc_offset[i]);
    }
    //right hand epose
    msg.data.push_back(qra_offset[0]);//12 -y  a,z
    msg.data.push_back(qra_offset[1]);//13 +x
    msg.data.push_back(qra_offset[2]);//14 -z
    msg.data.push_back(qra_offset[3]);//15 +y
    //right hand dynamixel + fingers
    msg.data.push_back(0);//16
    msg.data.push_back(0);//17
    msg.data.push_back(0);//18
    msg.data.push_back(0);//19
    //left hand epose
    msg.data.push_back(qla_offset[0]);//20 +y
    msg.data.push_back(qla_offset[1]);//21 +x
    msg.data.push_back(qla_offset[2]);//22 -z
    msg.data.push_back(qla_offset[3]);//23 -y
    //left hand dynamixel + fingers
    msg.data.push_back(0);//24
    msg.data.push_back(0);//25
    msg.data.push_back(0);//26
    msg.data.push_back(0);//27
    msg.data.push_back(0);//waist pitch 28
    msg.data.push_back(0);//waist yaw 29
    msg.data.push_back(head_pitch_motor);//head pitch 30
    msg.data.push_back(0);//head roll 31
    msg.data.push_back(head_yaw_motor);//head yaw 32


    chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    time+=.005;
}


return 0;
}
