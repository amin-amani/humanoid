#include "ros/ros.h"
#include "std_msgs/String.h"
#include"Eigen/Dense"
#include <vector>
#include <iostream>
#include <QString>
#include <QList>
#include "Robot.h"
#include"taskspaceonline3.h"
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
#include<geometry_msgs/Wrench.h>
#include<math.h>
#include<sensor_msgs/Imu.h>
#include<std_msgs/Float64.h>
#include "qcgenerator.h"
#include<termios.h>
#include<gazebo_msgs/LinkStates.h>
#include<sensor_msgs/JointState.h>
#include"pidcontroller.h"


using namespace  std;
using namespace  Eigen;

bool left_first=true;//right support in first step
bool backward=false;
bool turning=false;
double TurningRadius=.5;//for on spot .01;
bool sidewalk=false;
int bump_threshold=75;//85;
bool simulation=false;
bool AnkleZAdaptation=!false;
bool LogDataSend=false;

double k_pitch=1.5;//1;0.8;
double pelvis_roll_range=2.5;


double saturate(double a, double min, double max){
    if(a<min){return min;ROS_INFO("subceeding!");}
    else if(a>max){return max;ROS_INFO("exceeding!");}
    else{return a;}
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

double move_rest_back(double max,double t_local,double T_start ,double T_move,double T_rest,double T_back){
    double c3=(10*max)/pow(T_move,3);
    double c4=-(15*max)/pow(T_move,4);
    double c5=(6*max)/pow(T_move,5);
    double c3_r=(10*max)/pow(T_back,3);
    double c4_r=-(15*max)/pow(T_back,4);
    double c5_r=(6*max)/pow(T_back,5);
    double T_end=T_start+T_move+T_rest+T_back;
    double theta=0;
    if(t_local<T_start){theta=0;}
    else if (t_local<T_start+T_move){theta=c3*pow(t_local-T_start,3)+c4*pow(t_local-T_start,4)+c5*pow(t_local-T_start,5);}
    else if (t_local<T_start+T_move+T_rest){theta=max;}
    else if (t_local<T_start+T_move+T_rest+T_back){theta=c3_r*pow(T_end-t_local,3)+c4_r*pow(T_end-t_local,4)+c5_r*pow(T_end-t_local,5);}
    return theta;
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

MinimumJerkInterpolation CoefGen;

//data of left foot sensor
int a,b,c,d,e,f,g,h;

//angles of ankle adaptation

double AnkleZR,AnkleZL;
double AnkleZ_offsetR=0;
double AnkleZ_offsetL=0;


int qc_offset[12];
bool qc_initial_bool;

//

int qra_offset[4];
int qla_offset[4];

int inc_feedback[12];
void qc_initial(const sensor_msgs::JointState & msg){

    if (qc_initial_bool){

        for (int i = 0; i < 12; ++i) {
            qc_offset[i]=int(msg.position[i+1]);

        }

        for (int i = 12; i < 16; ++i){
            qra_offset[i-12]=int(msg.position[i+1]);
        }

        for (int i = 16; i < 20; ++i){
            qla_offset[i-16]=int(msg.position[i+1]);
        }


        qc_initial_bool=false;

        ROS_INFO("Offset=%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\nInitialized!",
                 qc_offset[0],qc_offset[1],qc_offset[2],qc_offset[3],qc_offset[4],
                qc_offset[5],qc_offset[6],qc_offset[7],qc_offset[8],qc_offset[9],
                qc_offset[10],qc_offset[11]);}
}

int abs_feedback[12];
void abs_feedback_func(const sensor_msgs::JointState & msg){
    for (int i = 0; i < 12; ++i) {
        abs_feedback[i]=int(msg.position[i+1]);
    }
}


MatrixXd quater2rot(double w,double x,double y, double z){
    MatrixXd R(3,3);
    R<<w*w+x*x-y*y-z*z,2*x*y-2*w*z,2*x*z+2*w*y,
            2*x*y+2*w*z,w*w-x*x+y*y-z*z,2*y*z-2*w*x,
            2*x*z-2*w*y,2*y*z+2*w*x,w*w-x*x-y*y+z*z;
    return R;
}

//*****quaternion to euler params in ankle
double quaternion2euler_pitch(double q0,double q1,double q2,double q3){
    double R11,R32,R33,R31,theta;
    R31=2*(q1*q3-q0*q2);
    R32=2*(q0*q1+q2*q3);
    R33=q0*q0-q1*q1-q2*q2+q3*q3;
    theta=atan2(-R31,sqrt(R32*R32+R33*R33));
    return theta;
}

double quaternion2euler_roll(double q0,double q1,double q2,double q3){
    double phi,R33,R32;
    R32=2*(q0*q1+q2*q3);
    R33=q0*q0-q1*q1-q2*q2+q3*q3;
    phi=atan2(R32,R33);
    return phi;
}


double GlobalTime;


int bump_pushed[8];
int bump_notpushed[8];
bool bump_initialize;
void receiveFootSensor(const std_msgs::Int32MultiArray& msg)
{
    if (msg.data.size()!=8) {
        qDebug("the size of sensor data is in wrong");
        return;
    }
    if(bump_initialize){
        for (int i = 0; i < 8; ++i) {
            bump_pushed[i]=msg.data[i];
        }
        bump_initialize=false;
    }
    //ROS_INFO("I heard: [%d  %d %d %d %d  %d %d %d]", (int)msg.data[0],(int)msg.data[1],(int)msg.data[2],(int)msg.data[3],(int)msg.data[4],(int)msg.data[5],(int)msg.data[6],(int)msg.data[7]);
    int temp[8];

//    bump_pushed[0]=1094;bump_pushed[1]= 844;bump_pushed[2]=3129;bump_pushed[3]=3005;
//    bump_pushed[4]=3126;bump_pushed[5]=2920;bump_pushed[6]=1212;bump_pushed[7]=920;
//    bump_notpushed[0]=1012;bump_notpushed[1]= 931;bump_notpushed[2]=3037;bump_notpushed[3]=3098;
//    bump_notpushed[4]=3042;bump_notpushed[5]=3009;bump_notpushed[6]=1120;bump_notpushed[7]=1015;

    bump_pushed[0]=1098;bump_pushed[1]= 837;bump_pushed[2]=3129;bump_pushed[3]=3000;
    bump_pushed[4]=3119;bump_pushed[5]=2907;bump_pushed[6]=1205;bump_pushed[7]=916;
    bump_notpushed[0]=1018;bump_notpushed[1]= 930;bump_notpushed[2]=3033;bump_notpushed[3]=3098;
    bump_notpushed[4]=3042;bump_notpushed[5]=3005;bump_notpushed[6]=1115;bump_notpushed[7]=1015;

    temp[0]=msg.data[0]-bump_notpushed[0];
    temp[1]=-1*(msg.data[1]-bump_notpushed[1]);
    temp[2]=msg.data[2]-bump_notpushed[2];
    temp[3]=-1*(msg.data[3]-bump_notpushed[3]);

    //normalizing data of sensors
    a=temp[0]*(100.0/(bump_pushed[0]-bump_notpushed[0]));
    b=temp[1]*(100.0/(bump_notpushed[1]-bump_pushed[1]));
    c=temp[2]*(100.0/(bump_pushed[2]-bump_notpushed[2]));
    d=temp[3]*(100.0/(bump_notpushed[3]-bump_pushed[3]));

    //ROS_INFO("I heard a b c d: [%d  %d %d %d]", a,b,c,d);

    temp[4]=msg.data[4]-bump_notpushed[4];
    temp[5]=-1*(msg.data[5]-bump_notpushed[5]);
    temp[6]=msg.data[6]-bump_notpushed[6];
    temp[7]=-1*(msg.data[7]-bump_notpushed[7]);

    //normalizing data of sensors
    e=temp[4]*(100.0/(bump_pushed[4]-bump_notpushed[4]));
    f=temp[5]*(100.0/(bump_notpushed[5]-bump_pushed[5]));
    g=temp[6]*(100.0/(bump_pushed[6]-bump_notpushed[6]));
    h=temp[7]*(100.0/(bump_notpushed[7]-bump_pushed[7]));


    if (a<0){a=0;} if (b<0){b=0;} if (c<0){c=0;} if (d<0){d=0;}
    if (e<0){e=0;} if (f<0){f=0;} if (g<0){g=0;} if (h<0){h=0;}
    //qDebug()<<"a:"<<a<<"\tb:"<<b<<"\tc:"<<c<<"\td:"<<d<<"\ne:"<<e<<"\tf:"<<f<<"\tg:"<<g<<"\th:"<<h;

//cout<<msg.data[0]<<"\t"<<msg.data[1]<<"\t"<<msg.data[2]<<"\t"<<msg.data[3]<<"\t"<<msg.data[4]
//               <<"\t"<<msg.data[5]<<"\t"<<msg.data[6]<<"\t"<<msg.data[7]<<endl;

}


double teta_motor_L=0;
double teta_motor_R=0;//pitch
double phi_motor_L=0;
double phi_motor_R=0;//roll

PIDController teta_PID_L;
PIDController teta_PID_R;
PIDController phi_PID_L;
PIDController phi_PID_R;

void ankleAdaptation(){
    //parameters of ankle adaptation
    double k1,k2,k3,k4;
    //k1=0.000015;    k2=0.000015;    k3=0.000015;    k4=0.000015;
    k1=0.00004;0.0000215;
    k2=0.00004;0.0000215;
    k3=0.00004;0.0000215;
    k4=0.00004;0.0000215;

    double threshold=4;
    double threshold2=70;

    if(a>threshold2 &&b>threshold2&&c>threshold2&&d>threshold2){

    }
    else if(a<threshold &&b<threshold &&c<threshold &&d<threshold){
      //  theta_motor_L->0,phi_motor_L->0
        teta_motor_L+=phi_PID_L.Calculate(0,teta_motor_L);
        phi_motor_L+=phi_PID_L.Calculate(0,phi_motor_L);
    }

    else if (a>threshold ||b>threshold ||c>threshold ||d>threshold){//left
        //-----------------Pitch left ankle motor control---------------//
        if (abs(b-a)>=abs(c-d)) {
            if (abs(a-b)<120) {teta_motor_L=1*teta_motor_L+k1*(a-b);}
        }
        else {
            if (abs(d-c)<120) {teta_motor_L=1*teta_motor_L+k1*(d-c);}
        }

        //----------------Roll left ankle motor control---------------//
        if (abs(c-b)>=abs(d-a)) {
            if (abs(c-b)<120) {phi_motor_L=1*phi_motor_L+k2*(c-b);}
        }
        else {
            if (abs(a-d)<120) {phi_motor_L=1*phi_motor_L+k2*(d-a);}
        }
    }

    if(e>threshold2 &&f>threshold2&&g>threshold2&&h>threshold2){

    }
    else if(e<threshold &&f<threshold &&g<threshold &&h<threshold){
      //  theta_motor_L->0,phi_motor_L->0
        teta_motor_R+=phi_PID_R.Calculate(0,teta_motor_R);
        phi_motor_R+=phi_PID_R.Calculate(0,phi_motor_R);
    }
    else if(e>threshold ||f>threshold ||g>threshold ||h>threshold){//right

        //-----------------Pitch left ankle motor control---------------//
        if (abs(f-e)>=abs(g-h)) {
            if (abs(e-f)<120) {teta_motor_R=1*teta_motor_R+k3*(e-f);}
        }
        else {
            if (abs(h-g)<120) {teta_motor_R=1*teta_motor_R+k3*(h-g);}
        }


        //----------------Roll left ankle motor control---------------//
        if (abs(g-f)>=abs(h-e)) {
            if (abs(g-f)<120) {phi_motor_R=1*phi_motor_R+k4*(g-f);}
        }
        else {
            if (abs(e-h)<120) {phi_motor_R=1*phi_motor_R+k4*(h-e);}
        }

}

        //------------------------saturation of ankle motors----------------------------//
    phi_motor_L=saturate(phi_motor_L,-M_PI/18,M_PI/18);
    phi_motor_R=saturate(phi_motor_R,-M_PI/18,M_PI/18);
    teta_motor_L=saturate(teta_motor_L,-M_PI/18,M_PI/18);
    teta_motor_R=saturate(teta_motor_R,-M_PI/18,M_PI/18);
//        if ((abs(phi_motor_L))>0.9) {phi_motor_L=0.9;}
//        if ((abs(teta_motor_L))>0.9) {teta_motor_L=0.9;}
//        if ((abs(phi_motor_R))>0.9) {phi_motor_L=0.9;}
//        if ((abs(teta_motor_R))>0.9) {teta_motor_R=0.9;}

        qDebug()<<"a:"<<a<<"\tb:"<<b<<"\tc:"<<c<<"\td:"<<d<<"\ne:"<<e<<"\tf:"<<f<<"\tg:"<<g<<"\th:"<<h;

        qDebug()<<"teta_motor_L:"<<teta_motor_L<<"phi_motor_L:"<<phi_motor_L<<
"\nteta_motor_R:"<<teta_motor_R<<"phi_motor_R:"<<phi_motor_R;
}


int main(int argc, char **argv)
{
double kp=.02;
    teta_PID_L.Init(.005,.9,-.9,kp,0,0);
    teta_PID_R.Init(.005,.9,-.9,kp,0,0);
    phi_PID_L.Init(.005,.9,-.9,kp,0,0);
    phi_PID_R.Init(.005,.9,-.9,kp,0,0);


    qc_initial_bool=!simulation;
    bump_initialize=false;

    vector<double> cntrl(13);
    QCgenerator QC;
    for (int i = 0; i < 12; ++i) {
        qc_offset[i]=0;
    }

    GlobalTime=0;

    //*******************This part of code is for initialization of joints of the robot for walking**********************************
    int count = 0;

    ros::init(argc, argv, "myNode");
    ros::NodeHandle nh;
    ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",100);
    ros::Publisher  contact_flag  = nh.advertise<std_msgs::Int32MultiArray>("contact_flag_timing",100);
    ros::Publisher  trajectory_data_pub  = nh.advertise<std_msgs::Float32MultiArray>("my_trajectory_data",100);
    std_msgs::Float32MultiArray trajectory_data;

    ros::Subscriber sub = nh.subscribe("/surena/bump_sensor_state", 1000, receiveFootSensor);
    ros::Subscriber qcinit = nh.subscribe("/surena/inc_joint_state", 1000, qc_initial);

    ros::Rate loop_rate(200);
    std_msgs::Int32MultiArray msg;
    std_msgs::MultiArrayDimension msg_dim;

    msg_dim.label = "joint_position";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);


    ROS_INFO("press any key to start!");
    getch();
    ROS_INFO("started!");



    while (ros::ok())
    {

        if (qc_initial_bool) {
            ROS_INFO_ONCE("qc is initializing!");
            ros::spinOnce();
            continue;
        }
        ankleAdaptation();
        for (int var = 0; var < 13; ++var) {
           cntrl[var]=0.0;
        }
        cntrl[5]=teta_motor_R;
        cntrl[6]=phi_motor_R;
        cntrl[11]=teta_motor_L;
        cntrl[12]=phi_motor_L;


        vector<int> qref(12);
        qref=QC.ctrldata2qc(cntrl);

        msg.data.clear();


            for(int  i = 0;i < 12;i++)
            {
                msg.data.push_back(qref[i]+qc_offset[i]);
            }

                for(int  i = 12;i < 29;i++)
                {
                    msg.data.push_back(0);
                }

            chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;

    }



    return 0;
}


