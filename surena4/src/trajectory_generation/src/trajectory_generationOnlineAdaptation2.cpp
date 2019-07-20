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
double TurningRadius=1;//for on spot .01;
bool sidewalk=false;
int bump_threshold=75;//85;
bool simulation=false;
bool AnkleZAdaptation=!false;
bool LogDataSend=false;
double ankle_adaptation_switch=0;// 1 for activating adaptation 0 for siktiring adaptation
double k_pitch=0;//.8;
double pelvis_roll_range=2.5;
bool waist_pitch_bool=false;



ros::Publisher pub1; ros::Publisher pub2; ros::Publisher pub3; ros::Publisher pub4;
ros::Publisher pub5; ros::Publisher pub6; ros::Publisher pub7; ros::Publisher pub8;
ros::Publisher pub9; ros::Publisher pub10; ros::Publisher pub11; ros::Publisher pub12;
ros::Publisher pub13; ros::Publisher pub14; ros::Publisher pub15; ros::Publisher pub16;
ros::Publisher pub17; ros::Publisher pub18; ros::Publisher pub19; ros::Publisher pub20;
ros::Publisher pub21; ros::Publisher pub22; ros::Publisher pub23; ros::Publisher pub24;
ros::Publisher pub25; ros::Publisher pub26; ros::Publisher pub27; ros::Publisher pub28;
ros::Publisher pub29; ros::Publisher pub30; ros::Publisher pub31;

double Mxl=0.0;
double Myl=0.0;
double Myr=0.0;
double Mxr=0.0;
VectorXd CoPR(2);
VectorXd CoPL(2);
double fzl=0.0;
double fzr=0.0;

MinimumJerkInterpolation CoefGen;

//data of left foot sensor
int a,b,c,d,e,f,g,h;

double PitchModified;

double AnkleZR,AnkleZL;
double AnkleZ_offsetR=0;
double AnkleZ_offsetL=0;


int qc_offset[12];
bool qc_initial_bool;

//
int bump_pushed[8];
int bump_notpushed[8];
bool bump_initialize;

int qra_offset[4];
int qla_offset[4];

int inc_feedback[12];
int abs_feedback[2];

double pelvis_orientation_pitch,pelvis_orientation_roll,pelvis_orientation_yaw;
double pelvis_acceleration[3];
double pelvis_angularVelocity[3];

Robot SURENA;//model of robot & kinematics funcs(IK & FK)
Robot SURENA_turning_side;
TaskSpaceOnline3 OnlineTaskSpace;
QList<LinkM> links;
MatrixXd PoseRoot;//position of pelvis respected to global coordinate
MatrixXd PoseRFoot;//position of right ankle joint respected to global coordinate
MatrixXd PoseLFoot;//position of left ankle joint respected to global coordinate
double GlobalTime;
double  DurationOfStartPhase;
double  DurationOfendPhase;
double shoulderPitchOffset;
MatrixXd CoefZStart;

double teta_motor_L=0;
double teta_motor_R=0;//pitch
double phi_motor_L=0;
double phi_motor_R=0;//roll

PIDController teta_PID_L;
PIDController teta_PID_R;
PIDController phi_PID_L;
PIDController phi_PID_R;

double k1=0.00004;
double k2=0.00004;
double k3=0.00004;
double k4=0.00004;

double threshold=4;
double threshold2=100;

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



void  SendGazebo(QList<LinkM> links,MatrixXd RollModifieds, double PitchModifieds, double theta_r, double phi_r, double theta_l, double phi_l){
    if(links.count()<28){qDebug()<<"index err";return;}
    std_msgs::Float64 data;

    data.data=links[1].JointAngle;    pub1.publish(data);
    data.data=links[2].JointAngle+RollModifieds(0,0);    pub2.publish(data);
    data.data=links[3].JointAngle+PitchModifieds;    pub3.publish(data);
    data.data=links[4].JointAngle;    pub4.publish(data);
    // data.data=links[5].JointAngle+theta_r;    pub5.publish(data);
    data.data=saturate(links[5].JointAngle,-M_PI/5.4,M_PI/4)+theta_r;    pub5.publish(data);
    data.data=links[6].JointAngle+phi_r;    pub6.publish(data);
    data.data=links[7].JointAngle;    pub7.publish(data);
    data.data=links[8].JointAngle+RollModifieds(1,0);    pub8.publish(data);
    data.data=links[9].JointAngle+PitchModifieds;    pub9.publish(data);
    data.data=links[10].JointAngle;    pub10.publish(data);
    //data.data=links[11].JointAngle+theta_l;    pub11.publish(data);
    data.data=saturate(links[11].JointAngle,-M_PI/5.4,M_PI/4)+theta_l;    pub11.publish(data);
    data.data=links[12].JointAngle+phi_l;    pub12.publish(data);
    data.data=links[13].JointAngle;    pub13.publish(data);
    data.data=links[14].JointAngle;    pub14.publish(data);
    data.data=links[15].JointAngle;    pub15.publish(data);
    data.data=links[16].JointAngle;    pub16.publish(data);
    data.data=links[17].JointAngle;    pub17.publish(data);
    data.data=links[18].JointAngle;    pub18.publish(data);
    data.data=links[19].JointAngle;    pub19.publish(data);
    data.data=links[20].JointAngle;    pub20.publish(data);
    data.data=links[21].JointAngle;    pub21.publish(data);
    data.data=links[22].JointAngle;    pub22.publish(data);
    data.data=links[23].JointAngle;    pub23.publish(data);
    data.data=links[24].JointAngle;    pub24.publish(data);
    data.data=links[25].JointAngle;    pub25.publish(data);
    data.data=links[26].JointAngle;    pub26.publish(data);
    data.data=links[27].JointAngle;    pub27.publish(data);
    data.data=links[28].JointAngle;    pub28.publish(data);
    data.data=0;    pub29.publish(data);
    data.data=0;    pub30.publish(data);
    data.data=0;    pub31.publish(data);
}

void  SendGazebo_reverse(QList<LinkM> links,MatrixXd RollModifieds, double PitchModifieds, double theta_r, double phi_r, double theta_l, double phi_l){
    if(links.count()<28){qDebug()<<"index err";return;}
    std_msgs::Float64 data;

    data.data=-links[7].JointAngle;    pub1.publish(data);
    data.data=-links[8].JointAngle+RollModifieds(0,0);    pub2.publish(data);
    data.data=links[9].JointAngle+PitchModifieds;    pub3.publish(data);
    data.data=links[10].JointAngle;    pub4.publish(data);
    // data.data=links[5].JointAngle+theta_r;    pub5.publish(data);
    data.data=saturate(links[11].JointAngle,-M_PI/5.4,M_PI/4)+theta_r;    pub5.publish(data);
    data.data=-links[12].JointAngle+phi_r;    pub6.publish(data);
    data.data=-links[1].JointAngle;    pub7.publish(data);
    data.data=-links[2].JointAngle+RollModifieds(1,0);    pub8.publish(data);
    data.data=links[3].JointAngle+PitchModifieds;    pub9.publish(data);
    data.data=links[4].JointAngle;    pub10.publish(data);
    //data.data=links[11].JointAngle+theta_l;    pub11.publish(data);
    data.data=saturate(links[5].JointAngle,-M_PI/5.4,M_PI/4)+theta_l;    pub11.publish(data);
    data.data=-links[6].JointAngle+phi_l;    pub12.publish(data);

    data.data=links[13].JointAngle;  pub13.publish(data);
    data.data=links[14].JointAngle;  pub14.publish(data);

    data.data=links[15].JointAngle; pub22.publish(data);
    data.data=-links[16].JointAngle; pub23.publish(data);
    data.data=-links[17].JointAngle; pub24.publish(data);
    data.data=links[18].JointAngle; pub25.publish(data);
    data.data=-links[19].JointAngle; pub26.publish(data);
    data.data=-links[20].JointAngle; pub27.publish(data);
    data.data=links[21].JointAngle; pub28.publish(data);

    data.data=links[22].JointAngle; pub15.publish(data);
    data.data=-links[23].JointAngle; pub16.publish(data);
    data.data=-links[24].JointAngle; pub17.publish(data);
    data.data=links[25].JointAngle; pub18.publish(data);
    data.data=-links[26].JointAngle; pub19.publish(data);
    data.data=-links[27].JointAngle; pub20.publish(data);
    data.data=links[28].JointAngle; pub21.publish(data);

    data.data=0;    pub29.publish(data);
    data.data=0;    pub30.publish(data);
    data.data=0;    pub31.publish(data);
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

    // ROS_INFO("I heard e f g h: [%d  %d %d %d]", e,f,g,h);

    //deleting data with negative sign
    if (a<0){a=0;} if (b<0){b=0;} if (c<0){c=0;} if (d<0){d=0;}
    if (e<0){e=0;} if (f<0){f=0;} if (g<0){g=0;} if (h<0){h=0;}
}



void qc_initial(const sensor_msgs::JointState & msg){
    //    for (int i = 0; i < 12; ++i) {
    //        inc_feedback[i]=int(msg.position[i+1]);
    //    }


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


void abs_feedback_func(const sensor_msgs::JointState & msg){
    for (int i = 0; i < 12; ++i) {
        abs_feedback[i]=int(msg.position[i+1]);
    }
}



void FT_left_feedback(const geometry_msgs::Wrench &msg){
fzl=msg.force.z;
    Mxl=-msg.torque.y;
    Myl=msg.torque.x;
}

void FT_right_feedback(const geometry_msgs::Wrench &msg){
fzr=msg.force.z;
 Mxr=-msg.torque.x;
 Myr=-msg.torque.y;
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




void imu_data_process(const sensor_msgs::Imu &msg){
    //MatrixXd R_pelvis(3,3);
    //R_pelvis=quater2rot(msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z);
    pelvis_orientation_roll=msg.orientation.x;
    pelvis_orientation_pitch=msg.orientation.y;
    pelvis_orientation_yaw=msg.orientation.z;
    pelvis_acceleration[0]=msg.linear_acceleration.x;
    pelvis_acceleration[1]=msg.linear_acceleration.y;
    pelvis_acceleration[2]=msg.linear_acceleration.z;
    pelvis_angularVelocity[0]=msg.angular_velocity.x;
    pelvis_angularVelocity[1]=msg.angular_velocity.y;
    pelvis_angularVelocity[2]=msg.angular_velocity.z;
    //qDebug()<<pelvis_orientation_roll;
}


void ankle_states(const gazebo_msgs::LinkStates& msg){
    double x_left, x_right,y_left, y_right,z_left, z_right;
    Vector3d vec_A_E;
    Vector3d vec_B_F;
    Vector3d vec_C_G;
    Vector3d vec_D_H;

    vec_A_E<<-.085,
            -.08,
            -.11;
    vec_B_F<<.135,
            -.08,
            -.11;
    vec_C_G<<.135,
            .08,
            -.11;
    vec_D_H<<-.085,
            .08,
            -.11;

    x_left=msg.pose[7].position.x;
    x_right=msg.pose[13].position.x;
    y_left=msg.pose[7].position.y;
    y_right=msg.pose[13].position.y;
    z_left=msg.pose[7].position.z;
    z_right=msg.pose[13].position.z;

    MatrixXd R_left(3,3);
    MatrixXd R_right(3,3);
    R_left=quater2rot(msg.pose[7].orientation.w,msg.pose[7].orientation.x,msg.pose[7].orientation.y,msg.pose[7].orientation.z);
    R_right=quater2rot(msg.pose[13].orientation.w,msg.pose[13].orientation.x,msg.pose[13].orientation.y,msg.pose[13].orientation.z);
    Vector3d temp;
    temp=R_left*vec_A_E;    //A=temp(2)+z_left;
    a=int(((.02-temp(2)-z_left)+abs(.02-temp(2)-z_left))*5000/2);
    temp=R_left*vec_B_F;    //B=temp(2)+z_left;
    b=int(((.02-temp(2)-z_left)+abs(.02-temp(2)-z_left))*5000/2);
    temp=R_left*vec_C_G;    //C=temp(2)+z_left;
    c=int(((.02-temp(2)-z_left)+abs(.02-temp(2)-z_left))*5000/2);
    temp=R_left*vec_D_H;    //D=temp(2)+z_left;
    d=int(((.02-temp(2)-z_left)+abs(.02-temp(2)-z_left))*5000/2);
    temp=R_right*vec_A_E;    //E=temp(2)+z_right;
    e=int(((.02-temp(2)-z_right)+abs(.02-temp(2)-z_right))*5000/2);
    temp=R_right*vec_B_F;    //F=temp(2)+z_right;
    f=int(((.02-temp(2)-z_right)+abs(.02-temp(2)-z_right))*5000/2);
    temp=R_right*vec_C_G;    //G=temp(2)+z_right;
    g=int(((.02-temp(2)-z_right)+abs(.02-temp(2)-z_right))*5000/2);
    temp=R_right*vec_D_H;    //H=temp(2)+z_right;
    h=int(((.02-temp(2)-z_right)+abs(.02-temp(2)-z_right))*5000/2);
    //ROS_INFO("z_left=%f  A=%d,B=%d,C=%d,D=%d\tz_right=%f  E=%d,F=%d,G=%d,H=%d",z_left,a,b,c,d,z_right,e,f,g,h);
    //    ROS_INFO("x_left=%f,x_right=%f,y_left=%f,y_right=%fz_left=%f,z_right=%f",x_left,x_right,y_left,y_right,z_left,z_right);
}



void StartPhase(){
    if ( GlobalTime<=DurationOfStartPhase) {

        double zStart=0;
        double yStart=0;
        double xStart=0;
        GlobalTime=GlobalTime+OnlineTaskSpace._timeStep;

        zStart=OnlineTaskSpace.InitialPelvisHeight+move2pose(OnlineTaskSpace.ReferencePelvisHeight-OnlineTaskSpace.InitialPelvisHeight,GlobalTime,0,DurationOfStartPhase);
        PoseRoot<<xStart,yStart,zStart,0,0,0;
        PoseRFoot<<0,
                -0.11500,
                0.112000,
                0,
                0,
                0;

        PoseLFoot<<0,
                0.11500,
                0.11200,
                0,
                0,
                0;
        if(turning||sidewalk){
            SURENA_turning_side.doIK("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot);
            SURENA_turning_side.doIK("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot);
        }
        else{
            SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot);
            SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot);
        }

        shoulderPitchOffset=SURENA.Links[3].JointAngle;

        double D_pitch=-1*OnlineTaskSpace.HipPitchModification*(M_PI/180);
        double TstartofPitchModify=DurationOfStartPhase/6;
        double TendofPitchModify=DurationOfStartPhase;
        if (GlobalTime>=TstartofPitchModify && GlobalTime<=TendofPitchModify){
            PitchModified=move2pose(D_pitch,GlobalTime,TstartofPitchModify,TendofPitchModify);
        }
    }

    if(fabs(GlobalTime-DurationOfStartPhase)<.001){waist_pitch_bool=true;}
}

void EndPhase(){

    if (GlobalTime>=(DurationOfStartPhase+OnlineTaskSpace.MotionTime) && GlobalTime<=DurationOfendPhase+DurationOfStartPhase+OnlineTaskSpace.MotionTime) {

        double zStart=0;
        double yStart=0;
        double xStart=0;
        GlobalTime=GlobalTime+OnlineTaskSpace._timeStep;
        zStart=OnlineTaskSpace.ReferencePelvisHeight+move2pose(OnlineTaskSpace.InitialPelvisHeight-OnlineTaskSpace.ReferencePelvisHeight,GlobalTime,DurationOfStartPhase+OnlineTaskSpace.MotionTime,DurationOfStartPhase+OnlineTaskSpace.MotionTime+DurationOfendPhase);
        PoseRoot<<xStart,yStart,zStart,0,0,0;
        PoseRFoot<<0,
                -0.11500,
                OnlineTaskSpace.currentRightFootZ,
                0,
                0,
                0;
        PoseLFoot<<0,
                0.11500,
                OnlineTaskSpace.currentLeftFootZ,
                0,
                0,
                0;

        SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot);
        SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot);

        links = SURENA.GetLinks();
        double D_pitch=-1*OnlineTaskSpace.HipPitchModification*(M_PI/180);
        double TstartofPitchModify=DurationOfendPhase/6+DurationOfStartPhase+OnlineTaskSpace.MotionTime;
        double TendofPitchModify=DurationOfendPhase*5/6+DurationOfStartPhase+OnlineTaskSpace.MotionTime;
        double D_time=TendofPitchModify-TstartofPitchModify;
        if (GlobalTime>=TstartofPitchModify && GlobalTime<=TendofPitchModify){
            PitchModified=D_pitch-move2pose(D_pitch,GlobalTime,TstartofPitchModify,TendofPitchModify);
        }
    }
}



void ankleOrientationAdaptationLeft(){
    //parameters of ankle adaptation
    if(a>threshold2 &&b>threshold2&&c>threshold2&&d>threshold2){

    }
    else if(a<threshold &&b<threshold &&c<threshold &&d<threshold){
        //  theta_motor_L->0,phi_motor_L->0
        //        teta_motor_L+=phi_PID_L.Calculate(0,teta_motor_L);
        //        phi_motor_L+=phi_PID_L.Calculate(0,phi_motor_L);
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
    phi_motor_L=saturate(phi_motor_L,-M_PI/18,M_PI/18);
    teta_motor_L=saturate(teta_motor_L,-M_PI/18,M_PI/18);
}
void ankleOrientationAdaptationRight(){
    if(e>threshold2 &&f>threshold2&&g>threshold2&&h>threshold2){

    }
    else if(e<threshold &&f<threshold &&g<threshold &&h<threshold){
        //  theta_motor_L->0,phi_motor_L->0
        //        teta_motor_R+=phi_PID_R.Calculate(0,teta_motor_R);
        //        phi_motor_R+=phi_PID_R.Calculate(0,phi_motor_R);
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


    phi_motor_R=saturate(phi_motor_R,-M_PI/18,M_PI/18);
    teta_motor_R=saturate(teta_motor_R,-M_PI/18,M_PI/18);
    //------------------------saturation of ankle motors----------------------------//


    //        qDebug()<<"a:"<<a<<"\tb:"<<b<<"\tc:"<<c<<"\td:"<<d<<"\ne:"<<e<<"\tf:"<<f<<"\tg:"<<g<<"\th:"<<h;

    //        qDebug()<<"teta_motor_L:"<<teta_motor_L<<"phi_motor_L:"<<phi_motor_L<<
    //"\nteta_motor_R:"<<teta_motor_R<<"phi_motor_R:"<<phi_motor_R;
}

int main(int argc, char **argv)
{
    if(turning&&TurningRadius<.2){OnlineTaskSpace.StepLength=TurningRadius/16*M_PI;}

    if(sidewalk||(turning&&TurningRadius<.2)){
        OnlineTaskSpace.Xe=0;
        OnlineTaskSpace.Xs=0;
        OnlineTaskSpace.side_extra_step_length=true;
        OnlineTaskSpace.CoeffArrayPelvis();
        OnlineTaskSpace.CoeffArrayAnkle();
        OnlineTaskSpace.CoeffSideStartEnd();

    }




    qc_initial_bool=!simulation;
    bump_initialize=false;

PIDController pitch4zmp;
double max_waist_pitch=M_PI/18;
double min_waist_pitch=-M_PI/18;
double k_waist_pitch=5e-3;
pitch4zmp.Init( OnlineTaskSpace._timeStep,max_waist_pitch,min_waist_pitch,k_waist_pitch,0,0);

    bool leftzstop=false;
    bool rightzstop=false;

    vector<double> cntrl(13);
    QCgenerator QC;
    for (int i = 0; i < 12; ++i) {
        qc_offset[i]=0;
    }

    double kp=.02;
    teta_PID_L.Init(.005,.9,-.9,kp,0,0);
    teta_PID_R.Init(.005,.9,-.9,kp,0,0);
    phi_PID_L.Init(.005,.9,-.9,kp,0,0);
    phi_PID_R.Init(.005,.9,-.9,kp,0,0);

    PIDController AnkleZR_PID;
    PIDController AnkleZL_PID;

    AnkleZR_PID.Init(.005,.03,-.03,.00001,0,0);
    AnkleZL_PID.Init(.005,.03,-.03,.00001,0,0);

    double footSensorSaturation=75;//if all sensors data are bigger than this amount, this means the foot is landed on the ground
    double footSensorthreshold=4;// will start orientaition correction

    GlobalTime=0;
    DurationOfStartPhase=3;
    DurationOfendPhase=3;

    MatrixXd RollModified(2,1);RollModified<<0,0;//parameters for hip roll angles charge, for keep pelvis straight
    PitchModified=0;
    PoseRoot.resize(6,1); //pelvis trajectory from taskspace_online,xyzrpy
    PoseRFoot.resize(6,1);//right ankle joint trajectory from taskspace_online,xyzrpy
    PoseLFoot.resize(6,1);//left ankle joint trajectory from taskspace_online,xyzrpy

    //*******************This part of code is for initialization of joints of the robot for walking**********************************
    int count = 0;

    ros::init(argc, argv, "myNode");
    ros::NodeHandle nh;
    ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",100);
    ros::Publisher  contact_flag  = nh.advertise<std_msgs::Int32MultiArray>("contact_flag_timing",100);
    ros::Publisher  trajectory_data_pub  = nh.advertise<std_msgs::Float32MultiArray>("my_trajectory_data",100);
    std_msgs::Float32MultiArray trajectory_data;

    ros::Subscriber sub = nh.subscribe("/surena/bump_sensor_state", 1000, receiveFootSensor);
    ros::Subscriber ft_left = nh.subscribe("/surena/ft_l_state",1000,FT_left_feedback);
    ros::Subscriber ft_right = nh.subscribe("/surena/ft_r_state",1000,FT_right_feedback);
    ros::Subscriber qcinit = nh.subscribe("/surena/inc_joint_state", 1000, qc_initial);
      ros::Subscriber abs_sub = nh.subscribe("/surena/abs_joint_state", 1000, abs_feedback_func);
     ros::Subscriber imusub = nh.subscribe("/surena/imu_state", 1000, imu_data_process);
    if(simulation){//gazebo publishers
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
    }
    ros::Subscriber ankleStates = nh.subscribe("/gazebo/link_states", 10, ankle_states);


    int32_t contact_flag_timing=1000;    int32_t contact_flag_sensor=1000;    int32_t contact_flag_sensor2=1000;

    ros::Rate loop_rate(200);
    std_msgs::Int32MultiArray msg;
    std_msgs::Int32MultiArray msg_contact_flag;
    std_msgs::MultiArrayDimension msg_dim;

    msg_dim.label = "joint_position";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);
    msg_contact_flag.data.clear();
    msg_contact_flag.data.push_back(contact_flag_timing);

    links=SURENA.GetLinks();
    OnlineTaskSpace.StepNumber=1;

    MatrixXd P;

    ROS_INFO("press any key to start!");
    getch();
    ROS_INFO("started!");


    bool firstcontactL=true;
    bool firstcontactR=true;
    bool fullcontactL=false;
    bool fullcontactR=false;

    //*****loging data in a file

    QTime pc_time;
    QByteArray data_pose;
    QString name_pose;
    name_pose="/media/milad/UBUNTU1604/robot_data/";
    //       name_pose+=QString::number(pc_time.currentTime().hour())+"_";
    //       name_pose+=QString::number(pc_time.currentTime().minute())+"_";
    //       name_pose+=QString::number(pc_time.currentTime().second())+"_";
    name_pose+="pose.txt";
    QFile myfile_pose(name_pose);



    QTime timer;
    QByteArray data_time;
    QString name_time="/media/milad/UBUNTU1604/robot_data/timecheck.txt";
    QFile myfile_time(name_time);


double waist_pitch=0;


int waist_pitch_counter=0;

    while (ros::ok())
    {

//qDebug()<<fzl<<"\t"<<fzr<<"\t"<<Mxr<<"\t"<<Myr<<"\t"<<Mxl<<"\t"<<Myl;
//qDebug()<<pelvis_orientation_roll;
        //MotionTime=TStart+NStride*2*Tc+TDs+TEnd;

        if (qc_initial_bool) {
            ROS_INFO_ONCE("qc is initializing!");
            ros::spinOnce();
            continue;
        }
         StartPhase();


        if (waist_pitch_bool){
            double zmpx=(Myl+Myr)/(fzl+fzr);
            waist_pitch-=pitch4zmp.Calculate(0,zmpx);
            waist_pitch=saturate(waist_pitch,-M_PI/18,M_PI/18);
            qDebug()<<waist_pitch<<"\t"<<fzl<<"\t"<<fzr<<"\t"<<Myr<<"\t"<<Myl<<"\t"<<zmpx;
            if(fabs(zmpx)>.002){waist_pitch_counter=0;}
            if(fabs(zmpx)<.002){waist_pitch_counter++;}
            if((waist_pitch_counter>20)||(waist_pitch>=max_waist_pitch)||(waist_pitch<=min_waist_pitch)){
                waist_pitch_bool=false;
//                break;
            }

        }



  else{


        if (GlobalTime>DurationOfStartPhase+OnlineTaskSpace.TStart&&GlobalTime<DurationOfStartPhase+OnlineTaskSpace.MotionTime-OnlineTaskSpace.TEnd-OnlineTaskSpace.TDs) {
            double adapt_time=fmod(GlobalTime-DurationOfStartPhase-OnlineTaskSpace.TStart,2*OnlineTaskSpace.Tc);



            if (adapt_time>OnlineTaskSpace.TDs&&adapt_time<OnlineTaskSpace.Tc+OnlineTaskSpace.TDs/2) {
                ankleOrientationAdaptationRight();
            }
            else{
                teta_motor_R+=phi_PID_R.Calculate(0,teta_motor_R);
                phi_motor_R+=phi_PID_R.Calculate(0,phi_motor_R);
            }
            if (adapt_time<OnlineTaskSpace.TDs/2||adapt_time>OnlineTaskSpace.Tc+OnlineTaskSpace.TDs) {
                ankleOrientationAdaptationLeft();
            }
            else{
                teta_motor_L+=phi_PID_L.Calculate(0,teta_motor_L);
                phi_motor_L+=phi_PID_L.Calculate(0,phi_motor_L);
            }
        }
        else{
            teta_motor_R+=phi_PID_R.Calculate(0,teta_motor_R);
            phi_motor_R+=phi_PID_R.Calculate(0,phi_motor_R);
            teta_motor_L+=phi_PID_L.Calculate(0,teta_motor_L);
            phi_motor_L+=phi_PID_L.Calculate(0,phi_motor_L);
        }






        int NumberOfTimeStep=(OnlineTaskSpace.Tc/OnlineTaskSpace._timeStep)+1;

        //-----------------------------------------------------------------------------------------------------//
        //------------------------------- main loop of cyclic walking -----------------------------------------//
        //-----------------------------------------------------------------------------------------------------//

        if (GlobalTime>DurationOfStartPhase && GlobalTime<(DurationOfStartPhase+OnlineTaskSpace.MotionTime)){
            //Ankle Trajectory Replacement
            double m1;//x_al
            double m2;//y_al
            double m3;//z_al
            double m4;//pitch_al
            double m5;//x_ar
            double m6;//y_ar
            double m7;//z_ar
            double m8;//pitch_ar

            GlobalTime=GlobalTime+OnlineTaskSpace._timeStep;

            OnlineTaskSpace.currentLeftFootX2=links[12].PositionInWorldCoordinate(0);
            OnlineTaskSpace.currentLeftFootY2=links[12].PositionInWorldCoordinate(1);
            OnlineTaskSpace.currentLeftFootZ=links[12].PositionInWorldCoordinate(2);

            OnlineTaskSpace.currentRightFootX2=links[6].PositionInWorldCoordinate(0);
            OnlineTaskSpace.currentRightFootY2=links[6].PositionInWorldCoordinate(1);
            OnlineTaskSpace.currentRightFootZ=links[6].PositionInWorldCoordinate(2);



            if ((OnlineTaskSpace.StepNumber==1) && (OnlineTaskSpace.localTiming>=OnlineTaskSpace.TStart) ) {
                OnlineTaskSpace.localTiming=OnlineTaskSpace._timeStep;//0.001999999999000000;
                OnlineTaskSpace.localtimingInteger=1;
                OnlineTaskSpace.StepNumber=OnlineTaskSpace.StepNumber+1;
            }

            else if ((OnlineTaskSpace.localtimingInteger>=NumberOfTimeStep) &&   (OnlineTaskSpace.StepNumber>1    &&   OnlineTaskSpace.StepNumber<(OnlineTaskSpace.NStep+2))) {
                OnlineTaskSpace.StepNumber=OnlineTaskSpace.StepNumber+1;
                OnlineTaskSpace.localTiming=OnlineTaskSpace._timeStep;//0.001999999999000000;
                OnlineTaskSpace.localtimingInteger=1;
            }

            MatrixXd m=OnlineTaskSpace.AnkleTrajectory(OnlineTaskSpace.globalTime,OnlineTaskSpace.StepNumber,OnlineTaskSpace.localTiming);

            m1=m(0,0); m2=m(1,0); m3=m(2,0); m4=m(3,0);
            m5=m(4,0); m6=m(5,0); m7=m(6,0); m8=m(7,0);


            RollModified=OnlineTaskSpace.RollAngleModification(OnlineTaskSpace.globalTime);
            P=OnlineTaskSpace.PelvisTrajectory (OnlineTaskSpace.globalTime);

            OnlineTaskSpace.globalTime=OnlineTaskSpace.globalTime+OnlineTaskSpace._timeStep;
            OnlineTaskSpace.localTiming=OnlineTaskSpace.localTiming+OnlineTaskSpace._timeStep;
            OnlineTaskSpace.localtimingInteger= OnlineTaskSpace.localtimingInteger+1;

            if (OnlineTaskSpace.globalTime<=OnlineTaskSpace.MotionTime){


                //if you want to have modification of height of pelvis please active the Pz(0,0) instead of P(2,0)

                if(!AnkleZAdaptation){AnkleZL=m3;AnkleZR=m7;}
                else{
                    double local_time_cycle=0;
                    if(GlobalTime<DurationOfStartPhase+OnlineTaskSpace.TStart){
                        AnkleZR=m7;
                        if(GlobalTime-DurationOfStartPhase<OnlineTaskSpace.Tx+OnlineTaskSpace.Tc){
                            AnkleZL=m3;
                        }
                        else{

                            if(!leftzstop){
                                if(a>bump_threshold||b>bump_threshold||c>bump_threshold||d>bump_threshold){
                                    leftzstop=true;
                                    AnkleZL=m3;
                                    AnkleZL+=AnkleZ_offsetL;
                                    AnkleZ_offsetL=AnkleZL-OnlineTaskSpace._lenghtOfAnkle;
                                    qDebug()<<"leftzstop=true AnkleZL="<<AnkleZL<<"offset="<<AnkleZ_offsetL;
                                }
                                else{AnkleZL=m3;
                                    AnkleZL+=AnkleZ_offsetL;
                                }
                            }

                        }
                    }
                    else if(GlobalTime<DurationOfStartPhase+OnlineTaskSpace.TGait+OnlineTaskSpace.TDs){
                        local_time_cycle=fmod(GlobalTime-DurationOfStartPhase-OnlineTaskSpace.TStart,2*OnlineTaskSpace.Tc);


                        if(local_time_cycle<=OnlineTaskSpace.TDs+OnlineTaskSpace.TStartofAnkleAdaptation){
                            rightzstop=false;
                            AnkleZR=m7;
                            AnkleZ_offsetR=0;
                        }
                        else if(local_time_cycle<=OnlineTaskSpace.TDs+OnlineTaskSpace.TSS){
                            if(!rightzstop){
                                if(e>bump_threshold||f>bump_threshold||g>bump_threshold||h>bump_threshold){
                                    rightzstop=true;
                                    AnkleZR=m7;
                                    AnkleZR+=AnkleZ_offsetR;
                                    AnkleZ_offsetR=AnkleZR-OnlineTaskSpace._lenghtOfAnkle;
                                    qDebug()<<"rightzstop=true AnkleZR="<<AnkleZR<<"offset="<<AnkleZ_offsetR;
                                    ROS_INFO("I heard a b c d: [%d  %d %d %d],e f g h: [%d  %d %d %d]", a,b,c,d,e,f,g,h);

                                }
                                else{AnkleZR=m7;
                                    AnkleZR+=AnkleZ_offsetR;
                                    if (AnkleZR<=0.132)
                                    {
                                        //AnkleZR-=AnkleZR_PID.Calculate(bump_threshold,(e+f+g+h)/4);
                                    cout<<"AnkleZR_PID"<<AnkleZR_PID.Calculate(bump_threshold,(e+f+g+h)/4)<<endl;
                                    }

                                }
                            }
                        }

                        else if(local_time_cycle<=2*OnlineTaskSpace.Tc-OnlineTaskSpace.TSS+OnlineTaskSpace.TStartofAnkleAdaptation){
                            AnkleZR=OnlineTaskSpace._lenghtOfAnkle+AnkleZ_offsetR-move2pose(AnkleZ_offsetR,local_time_cycle,OnlineTaskSpace.Tc,OnlineTaskSpace.Tc+OnlineTaskSpace.TDs);

                        }


                        if(local_time_cycle<=OnlineTaskSpace.Tc-OnlineTaskSpace.TSS+OnlineTaskSpace.TStartofAnkleAdaptation){
                            AnkleZL=OnlineTaskSpace._lenghtOfAnkle+AnkleZ_offsetL-move2pose(AnkleZ_offsetL,local_time_cycle,0,OnlineTaskSpace.TDs);

                        }
                        else if(local_time_cycle<=OnlineTaskSpace.Tc+OnlineTaskSpace.TDs+OnlineTaskSpace.TStartofAnkleAdaptation){
                            leftzstop=false;
                            AnkleZL=m3;
                            AnkleZ_offsetL=0;
                        }
                        else if(local_time_cycle<=OnlineTaskSpace.Tc+OnlineTaskSpace.TDs+OnlineTaskSpace.TSS){
                            if(!leftzstop){
                                if(a>bump_threshold||b>bump_threshold||c>bump_threshold||d>bump_threshold){
                                    leftzstop=true;

                                    AnkleZL=m3;
                                    AnkleZL+=AnkleZ_offsetL;
                                    AnkleZ_offsetL=AnkleZL-OnlineTaskSpace._lenghtOfAnkle;
                                    qDebug()<<"leftzstop=true AnkleZL="<<AnkleZL<<"offset="<<AnkleZ_offsetR;
                                    ROS_INFO("I heard a b c d: [%d  %d %d %d],e f g h: [%d  %d %d %d]", a,b,c,d,e,f,g,h);

                                }

                                else{AnkleZL=m3;
                                    AnkleZL+=AnkleZ_offsetL;
                                    if (AnkleZL<=0.132)
                                    {
                                    //    AnkleZL-=AnkleZL_PID.Calculate(bump_threshold,(a+b+c+d)/4);
                                    cout<<"AnkleZL_PID"<<AnkleZL_PID.Calculate(bump_threshold,(a+b+c+d)/4)<<endl;
                                    }
                                }
                            }
                        }

                    }
                    else{
                        AnkleZR=m7;
                        AnkleZL=m3;
                    }
                }

                PoseRoot<<P(0,0),
                        P(1,0),
                        P(2,0),// Pz(0,0)
                        0,
                        0,
                        0;

                PoseRFoot<<m5,
                        m6,
                        AnkleZR,
                        0,
                        m8,
                        0;

                PoseLFoot<<m1,
                        m2,
                        AnkleZL,
                        0,
                        m4,
                        0;

                double pitch_ar=m8;
                double pitch_al=m4;

                if(pitch_ar>=0){
                    PoseRFoot(2)=PoseRFoot(2)-(OnlineTaskSpace._lenghtOfAnkle*(1-cos(pitch_ar)))+OnlineTaskSpace.lf*sin(pitch_ar);
                    PoseRFoot(0)=PoseRFoot(0)+(OnlineTaskSpace.lf*(1-cos(pitch_ar)))+OnlineTaskSpace._lenghtOfAnkle*sin(pitch_ar);}
                else{
                    PoseRFoot(2)=PoseRFoot(2)-(OnlineTaskSpace._lenghtOfAnkle*(1-cos(pitch_ar)))-OnlineTaskSpace.lb*sin(pitch_ar);//+lb*sin(pitch_ar)
                    PoseRFoot(0)=PoseRFoot(0)-(OnlineTaskSpace.lb*(1-cos(pitch_ar)))+OnlineTaskSpace._lenghtOfAnkle*sin(pitch_ar);} // +_lenghtOfAnkle*sin(pitch_ar)

                if(pitch_al>=0){
                    PoseLFoot(2)=PoseLFoot(2)-(OnlineTaskSpace._lenghtOfAnkle*(1-cos(pitch_al)))+OnlineTaskSpace.lf*sin(pitch_al);
                    PoseLFoot(0)= PoseLFoot(0)+(OnlineTaskSpace.lf*(1-cos(pitch_al)))+OnlineTaskSpace._lenghtOfAnkle*sin(pitch_al);}
                else{
                    PoseLFoot(2)=PoseLFoot(2)-(OnlineTaskSpace._lenghtOfAnkle*(1-cos(pitch_al)))-OnlineTaskSpace.lb*sin(pitch_al); //+lb*sin(pitch_al)
                    PoseLFoot(0)=PoseLFoot(0)-(OnlineTaskSpace.lb*(1-cos(pitch_al)))+OnlineTaskSpace._lenghtOfAnkle*sin(pitch_al);} //+_lenghtOfAnkle*sin(pitch_al)


                if(backward){
                    double backward_coeff=.5;
                    PoseRoot(0,0)=-backward_coeff*PoseRoot(0,0);
                    PoseLFoot(0,0)=-backward_coeff*PoseLFoot(0,0);
                    PoseRFoot(0,0)=-backward_coeff*PoseRFoot(0,0);
                }


                MatrixXd R_P(3,3);  MatrixXd R_F_L(3,3);    MatrixXd R_F_R(3,3);
                R_P=MatrixXd::Identity(3,3);
                R_F_L=MatrixXd::Identity(3,3);
                R_F_R=MatrixXd::Identity(3,3);
                double pelvis_roll=-(PoseRoot(1,0)/OnlineTaskSpace.YpMax)*pelvis_roll_range*M_PI/180;//3 was good
                R_P<<1,0,0,
                        0,cos(pelvis_roll),-sin(pelvis_roll),
                        0,sin(pelvis_roll),cos(pelvis_roll);


                R_F_L<<cos(PoseLFoot(4)),0,sin(PoseLFoot(4)),
                        0,1,0,
                        -sin(PoseLFoot(4)),0,cos(PoseLFoot(4));

                R_F_R<<cos(PoseRFoot(4)),0,sin(PoseRFoot(4)),
                        0,1,0,
                        -sin(PoseRFoot(4)),0,cos(PoseRFoot(4));

                if(!turning && !sidewalk)
                {


                    SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,R_F_L,"Body", PoseRoot,R_P);
                    SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,R_F_R,"Body", PoseRoot,R_P);

                }

                if(turning){

                    //*****************************

                    double yaw_al,yaw_ar,yaw_p;

                    yaw_p=PoseRoot(0,0)/TurningRadius;
                    double sp=sin(yaw_p);
                    double cp=cos(yaw_p);
                    PoseRoot(0,0)=(TurningRadius-PoseRoot(1,0))*sp;
                    PoseRoot(1,0)=TurningRadius-(TurningRadius-PoseRoot(1,0))*cp;

                    yaw_al=PoseLFoot(0,0)/TurningRadius;
                    double s_al=sin(yaw_al);
                    double c_al=cos(yaw_al);
                    PoseLFoot(0,0)=(TurningRadius-PoseLFoot(1,0))*s_al;
                    PoseLFoot(1,0)=TurningRadius-(TurningRadius-PoseLFoot(1,0))*c_al;

                    yaw_ar=PoseRFoot(0,0)/TurningRadius;
                    double s_ar=sin(yaw_ar);
                    double c_ar=cos(yaw_ar);
                    PoseRFoot(0,0)=(TurningRadius-PoseRFoot(1,0))*s_ar;
                    PoseRFoot(1,0)=TurningRadius-(TurningRadius-PoseRFoot(1,0))*c_ar;

                    R_P<<cos(yaw_p),-sin(yaw_p),0,
                            sin(yaw_p),cos(yaw_p),0,
                            0,0,1;
                    R_F_R<<cos(yaw_ar),-sin(yaw_ar),0,
                            sin(yaw_ar),cos(yaw_ar),0,
                            0,0,1;
                    R_F_L<<cos(yaw_al),-sin(yaw_al),0,
                            sin(yaw_al),cos(yaw_al),0,
                            0,0,1;
                    //*--**********************************************

                    SURENA_turning_side.doIK("LLeg_AnkleR_J6",PoseLFoot,R_F_L,"Body", PoseRoot,R_P);
                    SURENA_turning_side.doIK("RLeg_AnkleR_J6",PoseRFoot,R_F_R,"Body", PoseRoot,R_P);

                }

                if(sidewalk){
                    double side_move_coef=.05/2/OnlineTaskSpace.StepLength;


                    //*****************************
                    PoseRoot(1,0)=PoseRoot(1,0)+side_move_coef*PoseRoot(0,0);
                    PoseRoot(0,0)=0;

                    PoseLFoot(1,0)=PoseLFoot(1,0)+side_move_coef*PoseLFoot(0,0);
                    PoseLFoot(0,0)=0;

                    PoseRFoot(1,0)=PoseRFoot(1,0)+side_move_coef*PoseRFoot(0,0);
                    PoseRFoot(0,0)=0;
                    cout<<GlobalTime<<"\t"<<PoseRoot(1)<<"\t"<<PoseRFoot(1)<<"\t"<<PoseLFoot(1)<<endl;

                    SURENA_turning_side.doIK("LLeg_AnkleR_J6",PoseLFoot,R_F_L,"Body", PoseRoot,R_P);
                    SURENA_turning_side.doIK("RLeg_AnkleR_J6",PoseRFoot,R_F_R,"Body", PoseRoot,R_P);
                }
                if(sidewalk&&turning){ROS_INFO("unable to turn and walk to side!"); break;}

            }

        }



        if(GlobalTime>=DurationOfendPhase+DurationOfStartPhase+OnlineTaskSpace.MotionTime){break;}
        if(turning || sidewalk) {links = SURENA_turning_side.GetLinks();}
        else{links = SURENA.GetLinks();}
        EndPhase();
}

        double k_roll_r=1;
        double k_roll_l=1;
        if(!left_first){
            k_roll_r=OnlineTaskSpace.LeftHipRollModification/OnlineTaskSpace.RightHipRollModification;
            k_roll_l=OnlineTaskSpace.RightHipRollModification/OnlineTaskSpace.LeftHipRollModification;
        }

        cntrl[0]=0.0;
        cntrl[1]=links[1].JointAngle;
        cntrl[2]=links[2].JointAngle+k_roll_r*RollModified(0,0);
        cntrl[3]=links[3].JointAngle+k_pitch*PitchModified;
        cntrl[4]=links[4].JointAngle;
        cntrl[5]=saturate(links[5].JointAngle,-M_PI/5,M_PI/4)+ankle_adaptation_switch*teta_motor_R;
        cntrl[6]=links[6].JointAngle+ankle_adaptation_switch*(phi_motor_R);//roll
        cntrl[7]=links[7].JointAngle;
        cntrl[8]=links[8].JointAngle+k_roll_l*RollModified(1,0);
        cntrl[9]=links[9].JointAngle+k_pitch*PitchModified;
        cntrl[10]=links[10].JointAngle;
        cntrl[11]=saturate(links[11].JointAngle,-M_PI/5,M_PI/4)+ankle_adaptation_switch*teta_motor_L;
        cntrl[12]=links[12].JointAngle+ankle_adaptation_switch*(phi_motor_L);

       // qDebug()<<"Pitch: hip:"<<cntrl[3]<<"\tknee:"<<cntrl[4]<<"\tankle:"<<cntrl[5];


        vector<int> qref(12);
        qref=QC.ctrldata2qc(cntrl);
        int q_motor_r[8];int q_motor_l[8];

        //  q_motor_r[0]=-int(10*.8*saturate(cntrl[9]-shoulderPitchOffset,-10,M_PI/6)*(GlobalTime>=DurationOfStartPhase)*180/M_PI*120/60)+0*qra_offset[0];
        q_motor_r[0]=int(10*(asin((PoseLFoot(0,0)-PoseRoot(0,0))/.6))*180/M_PI*120/60)+0*qra_offset[0];
        q_motor_r[1]=int(10*(0)*180/M_PI*120/60)+0*qra_offset[1];
        q_motor_r[2]=-int(7*(0)*180/M_PI*100/60)+0*qra_offset[2];
        q_motor_r[3]=int(7*(0)*180/M_PI*100/60)+0*qra_offset[3];

        q_motor_r[4]=int((0)*(2048)/M_PI);
        q_motor_r[5]=int((0)*(4000-2050)/(23*M_PI/180));
        q_motor_r[6]=int((0)*(4000-2050)/(23*M_PI/180));
        q_motor_r[7]=14;

        // q_motor_l[0]=int(10*.8*saturate(cntrl[3]-shoulderPitchOffset,-10,M_PI/6)*(GlobalTime>=DurationOfStartPhase)*180/M_PI*120/60)+0*qla_offset[0];
        q_motor_l[0]=-int(10*(asin((PoseRFoot(0,0)-PoseRoot(0,0))/.6))*180/M_PI*120/60)+0*qla_offset[0];
        q_motor_l[1]=int(10*(0)*180/M_PI*120/60)+0*qla_offset[1];
        q_motor_l[2]=-int(7*(0)*180/M_PI*100/60)+0*qla_offset[2];
        q_motor_l[3]=-int(7*(0)*180/M_PI*100/60)+0*qla_offset[3];

        q_motor_l[4]=int((0)*(2048)/M_PI);
        q_motor_l[5]=-int((0)*(4000-2050)/(23*M_PI/180));
        q_motor_l[6]=int((0)*(4000-2050)/(23*M_PI/180));
        q_motor_l[7]=14;

        int q_waist_pitch=waist_pitch*(1/(2*M_PI))*(2304)*100;


        msg.data.clear();

        if(left_first){
            for(int  i = 0;i < 12;i++)
            {
                msg.data.push_back(qref[i]+qc_offset[i]);
            }}
        else{
            msg.data.push_back(-qref[5]+qc_offset[0]);
            msg.data.push_back(-qref[4]+qc_offset[1]);
            msg.data.push_back(-qref[6]+qc_offset[2]);
            msg.data.push_back(-qref[7]+qc_offset[3]);
            msg.data.push_back(-qref[1]+qc_offset[4]);
            msg.data.push_back(-qref[0]+qc_offset[5]);
            msg.data.push_back(-qref[2]+qc_offset[6]);
            msg.data.push_back(-qref[3]+qc_offset[7]);
            msg.data.push_back(-qref[11]+qc_offset[8]);
            msg.data.push_back(-qref[10]+qc_offset[9]);
            msg.data.push_back(-qref[9]+qc_offset[10]);
            msg.data.push_back(-qref[8]+qc_offset[11]);
        }


        //right hand epose
        msg.data.push_back(q_motor_r[0]);//12 -y  a,z
        msg.data.push_back(q_motor_r[1]);//13 +x
        msg.data.push_back(q_motor_r[2]);//14 -z
        msg.data.push_back(q_motor_r[3]);//15 +y
        //right hand dynamixel + fingers
        msg.data.push_back(q_motor_r[4]);//16
        msg.data.push_back(q_motor_r[5]);//17
        msg.data.push_back(q_motor_r[6]);//18
        msg.data.push_back(q_motor_r[7]);//19
        //left hand epose
        msg.data.push_back(q_motor_l[0]);//20 +y
        msg.data.push_back(q_motor_l[1]);//21 +x
        msg.data.push_back(q_motor_l[2]);//22 -z
        msg.data.push_back(q_motor_l[3]);//23 -y
        //left hand dynamixel + fingers
        msg.data.push_back(q_motor_l[4]);//24
        msg.data.push_back(q_motor_l[5]);//25
        msg.data.push_back(q_motor_l[6]);//26
        msg.data.push_back(q_motor_l[7]);//27
        msg.data.push_back(q_waist_pitch);
        //        for(int  i = 12;i < 28;i++)
        //        {
        //            msg.data.push_back(0);
        //        }
        if(!simulation){chatter_pub.publish(msg);}

        if (simulation){
            if(left_first){SendGazebo(links,0*RollModified,0*PitchModified,teta_motor_R,phi_motor_R,teta_motor_L,phi_motor_L);}
            else{SendGazebo_reverse(links,0*RollModified,0*PitchModified,teta_motor_R,phi_motor_R,teta_motor_L,phi_motor_L);}
        }
        links = SURENA.GetLinks();


        if(LogDataSend){


            if(OnlineTaskSpace.localTiming<.1){contact_flag_timing=-contact_flag_timing;}//flag to show expected contact according to timing,sign of flag changes
            //-------------for detecting the first contact of Left foot with ground-------------//
            if (( (a)<footSensorthreshold && (b)<footSensorthreshold && (c)<footSensorthreshold && (d)<footSensorthreshold)){
                fullcontactL=false;                firstcontactL=true;            }
            if (firstcontactL==true &&( (a)>=footSensorthreshold || (b)>=footSensorthreshold || (c)>=footSensorthreshold || (d)>=footSensorthreshold)){
                contact_flag_sensor2=-contact_flag_sensor2;                firstcontactL=false;            }
            //-------------for detecting the full contact of Left foot with ground-------------//
            if ((!fullcontactL) && (a)>=footSensorSaturation && (b)>=footSensorSaturation && (c)>=footSensorSaturation && (d)>=footSensorSaturation){
                contact_flag_sensor=-contact_flag_sensor;                fullcontactL=true;            }
            //-------------for detecting the first contact of Right foot with ground-------------//
            if (( (e)<footSensorthreshold && (f)<footSensorthreshold && (g)<footSensorthreshold && (h)<footSensorthreshold)){
                fullcontactR=false;                firstcontactR=true;            }
            if (firstcontactR==true &&( (e)>=footSensorthreshold || (f)>=footSensorthreshold || (g)>=footSensorthreshold || (h)>=footSensorthreshold)){
                contact_flag_sensor2=-contact_flag_sensor2;                firstcontactR=false;            }
            //-------------for detecting the full contact of Right foot with ground-------------//
            if ((!fullcontactR) && (e)>=footSensorSaturation && (f)>=footSensorSaturation && (g)>=footSensorSaturation && (h)>=footSensorSaturation){
                contact_flag_sensor=-contact_flag_sensor;                fullcontactR=true;            }

            msg_contact_flag.data.clear();
            msg_contact_flag.data.push_back(contact_flag_timing);
            msg_contact_flag.data.push_back(contact_flag_sensor);
            msg_contact_flag.data.push_back(contact_flag_sensor2);

            contact_flag.publish(msg_contact_flag);

            trajectory_data.data.clear();
            trajectory_data.data.push_back(OnlineTaskSpace.globalTime);
            trajectory_data.data.push_back(PoseRoot(0));
            trajectory_data.data.push_back(PoseRoot(1));
            trajectory_data.data.push_back(PoseRoot(2));
            trajectory_data.data.push_back(PoseRFoot(0));
            trajectory_data.data.push_back(PoseRFoot(1));
            trajectory_data.data.push_back(PoseRFoot(2));
            trajectory_data.data.push_back(PoseRFoot(4));
            trajectory_data.data.push_back(PoseLFoot(0));
            trajectory_data.data.push_back(PoseLFoot(1));
            trajectory_data.data.push_back(PoseLFoot(2));
            trajectory_data.data.push_back(PoseLFoot(4));
            trajectory_data.data.push_back(double(a)/500);
            trajectory_data.data.push_back(double(b)/500);
            trajectory_data.data.push_back(double(c)/500);
            trajectory_data.data.push_back(double(d)/500);
            trajectory_data.data.push_back(double(e)/500);
            trajectory_data.data.push_back(double(f)/500);
            trajectory_data.data.push_back(double(g)/500);
            trajectory_data.data.push_back(double(h)/500);
            trajectory_data.data.push_back(cntrl[1]);
            trajectory_data.data.push_back(cntrl[2]);
            trajectory_data.data.push_back(cntrl[3]);
            trajectory_data.data.push_back(cntrl[4]);
            trajectory_data.data.push_back(cntrl[5]);
            trajectory_data.data.push_back(cntrl[6]);
            trajectory_data.data.push_back(cntrl[7]);
            trajectory_data.data.push_back(cntrl[8]);
            trajectory_data.data.push_back(cntrl[9]);
            trajectory_data.data.push_back(cntrl[10]);
            trajectory_data.data.push_back(cntrl[11]);
            trajectory_data.data.push_back(cntrl[12]);
            trajectory_data.data.push_back(cntrl[12]);
            trajectory_data.data.push_back(pelvis_orientation_roll);
            trajectory_data.data.push_back(pelvis_orientation_pitch);
            trajectory_data.data.push_back(pelvis_orientation_yaw);
            trajectory_data.data.push_back(qc_offset[0]);
            trajectory_data.data.push_back(qc_offset[1]);
            trajectory_data.data.push_back(qc_offset[2]);
            trajectory_data.data.push_back(qc_offset[3]);
            trajectory_data.data.push_back(qc_offset[4]);
            trajectory_data.data.push_back(qc_offset[5]);
            trajectory_data.data.push_back(qc_offset[6]);
            trajectory_data.data.push_back(qc_offset[7]);
            trajectory_data.data.push_back(qc_offset[8]);
            trajectory_data.data.push_back(qc_offset[9]);
            trajectory_data.data.push_back(qc_offset[10]);
            trajectory_data.data.push_back(qc_offset[11]);

            trajectory_data_pub.publish(trajectory_data);


            data_pose.append(QString::number(GlobalTime)+","+QString::number(PoseRoot(0))+","+QString::number(PoseRoot(1))+","+QString::number(PoseRoot(2))+","+
                             QString::number(PoseLFoot(0))+","+QString::number(PoseLFoot(1))+","+QString::number(PoseLFoot(2))+","+
                             QString::number(PoseRFoot(0))+","+QString::number(PoseRFoot(1))+","+QString::number(PoseRFoot(2))+","+
                             QString::number(PoseRFoot(4))+","+QString::number(PoseLFoot(4))+"\n");


        }

//if(GlobalTime>=DurationOfStartPhase){break;}
        if(count%20==0){ //use to print once in n steps
            // ROS_INFO("");
            //            ROS_INFO("I heard data of sensors :t=%f [%d %d %d %d] & [%d %d %d %d]",OnlineTaskSpace.globalTime,a,b,c,d,e,f,g,h);
            //     ROS_INFO("I heard a b c d: [%d  %d %d %d],e f g h: [%d  %d %d %d]", a,b,c,d,e,f,g,h);
            //    ROS_INFO("teta_motor_L=%f,teta_motor_R=%f,phi_motor_L=%f,phi_motor_R=%f",teta_motor_L,teta_motor_R,phi_motor_L,phi_motor_R);
            //   ROS_INFO("ankl pith min=%f,max=%f",min_test*180/M_PI,max_test*180/M_PI);
            //qDebug()<<"T="<<GlobalTime<<"/tTc="<<OnlineTaskSpace.Tc;
            //qDebug()<<"pelvis roll:"<<pelvis_orientation_roll<<"\tpitch:"<<pelvis_orientation_pitch<<"\tyaw:"<<pelvis_orientation_yaw;
        }

        /*
*/

        ros::spinOnce();
        loop_rate.sleep();
        ++count;


        data_time.append(QString::number(timer.elapsed())+"\n");

    }

    myfile_pose.remove();
    myfile_pose.open(QFile::ReadWrite);
    myfile_pose.write(data_pose);
    myfile_pose.close();

    myfile_time.remove();
    myfile_time.open(QFile::ReadWrite);
    myfile_time.write(data_time);
    myfile_time.close();


    //    myfile_inc.open(QFile::ReadWrite);
    //    myfile_inc.write(data_inc);
    //    myfile_inc.close();
    //    myfile_abs.open(QFile::ReadWrite);
    //    myfile_abs.write(data_abs);
    //    myfile_abs.close();
    //    myfile_abs.open(QFile::ReadWrite);
    //    myfile_abs.write(data_abs);
    //    myfile_abs.close();

    return 0;
}

