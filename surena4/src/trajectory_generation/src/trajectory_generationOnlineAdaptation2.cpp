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
#include"trajectory_generation/walk.h"
#include "trajectory_generation/walkRequest.h"
#include "trajectory_generation/walkResponse.h"


using namespace  std;
using namespace  Eigen;

double stepLength = .25;
int NStride=4;
bool left_first=!true;//right support in first step
bool turning=!false;
double TurningRadius=.01;//1.5/M_PI;//
bool move_active=false;

bool backward=false;
bool sidewalk=false;
int bump_threshold=75;//75 85;
bool simulation=false;
bool AnkleZAdaptation=!false;
bool LogDataSend=false;
double ankle_adaptation_switch=1;// 1 for activating adaptation 0 for siktiring adaptation
double k_pitch=0*.8;//1;0.8;
double pelvis_roll_range=1; //1

double rankle_inc,lankle_inc;
double lknee_inc,rknee_inc;
double lankle_absolute,rankle_absolute;
double lknee_absolute,rknee_absolute;


Robot SURENA;//model of robot & kinematics funcs(IK & FK)
Robot SURENA_turning_side;
TaskSpaceOnline3 OnlineTaskSpace(NStride,stepLength);
QList<LinkM> links;
MatrixXd PoseRoot;//position of pelvis respected to global coordinate
MatrixXd PoseRFoot;//position of right ankle joint respected to global coordinate
MatrixXd PoseLFoot;//position of left ankle joint respected to global coordinate
double GlobalTime;
double footSensorSaturation=75;//if all sensors data are bigger than this amount, this means the foot is landed on the ground
double footSensorthreshold=4;// will start orientaition correction

double DurationOfStartPhase;
double DurationOfendPhase;
double shoulderPitchOffset;



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
int bump_pushed2[8];
int bump_notpushed2[8];

bool bump_initialize;
vector<double> cntrl(13);
QCgenerator QC;

bool leftzstop;
bool rightzstop;

int qra_offset[4];
int qla_offset[4];

int inc_feedback[12];
int abs_feedback[12];
double Fzl,Fzr,Mxl,Mxr;

double pelvis_orientation_pitch,pelvis_orientation_roll,pelvis_orientation_yaw;
double pelvis_acceleration[3];
double pelvis_angularVelocity[3];

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
    if(a<min){//ROS_INFO("subceeding!");
        return min;}
    else if(a>max){//ROS_INFO("exceeding!");
        return max;}
    else{return a;}
}

bool StartWalk(trajectory_generation::walkRequest &req,trajectory_generation::walkResponse &res)
{


    if(!move_active){
    qc_initial_bool=true;
    for (int i = 0; i < 12; ++i) {
        qc_offset[i]=0;
    }

    leftzstop=false;
    rightzstop=false;
    GlobalTime=0;
    stepLength =req.stepLengh;
    NStride=req.stepCount;

    left_first=req.leftFirst;//right support in first step
QString note=" ";
if(left_first){note+="left ";}
else{note+="right ";}
    turning=false;
    backward=false;
    sidewalk=false;

    switch (req.motionID) {

    case 1:
        turning=true;
        TurningRadius=1.5/M_PI;//
        note+="turn(radius=1.5/PI)   number of strides= ";note+=QString::number(NStride);

        break;
    case 2:
        turning=true;
        TurningRadius=.01;//
        note+="on spot turn   number of strides= ";note+=QString::number(NStride);
        break;
    case 3:
        backward=true;
        note+="start backward walk   number of strides= ";note+=QString::number(NStride);
        break;
    case 4:
        sidewalk=true;
        note+="side walk   number of strides= ";note+=QString::number(NStride);
        break;
    default:
        turning=false;
        backward=false;
        sidewalk=false;
        note+="start straight walk   step-length= ";note+=QString::number(stepLength);
        note+="   number of strides= ";note+=QString::number(NStride);
        break;
    }
    TaskSpaceOnline3 TaskSpace_temp(NStride,stepLength);
    if(turning || sidewalk) {links = SURENA_turning_side.GetLinks();}
    else{links = SURENA.GetLinks();}

    OnlineTaskSpace.StepNumber=1;
    OnlineTaskSpace=TaskSpace_temp;
    if(turning&&TurningRadius<.2){OnlineTaskSpace.StepLength=TurningRadius/18*M_PI;
        OnlineTaskSpace.XofAnkleMaximumHeight=OnlineTaskSpace.StepLength*1.8;
                OnlineTaskSpace.Xe=0;
                OnlineTaskSpace.Xs=0;
                OnlineTaskSpace.zmp_max=0;
                OnlineTaskSpace.zmp_min=0;
    }

    if(sidewalk||(turning&&TurningRadius<.2)){
//        OnlineTaskSpace.Xe=0;
//        OnlineTaskSpace.Xs=0;
        OnlineTaskSpace.side_extra_step_length=true;
        OnlineTaskSpace.CoeffArrayPelvis();
        OnlineTaskSpace.CoeffArrayAnkle();
        OnlineTaskSpace.CoeffSideStartEnd();

    }

qDebug()<<note;
move_active=true;
res.result=10;

return true;
}
    else{
    return false;
    }
}
bool StopWalk(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    qDebug()<<"stop walk";
return true;
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



//    bump_pushed[0]=1100;bump_pushed[1]= 835;bump_pushed[2]=3140;bump_pushed[3]=2994;
//    bump_pushed[4]=3132;bump_pushed[5]=2916;bump_pushed[6]=1203;bump_pushed[7]=916;
//    bump_notpushed[0]=1012;bump_notpushed[1]= 928;bump_notpushed[2]=3033;bump_notpushed[3]=3097;
//    bump_notpushed[4]=3036;bump_notpushed[5]=3006;bump_notpushed[6]=1107;bump_notpushed[7]=1015;



    if (GlobalTime<1){
        for (int i = 0; i < 8; ++i) {
            bump_pushed[i]=msg.data[i];
        }
    }
    if(left_first){
    if(fabs(GlobalTime-(DurationOfStartPhase+OnlineTaskSpace.TStart-OnlineTaskSpace.TSS/2))<.2){
        for (int i = 0; i < 4; ++i) {
            bump_notpushed[i]=msg.data[i];
        }
    }

    if(fabs(GlobalTime-(DurationOfStartPhase+OnlineTaskSpace.TStart+OnlineTaskSpace.TDs+OnlineTaskSpace.TSS/2))<.2){
        for (int i = 4; i < 8; ++i) {
            bump_notpushed[i]=msg.data[i];
        }
    }
     }
    else{
        if(fabs(GlobalTime-(DurationOfStartPhase+OnlineTaskSpace.TStart-OnlineTaskSpace.TSS/2))<.2){
            for (int i = 4; i < 8; ++i) {
                bump_notpushed[i]=msg.data[i];
            }
        }

        if(fabs(GlobalTime-(DurationOfStartPhase+OnlineTaskSpace.TStart+OnlineTaskSpace.TDs+OnlineTaskSpace.TSS/2))<.2){
            for (int i = 0; i < 4; ++i) {
                bump_notpushed[i]=msg.data[i];
            }
        }
    }


    //    bump_pushed2[0]=1100;bump_pushed2[1]= 835;bump_pushed2[2]=3140;bump_pushed2[3]=2994;
    //    bump_pushed2[4]=3132;bump_pushed2[5]=2916;bump_pushed2[6]=1203;bump_pushed2[7]=916;
    //    bump_notpushed2[0]=1012;bump_notpushed2[1]= 928;bump_notpushed2[2]=3033;bump_notpushed2[3]=3097;
    //    bump_notpushed2[4]=3036;bump_notpushed2[5]=3006;bump_notpushed2[6]=1107;bump_notpushed2[7]=1015;

    //    for (int i = 0; i < 8; ++i) {
    //        qDebug()<<i<<"\t"<<bump_pushed[i]<<"("<<bump_pushed2[i]<<")\t"<<bump_notpushed[i]<<"("<<bump_notpushed2[i]<<")";
    //    }


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

    lankle_inc=msg.position[6];
    rankle_inc=msg.position[1];
    lknee_inc=msg.position[7];
    rknee_inc=msg.position[3];

}


void abs_feedback_func(const sensor_msgs::JointState & msg){
//    for (int i = 0; i < 12; ++i) {
//        abs_feedback[i]=int(msg.position[i+1]);
//    }

    lankle_absolute=msg.position[6];
    rankle_absolute=msg.position[1];
    lknee_absolute=msg.position[7];
    rknee_absolute=msg.position[3];


}


void FT_left_feedback(const geometry_msgs::Wrench &msg){
    Fzl=msg.force.z;
    Mxl=msg.torque.y;
}

void FT_right_feedback(const geometry_msgs::Wrench &msg){
    Fzr=msg.force.z;
    Mxr=msg.torque.x;
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
 // qDebug()<<pelvis_orientation_roll<<"\t"<<pelvis_orientation_pitch<<"\t"<<pelvis_orientation_yaw;

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
    phi_motor_L=saturate(phi_motor_L,-M_PI/90,M_PI/90);
    teta_motor_L=saturate(teta_motor_L,-M_PI/90,M_PI/90);
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


    phi_motor_R=saturate(phi_motor_R,-M_PI/90,M_PI/90);
    teta_motor_R=saturate(teta_motor_R,-M_PI/90,M_PI/90);
    //------------------------saturation of ankle motors----------------------------//


    //        qDebug()<<"a:"<<a<<"\tb:"<<b<<"\tc:"<<c<<"\td:"<<d<<"\ne:"<<e<<"\tf:"<<f<<"\tg:"<<g<<"\th:"<<h;

    //        qDebug()<<"teta_motor_L:"<<teta_motor_L<<"phi_motor_L:"<<phi_motor_L<<
    //"\nteta_motor_R:"<<teta_motor_R<<"phi_motor_R:"<<phi_motor_R;
}

int main(int argc, char **argv)
{
    if(turning&&TurningRadius<.2){OnlineTaskSpace.StepLength=TurningRadius/18*M_PI;
        OnlineTaskSpace.XofAnkleMaximumHeight=OnlineTaskSpace.StepLength*1.8;
                OnlineTaskSpace.Xe=0;
                OnlineTaskSpace.Xs=0;
                OnlineTaskSpace.zmp_max=0;
                OnlineTaskSpace.zmp_min=0;
    }

    if(sidewalk||(turning&&TurningRadius<.2)){
//        OnlineTaskSpace.Xe=0;
//        OnlineTaskSpace.Xs=0;
        OnlineTaskSpace.side_extra_step_length=true;
        OnlineTaskSpace.CoeffArrayPelvis();
        OnlineTaskSpace.CoeffArrayAnkle();
        OnlineTaskSpace.CoeffSideStartEnd();

    }




    qc_initial_bool=true;
    bump_initialize=false;



    leftzstop=false;
    rightzstop=false;


    for (int i = 0; i < 12; ++i) {
        qc_offset[i]=0;
    }

    double kp=.02;
    teta_PID_L.Init(.005,.9,-.9,kp,0,0);
    teta_PID_R.Init(.005,.9,-.9,kp,0,0);
    phi_PID_L.Init(.005,.9,-.9,kp,0,0);
    phi_PID_R.Init(.005,.9,-.9,kp,0,0);




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
    ros::ServiceServer WalkService = nh.advertiseService("Walk", StartWalk);
    ros::ServiceServer StopService = nh.advertiseService("Stop", StopWalk);
    ros::Subscriber sub = nh.subscribe("/surena/bump_sensor_state", 1000, receiveFootSensor);
    //   ros::Subscriber ft_left = nh.subscribe("/surena/ft_l_state",1000,FT_left_feedback);
    // ros::Subscriber ft_right = nh.subscribe("/surena/ft_r_state",1000,FT_right_feedback);
    ros::Subscriber qcinit = nh.subscribe("/surena/inc_joint_state", 1000, qc_initial);
    ros::Subscriber abs_sub = nh.subscribe("/surena/abs_joint_state", 1000, abs_feedback_func);
    ros::Subscriber imusub = nh.subscribe("/surena/imu_state", 1000, imu_data_process);

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

//    ROS_INFO("press any key to start!");
//    getch();
//    ROS_INFO("started!");


    bool firstcontactL=true;
    bool firstcontactR=true;
    bool fullcontactL=false;
    bool fullcontactR=false;

    //*****loging data in a file

    QByteArray data_pose;
    QByteArray data_ankle;



    while (ros::ok())
    {
if(move_active){
        //MotionTime=TStart+NStride*2*Tc+TDs+TEnd;

        if (qc_initial_bool) {
            ROS_INFO_ONCE("qc is initializing!");
            ros::spinOnce();
            continue;
        }

//ankle orientation adaptation
        if (GlobalTime>DurationOfStartPhase+OnlineTaskSpace.TStart&&GlobalTime<DurationOfStartPhase+OnlineTaskSpace.MotionTime-OnlineTaskSpace.TEnd-OnlineTaskSpace.TDs) {
            double adapt_time=fmod(GlobalTime-DurationOfStartPhase-OnlineTaskSpace.TStart,2*OnlineTaskSpace.Tc);



            if (adapt_time>OnlineTaskSpace.TDs&&adapt_time<OnlineTaskSpace.Tc+OnlineTaskSpace.TDs/2) {
                if(left_first){
                  ankleOrientationAdaptationRight();
                }
                else{
                  ankleOrientationAdaptationLeft();
                }

//                if (fabs(adapt_time-(OnlineTaskSpace.Tc+OnlineTaskSpace.TDs/2))<.005) {
//                    ROS_INFO("%f I heard a b c d: [%d  %d %d %d],e f g h: [%d  %d %d %d]",GlobalTime, a,b,c,d,e,f,g,h);
//        }
            }
            else{
                if(left_first){
                teta_motor_R+=phi_PID_R.Calculate(0,teta_motor_R);
                phi_motor_R+=phi_PID_R.Calculate(0,phi_motor_R);
                }
                else{
                    teta_motor_L+=phi_PID_L.Calculate(0,teta_motor_L);
                    phi_motor_L+=phi_PID_L.Calculate(0,phi_motor_L);
                }



            }
            if (adapt_time<OnlineTaskSpace.TDs/2||adapt_time>OnlineTaskSpace.Tc+OnlineTaskSpace.TDs) {

                if(left_first){
                  ankleOrientationAdaptationLeft();
                }
                else{
                  ankleOrientationAdaptationRight();
                }

                //            if (fabs(adapt_time-(OnlineTaskSpace.Tc+OnlineTaskSpace.TDs))<.005){
                //                ROS_INFO("%f I heard a b c d: [%d  %d %d %d],e f g h: [%d  %d %d %d]",GlobalTime, a,b,c,d,e,f,g,h);
                //            }

            }
            else{
                if(left_first){
                    teta_motor_L+=phi_PID_L.Calculate(0,teta_motor_L);
                    phi_motor_L+=phi_PID_L.Calculate(0,phi_motor_L);
                }
                else{
                          teta_motor_R+=phi_PID_R.Calculate(0,teta_motor_R);
                    phi_motor_R+=phi_PID_R.Calculate(0,phi_motor_R);
                }

            }
        }
        else{
            teta_motor_R+=phi_PID_R.Calculate(0,teta_motor_R);
            phi_motor_R+=phi_PID_R.Calculate(0,phi_motor_R);
            teta_motor_L+=phi_PID_L.Calculate(0,teta_motor_L);
            phi_motor_L+=phi_PID_L.Calculate(0,phi_motor_L);
        }




        StartPhase();



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
                    //first step just stopping left
                    if(GlobalTime<DurationOfStartPhase+OnlineTaskSpace.TStart){
                        AnkleZR=m7;
                        if(GlobalTime-DurationOfStartPhase<OnlineTaskSpace.Tx+OnlineTaskSpace.Tc){
                            AnkleZL=m3;
                        }
                        else{

                            if(!leftzstop){

                                if((left_first&&(a>bump_threshold||b>bump_threshold||c>bump_threshold||d>bump_threshold))||((!left_first)&&(e>bump_threshold||f>bump_threshold||g>bump_threshold||h>bump_threshold))){
                                    leftzstop=true;
                                    AnkleZL=m3;
                                    AnkleZL+=AnkleZ_offsetL;
                                    AnkleZ_offsetL=AnkleZL-OnlineTaskSpace._lenghtOfAnkle;
                                    qDebug()<<"leftzstop=true AnkleZL="<<AnkleZL<<"offset="<<AnkleZ_offsetL;
                                    ROS_INFO("I heard a b c d: [%d  %d %d %d],e f g h: [%d  %d %d %d]", a,b,c,d,e,f,g,h);
                                }
                                else{AnkleZL=m3;
                                    AnkleZL+=AnkleZ_offsetL;
                                }
                            }

                        }
                    }
                   //cyle z
                    else if(GlobalTime<DurationOfStartPhase+OnlineTaskSpace.TGait+OnlineTaskSpace.TDs){
                        local_time_cycle=fmod(GlobalTime-DurationOfStartPhase-OnlineTaskSpace.TStart,2*OnlineTaskSpace.Tc);

                        //right foot up
                        if(local_time_cycle<=OnlineTaskSpace.TDs+OnlineTaskSpace.TStartofAnkleAdaptation){
                            rightzstop=false;
                            AnkleZR=m7;
                            AnkleZ_offsetR=0;
                        }
                        //right foot stop
                        else if(local_time_cycle<=OnlineTaskSpace.TDs+OnlineTaskSpace.TSS){
                            if(!rightzstop){
                                if(((!left_first)&&(a>bump_threshold||b>bump_threshold||c>bump_threshold||d>bump_threshold))||((left_first)&&(e>bump_threshold||f>bump_threshold||g>bump_threshold||h>bump_threshold))){
                                    rightzstop=true;
                                    AnkleZR=m7;
                                    AnkleZR+=AnkleZ_offsetR;
                                    AnkleZ_offsetR=AnkleZR-OnlineTaskSpace._lenghtOfAnkle;
                                    qDebug()<<"rightzstop=true AnkleZR="<<AnkleZR<<"offset="<<AnkleZ_offsetR;
                                    ROS_INFO("I heard a b c d: [%d  %d %d %d],e f g h: [%d  %d %d %d]", a,b,c,d,e,f,g,h);

                                }
                                else{AnkleZR=m7;
                                    AnkleZR+=AnkleZ_offsetR;

                                }
                            }
                        }
                        // takhlie pye rast
                        else if(local_time_cycle<=2*OnlineTaskSpace.Tc-OnlineTaskSpace.TSS+OnlineTaskSpace.TStartofAnkleAdaptation){
                            AnkleZR=OnlineTaskSpace._lenghtOfAnkle+AnkleZ_offsetR-move2pose(AnkleZ_offsetR,local_time_cycle,OnlineTaskSpace.Tc,OnlineTaskSpace.Tc+OnlineTaskSpace.TDs);

                        }

                        // takhlie pye chap
                        if(local_time_cycle<=OnlineTaskSpace.Tc-OnlineTaskSpace.TSS+OnlineTaskSpace.TStartofAnkleAdaptation){
                            AnkleZL=OnlineTaskSpace._lenghtOfAnkle+AnkleZ_offsetL-move2pose(AnkleZ_offsetL,local_time_cycle,0,OnlineTaskSpace.TDs);

                        }
                        //left foot swing
                        else if(local_time_cycle<=OnlineTaskSpace.Tc+OnlineTaskSpace.TDs+OnlineTaskSpace.TStartofAnkleAdaptation){
                            leftzstop=false;
                            AnkleZL=m3;
                            AnkleZ_offsetL=0;
                        }
                        else if(local_time_cycle<=OnlineTaskSpace.Tc+OnlineTaskSpace.TDs+OnlineTaskSpace.TSS){
                            if(!leftzstop){
//left foot stop
                                if(((left_first)&&(a>bump_threshold||b>bump_threshold||c>bump_threshold||d>bump_threshold))||((!left_first)&&(e>bump_threshold||f>bump_threshold||g>bump_threshold||h>bump_threshold))){

                                    leftzstop=true;

                                    AnkleZL=m3;
                                    AnkleZL+=AnkleZ_offsetL;
                                    AnkleZ_offsetL=AnkleZL-OnlineTaskSpace._lenghtOfAnkle;
                                    qDebug()<<"leftzstop=true AnkleZL="<<AnkleZL<<"offset="<<AnkleZ_offsetR;
                                    ROS_INFO("I heard a b c d: [%d  %d %d %d],e f g h: [%d  %d %d %d]", a,b,c,d,e,f,g,h);

                                }

                                else{AnkleZL=m3;
                                    AnkleZL+=AnkleZ_offsetL;

                                }
                            }
                        }

                    }
                                        else if(GlobalTime<DurationOfStartPhase+OnlineTaskSpace.MotionTime){

                                            //right foot up
                                            if(GlobalTime<=DurationOfStartPhase+OnlineTaskSpace.TGait+OnlineTaskSpace.TDs+0.5*OnlineTaskSpace.TLastSS){
                                                rightzstop=false;
                                                AnkleZR=m7;
                                                AnkleZ_offsetR=0;
                                            }

                                            //right foot stop
                                           else if(GlobalTime<=DurationOfStartPhase+OnlineTaskSpace.MotionTime-OnlineTaskSpace.TE){
                                                if(!rightzstop){
                                                    if(((!left_first)&&(a>bump_threshold||b>bump_threshold||c>bump_threshold||d>bump_threshold))||((left_first)&&(e>bump_threshold||f>bump_threshold||g>bump_threshold||h>bump_threshold))){
                                                        rightzstop=true;
                                                        AnkleZR=m7;
                                                        AnkleZR+=AnkleZ_offsetR;
                                                        AnkleZ_offsetR=AnkleZR-OnlineTaskSpace._lenghtOfAnkle;
                                                        qDebug()<<"rightzstop=true AnkleZR="<<AnkleZR<<"offset="<<AnkleZ_offsetR;
                                                        ROS_INFO("I heard a b c d: [%d  %d %d %d],e f g h: [%d  %d %d %d]", a,b,c,d,e,f,g,h);

                                                    }
                                                    else{AnkleZR=m7;
                                                        AnkleZR+=AnkleZ_offsetR;


                                                    }
                                                }
                                            }
                                            // takhlie pye rast
                                            else if(GlobalTime<=DurationOfStartPhase+OnlineTaskSpace.MotionTime){
                                                AnkleZR=OnlineTaskSpace._lenghtOfAnkle+AnkleZ_offsetR-move2pose(AnkleZ_offsetR,GlobalTime-DurationOfStartPhase,OnlineTaskSpace.MotionTime-OnlineTaskSpace.TE,OnlineTaskSpace.MotionTime-0.66*OnlineTaskSpace.TE);

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

         double alpha=.9;
         if(sidewalk||(turning&&TurningRadius<.2)){alpha=1;}
         double time_margin=.01;
         double coef_y_la;
         double coef_y_ra;
         double coef_y_p;

         coef_y_la=1-move2pose(1-alpha,GlobalTime-DurationOfStartPhase,OnlineTaskSpace.Tx+OnlineTaskSpace.TDs+OnlineTaskSpace.TSS/2+time_margin,OnlineTaskSpace.TStart-time_margin)
                 +move2pose(1-alpha,GlobalTime-DurationOfStartPhase,OnlineTaskSpace.TGait-OnlineTaskSpace.TSS+time_margin,OnlineTaskSpace.TGait-time_margin);

         coef_y_ra=1-move2pose(1-alpha,GlobalTime-DurationOfStartPhase,OnlineTaskSpace.TStart+OnlineTaskSpace.TDs+time_margin,OnlineTaskSpace.TStart+OnlineTaskSpace.Tc-time_margin)
                 +move2pose(1-alpha,GlobalTime-DurationOfStartPhase,OnlineTaskSpace.TGait+OnlineTaskSpace.TDs+time_margin,OnlineTaskSpace.TGait+OnlineTaskSpace.Tc-time_margin);

         coef_y_p=1-move2pose(1-alpha,GlobalTime-DurationOfStartPhase,OnlineTaskSpace.TStart+OnlineTaskSpace.TDs/2+time_margin,OnlineTaskSpace.TStart+OnlineTaskSpace.TDs-time_margin)
                 +move2pose(1-alpha,GlobalTime-DurationOfStartPhase,OnlineTaskSpace.TGait+OnlineTaskSpace.TDs/2+time_margin,OnlineTaskSpace.TGait+OnlineTaskSpace.TDs-time_margin);





         PoseLFoot(1)=PoseLFoot(1)*coef_y_la;
PoseRFoot(1)=PoseRFoot(1)*coef_y_ra;
PoseRoot(1)=PoseRoot(1)*coef_y_p;




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
                    double side_move_coef=0.5;//.08/2/OnlineTaskSpace.StepLength;


                    //*****************************
                    PoseRoot(1,0)=PoseRoot(1,0)+side_move_coef*PoseRoot(0,0);
                    PoseRoot(0,0)=0;

                    PoseLFoot(1,0)=PoseLFoot(1,0)+side_move_coef*PoseLFoot(0,0);
                    PoseLFoot(0,0)=0;

                    PoseRFoot(1,0)=PoseRFoot(1,0)+side_move_coef*PoseRFoot(0,0);
                    PoseRFoot(0,0)=0;
                  //  cout<<GlobalTime<<"\t"<<PoseRoot(1)<<"\t"<<PoseRFoot(1)<<"\t"<<PoseLFoot(1)<<endl;

                    SURENA_turning_side.doIK("LLeg_AnkleR_J6",PoseLFoot,R_F_L,"Body", PoseRoot,R_P);
                    SURENA_turning_side.doIK("RLeg_AnkleR_J6",PoseRFoot,R_F_R,"Body", PoseRoot,R_P);
                }
                if(sidewalk&&turning){ROS_INFO("unable to turn and walk to side!"); break;}

            }

        }




        if(GlobalTime>=DurationOfendPhase+DurationOfStartPhase+OnlineTaskSpace.MotionTime){
            move_active=false;
qDebug()<<"done!";
SURENA_turning_side=SURENA;
            QFile myfile_pose("/media/cast/UBUNTU1604/robot_data/pose.txt");
            myfile_pose.remove();
            myfile_pose.open(QFile::ReadWrite);
            myfile_pose.write(data_pose);
            myfile_pose.close();
            data_pose.clear();
        }



        if(turning || sidewalk) {links = SURENA_turning_side.GetLinks();}
        else{links = SURENA.GetLinks();}
        EndPhase();


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
        cntrl[5]=saturate(links[5].JointAngle,-M_PI/5,M_PI/4)+ankle_adaptation_switch*(left_first*teta_motor_R+(!left_first)*teta_motor_L);
        cntrl[6]=links[6].JointAngle+ankle_adaptation_switch*(left_first*phi_motor_R+(!left_first)*phi_motor_L);//roll
        cntrl[7]=links[7].JointAngle;
        cntrl[8]=links[8].JointAngle+k_roll_l*RollModified(1,0);
        cntrl[9]=links[9].JointAngle+k_pitch*PitchModified;
        cntrl[10]=links[10].JointAngle;
        cntrl[11]=saturate(links[11].JointAngle,-M_PI/5,M_PI/4)+ankle_adaptation_switch*(left_first*teta_motor_L+(!left_first)*teta_motor_R);
        cntrl[12]=links[12].JointAngle+ankle_adaptation_switch*(left_first*phi_motor_L+(!left_first)*phi_motor_R);

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
        msg.data.push_back(0);
        //        for(int  i = 12;i < 28;i++)
        //        {
        //            msg.data.push_back(0);
        //        }
        chatter_pub.publish(msg);


        links = SURENA.GetLinks();

        data_pose.append(QString::number(GlobalTime)+","+QString::number(PoseRoot(0))+","+QString::number(PoseRoot(1))+","+QString::number(PoseRoot(2))+","+
                         QString::number(PoseLFoot(0))+","+QString::number(PoseLFoot(1))+","+QString::number(PoseLFoot(2))+","+
                         QString::number(PoseRFoot(0))+","+QString::number(PoseRFoot(1))+","+QString::number(PoseRFoot(2))+","+
                         QString::number(PoseRFoot(4))+","+QString::number(PoseLFoot(4))+"\n");
}
        ros::spinOnce();

        loop_rate.sleep();
        ++count;


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

            data_ankle.append(QString::number(GlobalTime)+","+QString::number(rankle_inc)+","+QString::number(cntrl[6])+","+QString::number(rankle_absolute)+","+
           QString::number(lankle_inc)+","+QString::number(cntrl[12])+","+QString::number(lankle_absolute)+","+
            QString::number(rknee_inc)+","+QString::number(cntrl[4])+","+QString::number(rknee_absolute)+","+
             QString::number(lknee_inc)+","+QString::number(cntrl[10])+","+QString::number(lknee_absolute)+"\n");
        }




        //data_time.append(QString::number(timer.elapsed())+"\n");

    }
if(LogDataSend){
    QFile myfile_pose("/media/cast/UBUNTU1604/robot_data/pose.txt");
    myfile_pose.remove();
    myfile_pose.open(QFile::ReadWrite);
    myfile_pose.write(data_pose);
    myfile_pose.close();
    QFile myfile_ankle("/media/cast/UBUNTU1604/robot_data/anklecurrent.txt");
    myfile_ankle.remove();
    myfile_ankle.open(QFile::ReadWrite);
    myfile_ankle.write(data_ankle);
    myfile_ankle.close();


 }



    return 0;
}


