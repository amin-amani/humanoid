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

ros::Publisher pub1; ros::Publisher pub2; ros::Publisher pub3; ros::Publisher pub4;
ros::Publisher pub5; ros::Publisher pub6; ros::Publisher pub7; ros::Publisher pub8;
ros::Publisher pub9; ros::Publisher pub10; ros::Publisher pub11; ros::Publisher pub12;
ros::Publisher pub13; ros::Publisher pub14; ros::Publisher pub15; ros::Publisher pub16;
ros::Publisher pub17; ros::Publisher pub18; ros::Publisher pub19; ros::Publisher pub20;
ros::Publisher pub21; ros::Publisher pub22; ros::Publisher pub23; ros::Publisher pub24;
ros::Publisher pub25; ros::Publisher pub26; ros::Publisher pub27; ros::Publisher pub28;
ros::Publisher pub29; ros::Publisher pub30; ros::Publisher pub31;

void  SendGazebo(QList<LinkM> links,MatrixXd RollModifieds, double PitchModifieds, double theta_r, double phi_r, double theta_l, double phi_l){
    if(links.count()<28){qDebug()<<"index err";return;}
    std_msgs::Float64 data;

    data.data=links[1].JointAngle;    pub1.publish(data);
    data.data=links[2].JointAngle+RollModifieds(0,0);    pub2.publish(data);
    data.data=links[3].JointAngle+PitchModifieds;    pub3.publish(data);
    data.data=links[4].JointAngle;    pub4.publish(data);
    data.data=links[5].JointAngle+theta_r;    pub5.publish(data);
    data.data=links[6].JointAngle+phi_r;    pub6.publish(data);
    data.data=links[7].JointAngle;    pub7.publish(data);
    data.data=links[8].JointAngle+RollModifieds(1,0);    pub8.publish(data);
    data.data=links[9].JointAngle+PitchModifieds;    pub9.publish(data);
    data.data=links[10].JointAngle;    pub10.publish(data);
    data.data=links[11].JointAngle+theta_l;    pub11.publish(data);
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
double teta_motor_L,teta_motor_R;//pitch
double phi_motor_L,phi_motor_R;//roll
double PitchModified;

int qc_offset[12];
bool qc_initial_bool;

//
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

    bump_notpushed[0]=1012;bump_notpushed[1]= 924;bump_notpushed[2]=3038;bump_notpushed[3]=3098;
    bump_notpushed[4]=3042;bump_notpushed[5]=3008;bump_notpushed[6]=1133;bump_notpushed[7]=1016;
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
    if (qc_initial_bool){

        for (int i = 0; i < 12; ++i) {
            qc_offset[i]=int(msg.position[i+1]);

        }

        qc_initial_bool=false;

        ROS_INFO("Offset=%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\nInitialized!",
                 qc_offset[0],qc_offset[1],qc_offset[2],qc_offset[3],qc_offset[4],
                qc_offset[5],qc_offset[6],qc_offset[7],qc_offset[8],qc_offset[9],
                qc_offset[10],qc_offset[11]);}
    }

double Fzl,Fzr,Mxl,Mxr;
void FT_left_feedback(const geometry_msgs::Wrench &msg){
 Fzl=msg.force.z;
 Mxl=msg.torque.y;
}

void FT_right_feedback(const geometry_msgs::Wrench &msg){
 Fzr=msg.force.z;
 Mxr=msg.torque.x;
}
bool wrench_init_bool;
void WrenchHomming(){
    double threshold= .5;
    double k=10;
    double max=20;
    if(wrench_init_bool && ((abs(Mxl)>threshold) || (abs(Mxr)>threshold))){
        ROS_INFO_ONCE("wrench initializing!");
        if (abs(Mxl)>threshold){
            if(k*Mxl<-max){qc_offset[5]-=max;}
            else if(k*Mxl>max){qc_offset[5]+=max;}
            else {qc_offset[5]+=k*Mxl;}
        }
        if (abs(Mxr)>threshold){
            if(k*Mxr<-max){qc_offset[0]-=max;}
            else if(k*Mxr>max){qc_offset[0]+=max;}
            else {qc_offset[0]+=k*Mxr;}
        }

    }
    if((abs(Mxl)<threshold) && (abs(Mxr)<threshold)){
    wrench_init_bool=false;
    ROS_INFO_ONCE("initilalzed!");
    }
}

MatrixXd quater2rot(double w,double x,double y, double z){
    MatrixXd R(3,3);
    R<<w*w+x*x-y*y-z*z,2*x*y-2*w*z,2*x*z+2*w*y,
            2*x*y+2*w*z,w*w-x*x+y*y-z*z,2*y*z-2*w*x,
            2*x*z-2*w*y,2*y*z+2*w*x,w*w-x*x-y*y+z*z;
    return R;
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


Robot SURENA;//model of robot & kinematics funcs(IK & FK)
TaskSpaceOnline3 OnlineTaskSpace;
QList<LinkM> links;
MatrixXd PoseRoot;//position of pelvis respected to global coordinate
MatrixXd PoseRFoot;//position of right ankle joint respected to global coordinate
MatrixXd PoseLFoot;//position of left ankle joint respected to global coordinate
double GlobalTime;
double  DurationOfStartPhase;
double  DurationOfendPhase;



void StartPhase(){
    if ( GlobalTime<=DurationOfStartPhase) {

        MinimumJerkInterpolation Coef;
        MatrixXd ZPosition(1,2);
        ZPosition<<OnlineTaskSpace.InitialPelvisHeight,OnlineTaskSpace.ReferencePelvisHeight;
        MatrixXd ZVelocity(1,2);
        ZVelocity<<0.000,0.000;
        MatrixXd ZAcceleration(1,2);
        ZAcceleration<<0.000,0.000;
        MatrixXd Time(1,2);
        Time<<0,DurationOfStartPhase;
        MatrixXd CoefZStart =Coef.Coefficient(Time,ZPosition,ZVelocity,ZAcceleration);
        double zStart=0;
        double yStart=0;
        double xStart=0;
        GlobalTime=GlobalTime+OnlineTaskSpace._timeStep;

        MatrixXd outputZStart= Coef.GetAccVelPos(CoefZStart,GlobalTime,0,5);
        zStart=outputZStart(0,0);

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

        SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot);
        SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot);

        MinimumJerkInterpolation CoefOffline;
        double D_pitch=-1*OnlineTaskSpace.HipPitchModification*(M_PI/180);
        double TstartofPitchModify=DurationOfStartPhase/6;
        double TendofPitchModify=DurationOfStartPhase;
        double D_time=TendofPitchModify-TstartofPitchModify;
        if (GlobalTime>=TstartofPitchModify && GlobalTime<=TendofPitchModify){
            MatrixXd Ct_pitch_st(1,2);
            Ct_pitch_st<<0 ,D_time;
            MatrixXd Cp_pitch_st(1,2);
            Cp_pitch_st<<0, D_pitch;
            MatrixXd Cv_pitch_st(1,2);
            Cv_pitch_st<<0 ,0;
            MatrixXd Ca_pitch_st(1,2);
            Ca_pitch_st<<0 ,0;
            MatrixXd C_pitch_st=CoefOffline.Coefficient(Ct_pitch_st,Cp_pitch_st,Cv_pitch_st,Ca_pitch_st);

            MatrixXd output=Coef.GetAccVelPos(C_pitch_st,GlobalTime-(TstartofPitchModify),0,5);
            PitchModified=output(0,0);

        }


    }
}

void EndPhase(){

    if (GlobalTime>=(DurationOfStartPhase+OnlineTaskSpace.MotionTime) && GlobalTime<=DurationOfendPhase+DurationOfStartPhase+OnlineTaskSpace.MotionTime) {

        MinimumJerkInterpolation Coef;
        MatrixXd ZPosition(1,2);
        //ZPosition<<Pz(0,0),Pz(0,0)-OnlineTaskSpace.ReferencePelvisHeight+OnlineTaskSpace.InitialPelvisHeight;//this one should be edited
        ZPosition<<OnlineTaskSpace.ReferencePelvisHeight,OnlineTaskSpace.InitialPelvisHeight;//this one should be edited
        MatrixXd ZVelocity(1,2);
        ZVelocity<<0.000,0.000;
        MatrixXd ZAcceleration(1,2);
        ZAcceleration<<0.000,0.000;

        MatrixXd Time(1,2);
        Time<<DurationOfStartPhase+OnlineTaskSpace.MotionTime,DurationOfStartPhase+OnlineTaskSpace.MotionTime+DurationOfendPhase;
        MatrixXd CoefZStart =Coef.Coefficient(Time,ZPosition,ZVelocity,ZAcceleration);

        double zStart=0;
        double yStart=0;
        double xStart=0;
        GlobalTime=GlobalTime+OnlineTaskSpace._timeStep;

        MatrixXd outputZStart= Coef.GetAccVelPos(CoefZStart,GlobalTime,DurationOfStartPhase+OnlineTaskSpace.MotionTime,5);
        zStart=outputZStart(0,0);

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


        MinimumJerkInterpolation CoefOffline;
        double D_pitch=-1*OnlineTaskSpace.HipPitchModification*(M_PI/180);
        double TstartofPitchModify=DurationOfendPhase/6+DurationOfStartPhase+OnlineTaskSpace.MotionTime;
        double TendofPitchModify=DurationOfendPhase*5/6+DurationOfStartPhase+OnlineTaskSpace.MotionTime;
        double D_time=TendofPitchModify-TstartofPitchModify;
        if (GlobalTime>=TstartofPitchModify && GlobalTime<=TendofPitchModify){
            MatrixXd Ct_pitch_st(1,2);
            Ct_pitch_st<<-D_time, 0;
            MatrixXd Cp_pitch_st(1,2);
            Cp_pitch_st<<D_pitch, 0;
            MatrixXd Cv_pitch_st(1,2);
            Cv_pitch_st<<0 ,0;
            MatrixXd Ca_pitch_st(1,2);
            Ca_pitch_st<<0 ,0;
            MatrixXd C_pitch_st=CoefOffline.Coefficient(Ct_pitch_st,Cp_pitch_st,Cv_pitch_st,Ca_pitch_st);

            MatrixXd output=Coef.GetAccVelPos(C_pitch_st,GlobalTime-(TendofPitchModify),-D_time,5);
            PitchModified=output(0,0);

        }



    }
}


void ankleAdaptation(){
    //parameters of ankle adaptation
    double k1,k2,k3,k4;
    k1=0.000015;    k2=0.000015;    k3=0.000015;    k4=0.000015;

if (true){//left
        //-----------------Pitch left ankle motor control---------------//
        if (abs(b-a)>=abs(c-d)) {
            if (abs(a-b)<100) {
                teta_motor_L=1*teta_motor_L+k1*(OnlineTaskSpace.LeftFootOrientationAdaptator==true)*(a-b);
            }
        }
        else {
            if (abs(d-c)<100) {
                teta_motor_L=1*teta_motor_L+k1*(OnlineTaskSpace.LeftFootOrientationAdaptator==true)*(d-c);
            }

        }


        //----------------Roll left ankle motor control---------------//
        if (abs(c-b)>=abs(d-a)) {
            if (abs(c-b)<100) {
                phi_motor_L=1*phi_motor_L+k2*(OnlineTaskSpace.LeftFootOrientationAdaptator==true)*(c-b);
            }
        }
        else {
            if (abs(a-d)<100) {
                phi_motor_L=1*phi_motor_L+k2*(OnlineTaskSpace.LeftFootOrientationAdaptator==true)*(d-a);
            }
        }
    }


if(true){//right

        //-----------------Pitch left ankle motor control---------------//
        if (abs(f-e)>=abs(g-h)) {
            if (abs(e-f)<100) {
                teta_motor_R=1*teta_motor_R+k3*(OnlineTaskSpace.RightFootOrientationAdaptator==true)*(e-f);
            }
        }
        else {
            if (abs(h-g)<100) {
                teta_motor_R=1*teta_motor_R+k3*(OnlineTaskSpace.RightFootOrientationAdaptator==true)*(h-g);
            }
        }


        //----------------Roll left ankle motor control---------------//
        if (abs(g-f)>=abs(h-e)) {
            if (abs(g-f)<100) {
                phi_motor_R=1*phi_motor_R+k4*(OnlineTaskSpace.RightFootOrientationAdaptator==true)*(g-f);
            }
        }
        else {
            if (abs(e-h)<100) {
                phi_motor_R=1*phi_motor_R+k4*(OnlineTaskSpace.RightFootOrientationAdaptator==true)*(h-e);
            }
        }
    }


    //------------------------saturation of ankle motors----------------------------//
    if ((abs(phi_motor_L))>0.9) {phi_motor_L=0.9;}
    if ((abs(teta_motor_L))>0.9) {teta_motor_L=0.9;}
    if ((abs(phi_motor_R))>0.9) {phi_motor_L=0.9;}
    if ((abs(teta_motor_R))>0.9) {teta_motor_R=0.9;}
}

MatrixXd Coef_teta_motor_L; MatrixXd Coef_phi_motor_L;
MatrixXd Coef_teta_motor_R; MatrixXd Coef_phi_motor_R;

void ankleAdaptationDischargeCoef(){
    MatrixXd Time(1,2);MatrixXd Position(1,2);MatrixXd Velocity(1,2);MatrixXd Acceleration(1,2);
    Time<<0,DurationOfStartPhase;
    Velocity<<0.000,0.000;
    Acceleration<<0.000,0.000;
    Position<<teta_motor_L,0;
    Coef_teta_motor_L =CoefGen.Coefficient(Time,Position,Velocity,Acceleration);
    Position<<teta_motor_R,0;
    Coef_teta_motor_R =CoefGen.Coefficient(Time,Position,Velocity,Acceleration);
    Position<<phi_motor_L,0;
    Coef_phi_motor_L =CoefGen.Coefficient(Time,Position,Velocity,Acceleration);
    Position<<phi_motor_L,0;
    Coef_phi_motor_R =CoefGen.Coefficient(Time,Position,Velocity,Acceleration);
}

void ankleAdaptationDischarge(){
    teta_motor_L= CoefGen.GetAccVelPos(Coef_teta_motor_L,GlobalTime,0,5)(0,0);
    teta_motor_R= CoefGen.GetAccVelPos(Coef_teta_motor_R,GlobalTime,0,5)(0,0);
    phi_motor_L= CoefGen.GetAccVelPos(Coef_phi_motor_L,GlobalTime,0,5)(0,0);
    phi_motor_R= CoefGen.GetAccVelPos(Coef_phi_motor_R,GlobalTime,0,5)(0,0);
}

int main(int argc, char **argv)
{
    vector<double> cntrl(13);
    QCgenerator QC;
    for (int i = 0; i < 12; ++i) {
        qc_offset[i]=0;
    }
    qc_initial_bool=true;
    bump_initialize=true;
    wrench_init_bool=true;

    teta_motor_L=0;
    teta_motor_R=0;
    phi_motor_L=0;
    phi_motor_R=0;

    double footSensorSaturation=75;//if all sensors data are bigger than this amount, this means the foot is landed on the ground
    double footSensorthreshold=4;// will start orientaition correction

    GlobalTime=0;
    DurationOfStartPhase=2;
    DurationOfendPhase=6;

    MatrixXd RollModified(2,1);RollModified<<0,0;//parameters for hip roll angles charge, for keep pelvis straight
    PitchModified=0;
    PoseRoot.resize(6,1); //pelvis trajectory from taskspace_online,xyzrpy
    PoseRFoot.resize(6,1);//right ankle joint trajectory from taskspace_online,xyzrpy
    PoseLFoot.resize(6,1);//left ankle joint trajectory from taskspace_online,xyzrpy

    //*******************This part of code is for initialization of joints of the robot for walking**********************************
    int count = 0;

    ros::init(argc, argv, "myNode");
    ros::NodeHandle nh;
    ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",1000);
    ros::Publisher  contact_flag  = nh.advertise<std_msgs::Int32MultiArray>("contact_flag_timing",100);
    ros::Subscriber sub = nh.subscribe("/surena/bump_sensor_state", 1000, receiveFootSensor);
    ros::Subscriber ft_left = nh.subscribe("/surena/ft_l_state",1000,FT_left_feedback);
    ros::Subscriber ft_right = nh.subscribe("/surena/ft_r_state",1000,FT_right_feedback);
    ros::Subscriber qcinit = nh.subscribe("/surena/inc_joint_state", 1000, qc_initial);
    if(true){//gazebo publishers
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
    while (ros::ok())
    {

        //  for robot test musbe uncommented
        qc_initial_bool=false;
        bump_initialize=false;
        wrench_init_bool=false;

        if (qc_initial_bool) {
            ROS_INFO_ONCE("qc is initializing!");
            ros::spinOnce();
            continue;
        }

        if(wrench_init_bool){
            WrenchHomming();
            for(int  i = 0;i < 12;i++){msg.data.push_back(qc_offset[i]);}
            for(int  i = 12;i < 28;i++){msg.data.push_back(0);}
            chatter_pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }


        if(OnlineTaskSpace.localTiming<.1){contact_flag_timing=-contact_flag_timing;}//flag to show expected contact according to timing,sign of flag changes

        //-------------for detecting the first contact of Left foot with ground-------------//
        //-------------flag to show contact detected by sensors,sign of flag changes-------------//
        if (( (a)<footSensorthreshold && (b)<footSensorthreshold && (c)<footSensorthreshold && (d)<footSensorthreshold)){
        fullcontactL=false;
        firstcontactL=true;
        }
        // if (firstcontact==true &&( (a)>=footSensorSaturation || (b)>=footSensorSaturation || (c)>=footSensorSaturation || (d)>=footSensorSaturation)){
        if (firstcontactL==true &&( (a)>=footSensorthreshold || (b)>=footSensorthreshold || (c)>=footSensorthreshold || (d)>=footSensorthreshold)){

            contact_flag_sensor2=-contact_flag_sensor2;
            firstcontactL=false;
            ROS_INFO("shalap [%f  %f] a=%d b=%d c=%d d=%d", OnlineTaskSpace.localTiming,OnlineTaskSpace.globalTime,a,b,c,d);
        }
        //-------------for detecting the full contact of Left foot with ground-------------//
        //-------------flag to show contact detected by sensors,sign of flag changes-------------//
        if ((!fullcontactL) && (a)>=footSensorSaturation && (b)>=footSensorSaturation && (c)>=footSensorSaturation && (d)>=footSensorSaturation){
            contact_flag_sensor=-contact_flag_sensor;
            ROS_INFO("left swing foot landing is successful = [%f  %f] a=%d b=%d c=%d d=%d", OnlineTaskSpace.localTiming,OnlineTaskSpace.globalTime,a,b,c,d);
            fullcontactL=true;
        }



        //-------------for detecting the first contact of Right foot with ground-------------//
        //-------------flag to show contact detected by sensors,sign of flag changes-------------//
        if (( (e)<footSensorthreshold && (f)<footSensorthreshold && (g)<footSensorthreshold && (h)<footSensorthreshold)){
        fullcontactR=false;
        firstcontactR=true;
        }
       // if (firstcontact==true && ( (e)>=footSensorSaturation || (f)>=footSensorSaturation || (g)>=footSensorSaturation || (h)>=footSensorSaturation)){
        if (firstcontactR==true &&( (e)>=footSensorthreshold || (f)>=footSensorthreshold || (g)>=footSensorthreshold || (h)>=footSensorthreshold)){

            contact_flag_sensor2=-contact_flag_sensor2;
            firstcontactR=false;
            ROS_INFO("shooloop[%f  %f]  e=%d f=%d g=%d h=%d", OnlineTaskSpace.localTiming,OnlineTaskSpace.globalTime,e,f,g,h);
        }

        //-------------for detecting the full contact of Right foot with ground-------------//
        if ((!fullcontactR) && (e)>=footSensorSaturation && (f)>=footSensorSaturation && (g)>=footSensorSaturation && (h)>=footSensorSaturation){
            contact_flag_sensor=-contact_flag_sensor;
            ROS_INFO("Right swing foot landing is successful= [%f  %f]  e=%d f=%d g=%d h=%d", OnlineTaskSpace.localTiming,OnlineTaskSpace.globalTime,e,f,g,h);
            fullcontactR=true;
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

            if ((OnlineTaskSpace.StepNumber==1) && (OnlineTaskSpace.localTiming>=OnlineTaskSpace.TStart) ) {
                OnlineTaskSpace.localTiming=OnlineTaskSpace._timeStep;//0.001999999999000000;
                OnlineTaskSpace.localtimingInteger=1;
                OnlineTaskSpace.StepNumber=OnlineTaskSpace.StepNumber+1;
             //   KLtemp=false;

            }

            else if ((OnlineTaskSpace.localtimingInteger>=NumberOfTimeStep) &&   (OnlineTaskSpace.StepNumber>1    &&   OnlineTaskSpace.StepNumber<(OnlineTaskSpace.NStep+2))) {
                OnlineTaskSpace.StepNumber=OnlineTaskSpace.StepNumber+1;
                OnlineTaskSpace.localTiming=OnlineTaskSpace._timeStep;//0.001999999999000000;
                OnlineTaskSpace.localtimingInteger=1;


            }

            OnlineTaskSpace.currentLeftFootX2=links[12].PositionInWorldCoordinate(0);
            OnlineTaskSpace.currentLeftFootY2=links[12].PositionInWorldCoordinate(1);
            OnlineTaskSpace.currentLeftFootZ=links[12].PositionInWorldCoordinate(2);

            OnlineTaskSpace.currentRightFootX2=links[6].PositionInWorldCoordinate(0);
            OnlineTaskSpace.currentRightFootY2=links[6].PositionInWorldCoordinate(1);
            OnlineTaskSpace.currentRightFootZ=links[6].PositionInWorldCoordinate(2);


            MatrixXd m=OnlineTaskSpace.AnkleTrajectory(OnlineTaskSpace.globalTime,OnlineTaskSpace.StepNumber,OnlineTaskSpace.localTiming);

            m1=m(0,0); m2=m(1,0); m3=m(2,0); m4=m(3,0);
            m5=m(4,0); m6=m(5,0); m7=m(6,0); m8=m(7,0);
//ROS_INFO("g.t=%f,l.t=%f",OnlineTaskSpace.globalTime,OnlineTaskSpace.localTiming);
//            ROS_INFO("m1=%f,m2=%f,m3=%f,m4=%f,m5=%f,m6=%f,m7=%f,m8=%f,",m1,m2,m3,m4,m5,m6,m7,m8);

            RollModified=OnlineTaskSpace.RollAngleModification(OnlineTaskSpace.globalTime);
            P=OnlineTaskSpace.PelvisTrajectory (OnlineTaskSpace.globalTime);

            OnlineTaskSpace.globalTime=OnlineTaskSpace.globalTime+OnlineTaskSpace._timeStep;
            OnlineTaskSpace.localTiming=OnlineTaskSpace.localTiming+OnlineTaskSpace._timeStep;
            OnlineTaskSpace.localtimingInteger= OnlineTaskSpace.localtimingInteger+1;


            if (round(OnlineTaskSpace.globalTime)<=round(OnlineTaskSpace.MotionTime)){


                //if you want to have modification of height of pelvis please active the Pz(0,0) instead of P(2,0)
                PoseRoot<<P(0,0),
                        P(1,0),
                        P(2,0),// Pz(0,0)
                        0,
                        0,
                        0;

                PoseRFoot<<m5,
                        m6,
                        m7,
                        0,
                        -1*m8*(M_PI/180),
                        0;

                PoseLFoot<<m1,
                        m2,
                        m3,
                        0,
                        -1*m4*(M_PI/180),
                        0;


                SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot);
                SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot);

                SURENA.ForwardKinematic(1);
            }

        }
        ankleAdaptation();
        EndPhase();


        links = SURENA.GetLinks();


        double ankle_adaptation_switch=0;// 1 for activating adaptation 0 for siktiring adaptation
        double k_roll_corr=0;
        cntrl[0]=0.0;
        cntrl[1]=(links[1].JointAngle);
        cntrl[2]=(links[2].JointAngle+1*RollModified(0,0));//+k_roll_corr*(links[2].JointAngle-roll_absoulte[0])
        cntrl[3]=links[3].JointAngle+PitchModified;
        cntrl[4]=links[4].JointAngle;
        cntrl[5]=links[5].JointAngle+ankle_adaptation_switch*teta_motor_R;//pitch
        cntrl[6]=links[6].JointAngle+ankle_adaptation_switch*(phi_motor_R);//roll
        cntrl[7]=links[7].JointAngle;
        cntrl[8]=links[8].JointAngle+1*RollModified(1,0);//+k_roll_corr*(links[8].JointAngle-roll_absoulte[1])
        cntrl[9]=links[9].JointAngle+PitchModified;
        cntrl[10]=links[10].JointAngle;
        cntrl[11]=links[11].JointAngle+ankle_adaptation_switch*teta_motor_L;
        cntrl[12]=links[12].JointAngle+ankle_adaptation_switch*(phi_motor_L);


        vector<int> qref(12);
        qref=QC.ctrldata2qc(cntrl);

        msg.data.clear();
        bool left_first=true;//right support in first step
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

        for(int  i = 12;i < 28;i++)
        {
            msg.data.push_back(0);
        }

        chatter_pub.publish(msg);

        SendGazebo(links,0*RollModified,0*PitchModified,0*teta_motor_R,0*phi_motor_R,0*teta_motor_L,0*phi_motor_L);


        msg_contact_flag.data.clear();
        msg_contact_flag.data.push_back(contact_flag_timing);
        msg_contact_flag.data.push_back(contact_flag_sensor);
        msg_contact_flag.data.push_back(contact_flag_sensor2);

        contact_flag.publish(msg_contact_flag);

        if(count%20==0){ //use to print once in n steps
            // ROS_INFO("");
             ROS_INFO("I heard data of sensors :t=%f [%d %d %d %d] & [%d %d %d %d]",OnlineTaskSpace.globalTime,a,b,c,d,e,f,g,h);
             ROS_INFO("teta_motor_L=%f,teta_motor_R=%f,phi_motor_L=%f,phi_motor_R=%f",teta_motor_L,teta_motor_R,phi_motor_L,phi_motor_R);
        }

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}


