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
#include "pidcontroller.h"
#include<rosgraph_msgs/Clock.h>
#include <std_msgs/Empty.h>
#include "std_srvs/Empty.h"
#include "qcgenerator.h"
#include <qthread.h>
#include "/home/cast/humanoid/surena4/devel/include/xsens_msgs/orientationEstimate.h"

#include<gazebo_msgs/ApplyBodyWrenchRequest.h>
#include <gazebo_msgs/GetLinkState.h>
#include<gazebo_msgs/LinkStates.h>
#include <nav_msgs/Odometry.h>
#include "std_srvs/Empty.h"
#include "qcgenerator.h"


using namespace  std;
using namespace  Eigen;

//sensor imu----------------------------------------------sensor imu
//sensor imu-----------------------------------------------sensor imu
//void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
//{
//  ROS_INFO("I heard: [%f]", msg->orientation.x);
//}
//sensor imu----------------------------------------------sensor imu
//sensor imu----------------------------------------------sensor imu

//ros::NodeHandle nh;


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

double teta_L=0;
double phi_L=0;

double teta_R=0;
double phi_R=0;

double teta_center=0;
double phi_center=0;

PIDController teta_pid_L;
PIDController phi_pid_L;
PIDController teta_pid_R;
PIDController phi_pid_R;

PIDController teta_pid_center;
PIDController phi_pid_center;

double p_teta,i_teta,d_teta,p_phi,i_phi,d_phi,dt,rate;
double p_teta_center,i_teta_center,d_teta_center,p_phi_center,i_phi_center,d_phi_center;
double teta_motor_L=0;
double teta_motor_R=0;
double phi_motor_L=0;
double phi_motor_R=0;
double teta_motor_center=0;
double phi_motor_center=0;
double timestep=.01;
double time_=0;

double quaternion2ankle_pitch(double q0,double q1,double q2,double q3){
    double R11,R32,R33,R31,theta;
    //    R11=q0*q0+q1*q1-q2*q2-q3*q3;
    R31=2*(q1*q3-q0*q2);
    //    theta=atan2(-R31,R11);
    R32=2*(q0*q1+q2*q3);
    R33=q0*q0-q1*q1-q2*q2+q3*q3;
    theta=atan2(-R31,sqrt(R32*R32+R33*R33));

    return theta;
}
double quaternion2ankle_roll(double q0,double q1,double q2,double q3){
        double R23,R22,phi,R33,R32;
    //R23=2*(q2*q3-q0*q1);
    //R22=q0*q0-q1*q1+q2*q2-q3*q3;
    //phi=atan2(-R23,R22);
    R32=2*(q0*q1+q2*q3);
    R33=q0*q0-q1*q1-q2*q2+q3*q3;
    phi=atan2(R32,R33);
    return phi;
}

geometry_msgs::Pose GetLinkPosition(QString linkName ,const gazebo_msgs::LinkStates::ConstPtr linkStates)
{
    geometry_msgs::Pose result;
    const std::vector<std::string> &names = linkStates->name;
    const std::vector<geometry_msgs::Pose> positions = linkStates->pose;



    for(int i=0;i<names.size();i++)
    {


        if(QString::fromStdString( names[i]).contains(linkName))
        {

            return  positions[i];

        }
    }

    return result;
}

void chatterCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
    const std::vector<std::string> &names = msg->name;
    //const std::vector<geometry_msgs::Pose> psitions = msg->pose;
    //  double distance=0;

    geometry_msgs::Pose parentPosition_L=GetLinkPosition("robot::LLeg_Foot_Link",msg);
//    qDebug()<<"we are here:"<< parentPosition.orientation.x<<parentPosition.orientation.y<<parentPosition.orientation.z<<parentPosition.orientation.w;
   teta_L= quaternion2ankle_pitch( parentPosition_L.orientation.w,parentPosition_L.orientation.x,parentPosition_L.orientation.y,parentPosition_L.orientation.z);
   phi_L=quaternion2ankle_roll( parentPosition_L.orientation.w,parentPosition_L.orientation.x,parentPosition_L.orientation.y,parentPosition_L.orientation.z);

   geometry_msgs::Pose parentPosition_R=GetLinkPosition("robot::LLeg_Foot_Link",msg);
//    qDebug()<<"we are here:"<< parentPosition_R.orientation.x<<parentPosition_R.orientation.y<<parentPosition_R.orientation.z<<parentPosition_R.orientation.w;
  teta_R= quaternion2ankle_pitch( parentPosition_R.orientation.w,parentPosition_R.orientation.x,parentPosition_R.orientation.y,parentPosition_R.orientation.z);
  phi_R=quaternion2ankle_roll( parentPosition_R.orientation.w,parentPosition_R.orientation.x,parentPosition_R.orientation.y,parentPosition_R.orientation.z);

  geometry_msgs::Pose parentPosition_center=GetLinkPosition("robot::pelvis",msg);
//    qDebug()<<"we are here:"<< parentPosition_center.orientation.x<<parentPosition_center.orientation.y<<parentPosition_center.orientation.z<<parentPosition_center.orientation.w;
 teta_center= quaternion2ankle_pitch( parentPosition_center.orientation.w,parentPosition_center.orientation.x,parentPosition_center.orientation.y,parentPosition_center.orientation.z);
 phi_center=quaternion2ankle_roll( parentPosition_center.orientation.w,parentPosition_center.orientation.x,parentPosition_center.orientation.y,parentPosition_center.orientation.z);


   qDebug()<<"teta_L:"<< teta_L*180/M_PI<<", Phi_L="<<phi_L*180/M_PI<<"teta_R:"<< teta_R*180/M_PI<<", Phi_R="<<phi_R*180/M_PI<<"teta_center:"<< teta_center*180/M_PI<<", Phi_center="<<phi_center*180/M_PI;
    //qDebug()<<QString::fromStdString( names[9])<<psitions[9].position.x<<psitions[9].position.y<<psitions[9].position.z;
    //ROS_INFO("I heard");


}

void  SendGazebo(QList<LinkM> links){
    if(links.count()<28){qDebug()<<"index err";return;}
    std_msgs::Float64 data;
    data.data=links[1].JointAngle;
    pub1.publish(data);
    data.data=links[2].JointAngle;
    pub2.publish(data);
    data.data=links[3].JointAngle;
    pub3.publish(data);
    data.data=links[4].JointAngle;
    pub4.publish(data);
    data.data=links[5].JointAngle;
    pub5.publish(data);
    data.data=links[6].JointAngle;
    pub6.publish(data);
    data.data=links[7].JointAngle;
    pub7.publish(data);
    data.data=links[8].JointAngle;
    pub8.publish(data);
    data.data=links[9].JointAngle;
    pub9.publish(data);
    data.data=links[10].JointAngle;
    pub10.publish(data);
    data.data=links[11].JointAngle;
    pub11.publish(data);
    data.data=links[12].JointAngle;
    pub12.publish(data);
    data.data=links[13].JointAngle;
    pub13.publish(data);
    data.data=links[14].JointAngle;
    pub14.publish(data);
    data.data=links[15].JointAngle;
    pub15.publish(data);
    data.data=links[16].JointAngle;
    pub16.publish(data);
    data.data=links[17].JointAngle;
    pub17.publish(data);
    data.data=links[18].JointAngle;
    pub18.publish(data);
    data.data=links[19].JointAngle;
    pub19.publish(data);
    data.data=links[20].JointAngle;
    pub20.publish(data);
    data.data=links[21].JointAngle;
    pub21.publish(data);
    data.data=links[22].JointAngle;
    pub22.publish(data);
    data.data=links[23].JointAngle;
    pub23.publish(data);
    data.data=links[24].JointAngle;
    pub24.publish(data);
    data.data=links[25].JointAngle;
    pub25.publish(data);
    data.data=links[26].JointAngle;
    pub26.publish(data);
    data.data=links[27].JointAngle;
    pub27.publish(data);
    data.data=links[28].JointAngle;
    pub28.publish(data);
}


void  SendGazebo_with_control(QList<LinkM> links,vector<double> cntrl){
    if(links.count()<28){qDebug()<<"index err";return;}
    std_msgs::Float64 data;
    data.data=links[1].JointAngle+cntrl[1];
    pub1.publish(data);
    data.data=links[2].JointAngle+cntrl[2];
    pub2.publish(data);
    data.data=links[3].JointAngle+cntrl[3];
    pub3.publish(data);
    data.data=links[4].JointAngle+cntrl[4];
    pub4.publish(data);
    data.data=links[5].JointAngle+cntrl[5];
    pub5.publish(data);
    data.data=links[6].JointAngle+cntrl[6];
    pub6.publish(data);
    data.data=links[7].JointAngle+cntrl[7];
    pub7.publish(data);
    data.data=links[8].JointAngle+cntrl[8];
    pub8.publish(data);
    data.data=links[9].JointAngle+cntrl[9];
    pub9.publish(data);
    data.data=links[10].JointAngle+cntrl[10];
    pub10.publish(data);
    data.data=links[11].JointAngle+cntrl[11];
    pub11.publish(data);
    data.data=links[12].JointAngle+cntrl[12];
    pub12.publish(data);
    data.data=links[13].JointAngle;
    pub13.publish(data);
    data.data=links[14].JointAngle;
    pub14.publish(data);
    data.data=links[15].JointAngle;
    pub15.publish(data);
    data.data=links[16].JointAngle;
    pub16.publish(data);
    data.data=links[17].JointAngle;
    pub17.publish(data);
    data.data=links[18].JointAngle;
    pub18.publish(data);
    data.data=links[19].JointAngle;
    pub19.publish(data);
    data.data=links[20].JointAngle;
    pub20.publish(data);
    data.data=links[21].JointAngle;
    pub21.publish(data);
    data.data=links[22].JointAngle;
    pub22.publish(data);
    data.data=links[23].JointAngle;
    pub23.publish(data);
    data.data=links[24].JointAngle;
    pub24.publish(data);
    data.data=links[25].JointAngle;
    pub25.publish(data);
    data.data=links[26].JointAngle;
    pub26.publish(data);
    data.data=links[27].JointAngle;
    pub27.publish(data);
    data.data=links[28].JointAngle;
    pub28.publish(data);
}

void RecievTime(const rosgraph_msgs::Clock & msg)
{
    time_ =double(msg.clock.nsec)*1e-9 + double(msg.clock.sec);
}

void RecievIMULeft(const sensor_msgs::Imu & msg)
{

    teta_L= msg.orientation.y;
    phi_L=msg.orientation.x;
    //ROS_INFO("Theta_L:[%f] Phi_L:[%f]",  teta_L*180/3.141592,phi_L*180/3.141592);

}



void RecievIMURight(const sensor_msgs::Imu & msg)
{
    teta_R= msg.orientation.y;
    phi_R=msg.orientation.x;
    //ROS_INFO("Theta_R:[%f] Phi_R:[%f]",  teta_R*180/3.141592,phi_R*180/3.141592);
}

void RecievIMUCenter(const xsens_msgs::orientationEstimate & msg)
{
    teta_center= msg.pitch;
   phi_center=msg.roll;
    //ROS_INFO("Theta_R:[%f] Phi_R:[%f]",  teta_R*180/3.141592,phi_R*180/3.141592);
}


int main(int argc, char **argv)
{
    //check _timesteps
    std_srvs::Empty emptyCall;
    QElapsedTimer timer;
    vector<int> qref(12);
    Robot SURENA;
    TaskSpaceOffline SURENAOffilneTaskSpace;
    QList<LinkM> links;
    MatrixXd PoseRoot;
    MatrixXd PoseRFoot;
    MatrixXd PoseLFoot;
    double dt;
    double hipRoll=0;
    double StartTime=0;
    double RollTime=0;
    double WalkTime=0;
    double  DurationOfStartPhase=6;
    double  DurationOfendPhase=6;
    //SURENAOffilneTaskSpace.GetAccVelPos();
    bool startPhase=true;
    bool endPhase=true;
    PoseRoot.resize(6,1);
    PoseRFoot.resize(6,1);
    PoseLFoot.resize(6,1);

    //*******************This part of code is for initialization of joints of the robot for walking**********************************
    int count = 0;

    ros::init(argc, argv, "myNode");

    ros::NodeHandle nh;
    ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",100);
    ros::Subscriber  IMULeft = nh.subscribe("/yei2000154", 100, RecievIMULeft);
    ros::Subscriber  IMURight = nh.subscribe("/yei200015B", 100, RecievIMURight);
    ros::Subscriber  IMULCenter = nh.subscribe("/mti/filter/orientation", 100, RecievIMUCenter);

    ros::ServiceClient tareLeft= nh.serviceClient<std_srvs::Empty>("/Tareyei2000154");
    ros::ServiceClient tareRight= nh.serviceClient<std_srvs::Empty>("/Tareyei200015B");

    ros::Subscriber sub = nh.subscribe("/gazebo/link_states", 100, &chatterCallback);

    pub1  = nh.advertise<std_msgs::Float64>("rrbot/joint1_position_controller/command",100);
    pub2  = nh.advertise<std_msgs::Float64>("rrbot/joint2_position_controller/command",100);
    pub3  = nh.advertise<std_msgs::Float64>("rrbot/joint3_position_controller/command",100);
    pub4  = nh.advertise<std_msgs::Float64>("rrbot/joint4_position_controller/command",100);
    pub5  = nh.advertise<std_msgs::Float64>("rrbot/joint5_position_controller/command",100);
    pub6  = nh.advertise<std_msgs::Float64>("rrbot/joint6_position_controller/command",100);
    pub7  = nh.advertise<std_msgs::Float64>("rrbot/joint7_position_controller/command",100);
    pub8  = nh.advertise<std_msgs::Float64>("rrbot/joint8_position_controller/command",100);
    pub9  = nh.advertise<std_msgs::Float64>("rrbot/joint9_position_controller/command",100);
    pub10 = nh.advertise<std_msgs::Float64>("rrbot/joint10_position_controller/command",100);
    pub11 = nh.advertise<std_msgs::Float64>("rrbot/joint11_position_controller/command",100);
    pub12 = nh.advertise<std_msgs::Float64>("rrbot/joint12_position_controller/command",100);
    pub13 = nh.advertise<std_msgs::Float64>("rrbot/joint13_position_controller/command",100);
    pub14 = nh.advertise<std_msgs::Float64>("rrbot/joint14_position_controller/command",100);
    pub15 = nh.advertise<std_msgs::Float64>("rrbot/joint15_position_controller/command",100);
    pub16 = nh.advertise<std_msgs::Float64>("rrbot/joint16_position_controller/command",100);
    pub17 = nh.advertise<std_msgs::Float64>("rrbot/joint17_position_controller/command",100);
    pub18 = nh.advertise<std_msgs::Float64>("rrbot/joint18_position_controller/command",100);
    pub19 = nh.advertise<std_msgs::Float64>("rrbot/joint19_position_controller/command",100);
    pub20 = nh.advertise<std_msgs::Float64>("rrbot/joint20_position_controller/command",100);
    pub21 = nh.advertise<std_msgs::Float64>("rrbot/joint21_position_controller/command",100);
    pub22 = nh.advertise<std_msgs::Float64>("rrbot/joint22_position_controller/command",100);
    pub23 = nh.advertise<std_msgs::Float64>("rrbot/joint23_position_controller/command",100);
    pub24 = nh.advertise<std_msgs::Float64>("rrbot/joint24_position_controller/command",100);
    pub25 = nh.advertise<std_msgs::Float64>("rrbot/joint25_position_controller/command",100);
    pub26 = nh.advertise<std_msgs::Float64>("rrbot/joint26_position_controller/command",100);
    pub27 = nh.advertise<std_msgs::Float64>("rrbot/joint27_position_controller/command",100);
    pub28 = nh.advertise<std_msgs::Float64>("rrbot/joint28_position_controller/command",100);

    rate=100;
    ros::Rate loop_rate(rate);
    std_msgs::Int32MultiArray msg;
    std_msgs::MultiArrayDimension msg_dim;
    dt=1/rate;
    p_teta=0.03;
    p_phi=0.03;
    i_teta=0;i_phi=0;
    d_teta=0;d_phi=0;
    teta_pid_L.Init(dt,0.15,-.15,p_teta,i_teta,d_teta);
    phi_pid_L.Init(dt,.15,-.15,p_phi,i_phi,d_phi);
    teta_pid_R.Init(dt,.15,-.15,p_teta,i_teta,d_teta);
    phi_pid_R.Init(dt,.15,-.15,p_phi,i_phi,d_phi);


    p_teta_center=0.00009;
    p_phi_center=0.00009;
    i_teta_center=0;i_phi_center=0;
    d_teta_center=0;d_phi_center=0;
    teta_pid_center.Init(dt,0.15,-.15,p_teta_center,i_teta_center,d_teta_center);
    phi_pid_center.Init(dt,.15,-.15,p_phi_center,i_phi_center,d_phi_center);


    msg_dim.label = "joint_position";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);

    //sensor imu----------------------------------------------sensor imu
    //sensor imu----------------------------------------------sensor imu
    //  timer.start();
    //  ros::Subscriber sub = nh.subscribe("/mti/sensor/imu", 1000, chatterCallback);
    //sensor imu----------------------------------------------sensor imu
    //sensor imu----------------------------------------------sensor imu
//    msg.data.clear();
//    for(int  i = 0;i < 12;i++)
//    {
//        qref[i] = 0;
//        //  cout << qref[i-1] <<" , "<<flush;
//        msg.data.push_back(qref[i]);
//    }
//    chatter_pub.publish(msg);
//    tareLeft.call(emptyCall);
//    tareRight.call(emptyCall);
//    QThread::msleep(500);





    while (ros::ok())
    {
        // qDebug()<<StartTime;

        if (startPhase==true && StartTime<=DurationOfStartPhase) {
            MinimumJerkInterpolation Coef;
            MatrixXd ZPosition(1,2);
            ZPosition<<0.95100,0.8300;
            MatrixXd ZVelocity(1,2);
            ZVelocity<<0.000,0.000;
            MatrixXd ZAcceleration(1,2);
            ZAcceleration<<0.000,0.000;


            MatrixXd Time(1,2);
            Time<<0,DurationOfStartPhase;
            MatrixXd CoefZStart =Coef.Coefficient(Time,ZPosition,ZVelocity,ZAcceleration);

            //R1=AnkleRollRight
            //R2=AnklePitchRight
            //R3=KneePitchRight
            //R4=HipPitchRight
            //R5=HipRollRight
            //R6=HipYawRight

            //L1=AnkleRollLeft
            //L2=AnklePitchLeft
            //L3=KneePitchLeft
            //L4=HipPitchLeft
            //L5=HipRollLeft
            //L6=HipYawLeft

            //R1=0*(1/2Pi)*(2304)*100  R2=1*-1*(1/2Pi)*(2304)*100   R3=2(1/2Pi)*(2304)*50   R4=3*-1(1/2Pi)*(2304)*80    L2=4*(1/2Pi)*(2304)*100    L1=5*(1/2Pi)*(2304)*100   L3=6*-1*(1/2Pi)*(2304)*50     L4=7*(1/2Pi)*(2304)*80   R6=8*-1*(1/2Pi)*(2304)*120   R5=9*(1/2Pi)*(2304)*120    L5=10*(1/2Pi)*(2304)*120    L6=11*-1*(1/2Pi)*(2304)*120


            //qDebug()<<mm(0,0);
            double zStart=0;
            double yStart=0;
            double xStart=0;
            StartTime=StartTime+SURENAOffilneTaskSpace._timeStep;

            MatrixXd outputZStart= SURENAOffilneTaskSpace.GetAccVelPos(CoefZStart,StartTime,0,5);
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
        }



        if (StartTime>DurationOfStartPhase && StartTime<(DurationOfStartPhase+SURENAOffilneTaskSpace.MotionTime)){

            bool walk=true;
            double m1;
            double m2;
            double m3;
            double m4;
            double m5;
            double m6;
            StartTime=StartTime+SURENAOffilneTaskSpace._timeStep;
            //qDebug()<<StartTime;
            MatrixXd P;
            if(walk==true){
                MatrixXd m=SURENAOffilneTaskSpace.AnkleTrajectory(SURENAOffilneTaskSpace.globalTime);
                m1=m(0,0);
                m2=m(1,0);
                m3=m(2,0);
                m4=m(3,0);
                m5=m(4,0);
                m6=m(5,0);


                P=SURENAOffilneTaskSpace.PelvisTrajectory (SURENAOffilneTaskSpace.globalTime);

                // just some samples for plotting!!!
                //                SURENAOffilneTaskSpace.CoMXVector.append(P(0,0));
                //                SURENAOffilneTaskSpace.timeVector.append(SURENAOffilneTaskSpace.globalTime);
                //                SURENAOffilneTaskSpace.LeftFootXTrajectory.append(m1);



                SURENAOffilneTaskSpace.globalTime=SURENAOffilneTaskSpace.globalTime+SURENAOffilneTaskSpace._timeStep;

                if (round(SURENAOffilneTaskSpace.globalTime)<=round(SURENAOffilneTaskSpace.MotionTime)){

                    PoseRoot<<P(0,0),
                            P(1,0),
                            P(2,0),
                            0,
                            0,
                            0;

                    PoseRFoot<<m4,
                            m5,
                            m6,
                            0,
                            0,
                            0;

                    PoseLFoot<<m1,
                            m2,
                            m3,
                            0,
                            0,
                            0;

                    //// hip roll modification
                    //                    if (SURENAOffilneTaskSpace.LeftSupport==true && SURENAOffilneTaskSpace.HipRollModification==true){

                    //                        SURENA.doIKhipRollModify("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot,-1*hipRoll);
                    //                        SURENA.doIKhipRollModify("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot,0);

                    //                    }

                    //                    else if (SURENAOffilneTaskSpace.HipRollModification==true && SURENAOffilneTaskSpace.LeftSupport!=true )  {
                    //                        SURENA.doIKhipRollModify("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot,0);
                    //                        SURENA.doIKhipRollModify("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot,1*hipRoll);
                    //                    }



                    SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot);
                    SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot);
                    //R1=AnkleRollRight
                    //R2=AnklePitchRight
                    //R3=KneePitchRight
                    //R4=HipPitchRight
                    //R5=HipRollRight
                    //R6=HipYawRight

                    //L1=AnkleRollLeft
                    //L2=AnklePitchLeft
                    //L3=KneePitchLeft
                    //L4=HipPitchLeft
                    //L5=HipRollLeft
                    //L6=HipYawLeft

                    //R1=0*(1/2Pi)*(2304)*100  R2=1*-1*(1/2Pi)*(2304)*100   R3=2(1/2Pi)*(2304)*50   R4=3*-1(1/2Pi)*(2304)*80    L2=4*(1/2Pi)*(2304)*100    L1=5*(1/2Pi)*(2304)*100   L3=6*-1*(1/2Pi)*(2304)*50     L4=7*(1/2Pi)*(2304)*80   R6=8*-1*(1/2Pi)*(2304)*120   R5=9*(1/2Pi)*(2304)*120    L5=10*(1/2Pi)*(2304)*120    L6=11*-1*(1/2Pi)*(2304)*120

                }

            }
        }

        if (endPhase==true &&  StartTime>=(DurationOfStartPhase+SURENAOffilneTaskSpace.MotionTime) && StartTime<=DurationOfendPhase+DurationOfStartPhase+SURENAOffilneTaskSpace.MotionTime) {
            MinimumJerkInterpolation Coef;
            MatrixXd ZPosition(1,2);
            ZPosition<<0.830,0.95100;
            MatrixXd ZVelocity(1,2);
            ZVelocity<<0.000,0.000;
            MatrixXd ZAcceleration(1,2);
            ZAcceleration<<0.000,0.000;


            MatrixXd Time(1,2);
            Time<<DurationOfStartPhase+SURENAOffilneTaskSpace.MotionTime,DurationOfStartPhase+SURENAOffilneTaskSpace.MotionTime+DurationOfendPhase;
            MatrixXd CoefZStart =Coef.Coefficient(Time,ZPosition,ZVelocity,ZAcceleration);

            double zStart=0;
            double yStart=0;
            double xStart=0;
            StartTime=StartTime+SURENAOffilneTaskSpace._timeStep;

            MatrixXd outputZStart= SURENAOffilneTaskSpace.GetAccVelPos(CoefZStart,StartTime,DurationOfStartPhase+SURENAOffilneTaskSpace.MotionTime,5);
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
        }
        links = SURENA.GetLinks();

        MatrixXi mappingJoints(12,1);
        //R6=0*(1/2Pi)*(2304)*100  R5=1*-1*(1/2Pi)*(2304)*100   R4=2*(1/2Pi)*(2304)*50   R3=3*-1(1/2Pi)*(2304)*80  R2=9*(1/2Pi)*(2304)*120  R1=8*-1*(1/2Pi)*(2304)*120
        //  L6=5*(1/2Pi)*(2304)*100   L5=4*(1/2Pi)*(2304)*100   L4=6*-1*(1/2Pi)*(2304)*50     L3=7*(1/2Pi)*(2304)*80  L2=10*(1/2Pi)*(2304)*120   L1=11*-1*(1/2Pi)*(2304)*120

        teta_motor_L=teta_motor_L+teta_pid_L.Calculate(0,teta_L);
        phi_motor_L=phi_motor_L+phi_pid_L.Calculate(0,phi_L);


        teta_motor_R=teta_motor_R+teta_pid_R.Calculate(0,teta_R);
        phi_motor_R=phi_motor_R+phi_pid_R.Calculate(0,phi_R);

        teta_motor_center=teta_motor_center+teta_pid_center.Calculate(0,teta_center);
        phi_motor_center=phi_motor_center+phi_pid_center.Calculate(0,phi_center);


        double maximum_d=.15;

        if (teta_motor_L<-maximum_d){teta_motor_L=-maximum_d;}
        if (teta_motor_L>maximum_d){teta_motor_L=maximum_d;}
        if (phi_motor_L<-maximum_d){phi_motor_L=-maximum_d;}
        if (phi_motor_L>maximum_d){phi_motor_L=maximum_d;}

        if (teta_motor_R<-maximum_d){teta_motor_R=-maximum_d;}
        if (teta_motor_R>maximum_d){teta_motor_R=maximum_d;}
        if (phi_motor_R<-maximum_d){phi_motor_R=-maximum_d;}
        if (phi_motor_R>maximum_d){phi_motor_R=maximum_d;}




        //mappingJoints<<links[6].JointAngle*(1/(2*M_PI))*(2304)*100+(phi_motor_R*2304*100/2/3.141592),//(phi_pid_R.Calculate(0,phi_R)*2304*100/2/3.141592),
        //    links[5].JointAngle*(-1)*(1/(2*M_PI))*(2304)*100+(-teta_motor_R*2304*100/2/3.141592),//(-teta_pid_R.Calculate(0,teta_R)*2304*100/2/3.141592),
        //    links[4].JointAngle*(1/(2*M_PI))*(2304)*50,
        //    links[3].JointAngle*-1*(1/(2*M_PI))*(2304)*80,
        //    links[11].JointAngle*(1/(2*M_PI))*(2304)*100+(teta_motor_L*2304*100/2/3.141592),//(teta_pid_L.Calculate(0,teta_L)*2304*100/2/3.141592),
        //    links[12].JointAngle*(1/(2*M_PI))*(2304)*100+(phi_motor_L*2304*100/2/3.141592),//(phi_pid_L.Calculate(0,phi_L)*2304*100/2/3.141592),
        //    links[10].JointAngle*-1*(1/(2*M_PI))*(2304)*50,
        //    links[9].JointAngle*(1/(2*M_PI))*(2304)*80,
        //    links[1].JointAngle*(-1)*(1/(2*M_PI))*(2304)*120,
        //    links[2].JointAngle*(1/(2*M_PI))*(2304)*120,
        //    links[8].JointAngle*(1/(2*M_PI))*(2304)*120,
        //    links[7].JointAngle*-1*(1/(2*M_PI))*(2304)*120;

        vector<double> cntrl(13);

        if (SURENAOffilneTaskSpace.DoubleSupport) {

            cntrl[0]=0.0;
            cntrl[1]=0;
            cntrl[2]=0;
            cntrl[3]=0;
            cntrl[4]=0;
            cntrl[5]=0;
            cntrl[6]=0;
            cntrl[7]=0;
            cntrl[8]=0;
            cntrl[9]=0;
            cntrl[10]=0;
            cntrl[11]=0;
            cntrl[12]=0;
            teta_motor_L=0;
            teta_motor_R=0;
            phi_motor_L=0;
            phi_motor_R=0;
            teta_motor_center=0;
            phi_motor_center=0;

           // ROS_INFO("t={%d} : double_support",timer.elapsed());
        }
        else{
            if (SURENAOffilneTaskSpace.LeftSupport) {
                cntrl[0]=0.0;
                cntrl[1]=0;
                cntrl[2]=0;
                cntrl[3]=0;
                cntrl[4]=0;
                cntrl[5]=1*teta_motor_R;
                cntrl[6]=1*phi_motor_R;
                cntrl[7]=0;
                cntrl[8]=0*phi_motor_center;
                cntrl[9]=0;
                cntrl[10]=0;
                cntrl[11]=0;
                cntrl[12]=0;
                teta_motor_L=0;
                phi_motor_L=0;
                teta_motor_center=0;


               // ROS_INFO("t={%d} : left_support",timer.elapsed());
            }
            else{
                cntrl[0]=0;
                cntrl[1]=0;
                cntrl[2]=-0*phi_motor_center;
                cntrl[3]=0;
                cntrl[4]=0;
                cntrl[5]=0;
                cntrl[6]=0;
                cntrl[5]=0;
                cntrl[6]=0;
                cntrl[7]=0;
                cntrl[8]=0;
                cntrl[9]=0;
                cntrl[10]=0;
                cntrl[11]=1*teta_motor_L;
                cntrl[12]=1*phi_motor_L;
                teta_motor_R=0;
                phi_motor_R=0;
                teta_motor_center=0;
              //  ROS_INFO("t={%d} : right_support",timer.elapsed());
            }
        }

        QCgenerator QC;
        qref=QC.data2qc(links,cntrl);
        msg.data.clear();
        for(int  i = 1;i < 13;i++)
        {
            // qref[i-1] = mappingJoints(i-1,0);
            //  cout << qref[i-1] <<" , "<<flush;
            msg.data.push_back(qref[i-1]);

        }
        // std::string varAsString = std::to_string(qref[i-1]);
        // msg.data =varAsString;
        SendGazebo_with_control(links,cntrl);
        chatter_pub.publish(msg);
         ROS_INFO("t={%d} c={%d}",timer.elapsed(),count);

        //ROS_INFO("teta_r={%f} phi_r={%f} teta_l={%f} phi_l={%f}",teta_motor_R,phi_motor_R,teta_motor_L,phi_motor_L);
//        ROS_INFO("teta_l_IMU={%f} phi_l_IMU={%f} teta_l={%f} phi_l={%f}",teta_L,phi_L,teta_motor_L,phi_motor_L);
 //ROS_INFO("teta_C_IMU={%f} phi_C_IMU={%f} ",teta_center,phi_center);
        //
        //    std_msgs::String msg;
        // msg.data = "milad";

        //R1=AnkleRollRight
        //R2=AnklePitchRight
        //R3=KneePitchRight
        //R4=HipPitchRight
        //R5=HipRollRight
        //R6=HipYawRight

        //L1=AnkleRollLeft
        //L2=AnklePitchLeft
        //L3=KneePitchLeft
        //L4=HipPitchLeft
        //L5=HipRollLeft
        //L6=HipYawLeft

        //R1=0*(1/2Pi)*(2304)*100  R2=1*-1*(1/2Pi)*(2304)*100   R3=2(1/2Pi)*(2304)*50   R4=3*-1(1/2Pi)*(2304)*80   R5=9*(1/2Pi)*(2304)*120  R6=8*-1*(1/2Pi)*(2304)*120      L1=5*(1/2Pi)*(2304)*100  L2=4*(1/2Pi)*(2304)*100  L3=6*-1*(1/2Pi)*(2304)*50    L4=7*(1/2Pi)*(2304)*80    L5=10*(1/2Pi)*(2304)*120    L6=11*-1*(1/2Pi)*(2304)*120


        //R6=AnkleRollRight
        //R5=AnklePitchRight
        //R4=KneePitchRight
        //R3=HipPitchRight
        //R2=HipRollRight
        //R1=HipYawRight

        //L6=AnkleRollLeft
        //L5=AnklePitchLeft
        //L4=KneePitchLeft
        //L3=HipPitchLeft
        //L2=HipRollLeft
        //L1=HipYawLeft

        //R6=0*(1/2Pi)*(2304)*100  R5=1*-1*(1/2Pi)*(2304)*100   R4=2(1/2Pi)*(2304)*50   R3=3*-1(1/2Pi)*(2304)*80  R2=9*(1/2Pi)*(2304)*120  R1=8*-1*(1/2Pi)*(2304)*120    L6=5*(1/2Pi)*(2304)*100   L5=4*(1/2Pi)*(2304)*100   L4=6*-1*(1/2Pi)*(2304)*50     L3=7*(1/2Pi)*(2304)*80  L2=10*(1/2Pi)*(2304)*120   L1=11*-1*(1/2Pi)*(2304)*120

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}

