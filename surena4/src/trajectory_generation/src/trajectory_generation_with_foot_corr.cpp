#include "ros/ros.h"
#include "std_msgs/String.h"
#include"Eigen/Dense"
#include <vector>
#include <iostream>
#include <QString>
#include <QList>
#include "Robot.h"
#include"TaskSpace.h"
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

using namespace  std;
using namespace  Eigen;

//sensor imu----------------------------------------------sensor imu
//sensor imu----------------------------------------------sensor imu
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

ros::Subscriber orintation;

void RecievIMULeft(const sensor_msgs::Imu & msg)
{
  ROS_INFO("Left:[%f] [%f] [%f] [%f]",  msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w);
 //  ROS_INFO("I heard");
}

void RecievIMURight(const sensor_msgs::Imu & msg)
{
  ROS_INFO("Right:[%f] [%f] [%f] [%f]",  msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w);
 //  ROS_INFO("I heard");
}

void RecievIMUCenter(const sensor_msgs::Imu & msg)
{
  ROS_INFO("Center:[%f] [%f] [%f] [%f]", msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w);
 //  ROS_INFO("I heard");
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
int main(int argc, char **argv)
{
  //check _timesteps
    QElapsedTimer timer;
  vector<double> qref(12);
  Robot SURENA;
  TaskSpaceOffline SURENAOffilneTaskSpace;
  QList<LinkM> links;
  MatrixXd PoseRoot;
  MatrixXd PoseRFoot;
  MatrixXd PoseLFoot;
  double dt;

  double StartTime=0;
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
ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",1000);

ros::Subscriber  IMULeft = nh.subscribe("/yei2000154", 100, RecievIMULeft);
ros::Subscriber  IMURight = nh.subscribe("/yei200015B", 100, RecievIMURight);
ros::Subscriber  IMUCenter = nh.subscribe("/mti/sensor/imu", 100, RecievIMUCenter);

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


  ros::Rate loop_rate(100);
  std_msgs::Int32MultiArray msg;
  std_msgs::MultiArrayDimension msg_dim;

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
    msg.data.clear();
    MatrixXi mappingJoints(12,1);
    //R6=0*(1/2Pi)*(2304)*100  R5=1*-1*(1/2Pi)*(2304)*100   R4=2*(1/2Pi)*(2304)*50   R3=3*-1(1/2Pi)*(2304)*80  R2=9*(1/2Pi)*(2304)*120  R1=8*-1*(1/2Pi)*(2304)*120
    //  L6=5*(1/2Pi)*(2304)*100   L5=4*(1/2Pi)*(2304)*100   L4=6*-1*(1/2Pi)*(2304)*50     L3=7*(1/2Pi)*(2304)*80  L2=10*(1/2Pi)*(2304)*120   L1=11*-1*(1/2Pi)*(2304)*120
mappingJoints<<links[6].JointAngle*(1/(2*M_PI))*(2304)*100,
    links[5].JointAngle*(-1)*(1/(2*M_PI))*(2304)*100,
    links[4].JointAngle*(1/(2*M_PI))*(2304)*50,
    links[3].JointAngle*-1*(1/(2*M_PI))*(2304)*80,
    links[11].JointAngle*(1/(2*M_PI))*(2304)*100,
    links[12].JointAngle*(1/(2*M_PI))*(2304)*100,
    links[10].JointAngle*-1*(1/(2*M_PI))*(2304)*50,
    links[9].JointAngle*(1/(2*M_PI))*(2304)*80,
    links[1].JointAngle*(-1)*(1/(2*M_PI))*(2304)*120,
    links[2].JointAngle*(1/(2*M_PI))*(2304)*120,
    links[8].JointAngle*(1/(2*M_PI))*(2304)*120,
    links[7].JointAngle*-1*(1/(2*M_PI))*(2304)*120;

    for(int  i = 1;i < 13;i++)
    {
      qref[i-1] = mappingJoints(i-1,0);
      //  cout << qref[i-1] <<" , "<<flush;
      msg.data.push_back(qref[i-1]);

    }
    // std::string varAsString = std::to_string(qref[i-1]);
    // msg.data =varAsString;
    //SendGazebo(links);
 // chatter_pub.publish(msg);
   // ROS_INFO("t1={%d} c={%d}",timer.elapsed(),count);
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

