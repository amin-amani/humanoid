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


#include<sensor_msgs/Imu.h>
#include<std_msgs/Float64.h>
#include "pidcontroller.h"
#include<rosgraph_msgs/Clock.h>

#include "std_msgs/String.h"
#include<gazebo_msgs/ApplyBodyWrench.h>

#include<gazebo_msgs/ApplyBodyWrenchRequest.h>
#include <gazebo_msgs/GetLinkState.h>
#include<gazebo_msgs/LinkStates.h>
#include <nav_msgs/Odometry.h>
#include "std_srvs/Empty.h"
#include "qcgenerator.h"
#include "/home/cast/humanoid/surena4/devel/include/xsens_msgs/Internal.h"
#include "/home/cast/humanoid/surena4/devel/include/xsens_msgs/velocityEstimate.h"
#include "/home/cast/humanoid/surena4/devel/include/xsens_msgs/sensorSample.h"
#include "/home/cast/humanoid/surena4/devel/include/xsens_msgs/ImuSensorSample.h"
#include "/home/cast/humanoid/surena4/devel/include/xsens_msgs/orientationEstimate.h"
#include "/home/cast/humanoid/surena4/devel/include/xsens_msgs/positionEstimate.h"
#include "/home/cast/humanoid/surena4/devel/include/xsens_msgs/velocityEstimate.h"
#include "/home/cast/humanoid/surena4/devel/include/xsens_msgs/XsensQuaternion.h"

using namespace  std;
using namespace  Eigen;
//data of left foot sensor

double n_f;
double w_y;
double w_y_f;
double pitch;
double pitch_bias;


vector<double> w_y_s(100);
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

//ros::Publisher pid1 ;

PIDController push_recovery_knee;
PIDController push_recovery_hip;
PIDController push_recovery_ankle;


//void  SendGazebo(QList<LinkM> links){
//    if(links.count()<28){qDebug()<<"index err";return;}
//    std_msgs::Float64 data;
//    data.data=links[1].JointAngle;
//    pub1.publish(data);
//    data.data=links[2].JointAngle;
//    pub2.publish(data);
//    data.data=links[3].JointAngle;
//    pub3.publish(data);
//    data.data=links[4].JointAngle;
//    pub4.publish(data);
//    data.data=links[5].JointAngle;
//    pub5.publish(data);
//    data.data=links[6].JointAngle;
//    pub6.publish(data);
//    data.data=links[7].JointAngle;
//    pub7.publish(data);
//    data.data=links[8].JointAngle;
//    pub8.publish(data);
//    data.data=links[9].JointAngle;
//    pub9.publish(data);
//    data.data=links[10].JointAngle;
//    pub10.publish(data);
//    data.data=links[11].JointAngle;
//    pub11.publish(data);
//    data.data=links[12].JointAngle;
//    pub12.publish(data);
//    data.data=links[13].JointAngle;
//    pub13.publish(data);
//    data.data=links[14].JointAngle;
//    pub14.publish(data);
//    data.data=links[15].JointAngle;
//    pub15.publish(data);
//    data.data=links[16].JointAngle;
//    pub16.publish(data);
//    data.data=links[17].JointAngle;
//    pub17.publish(data);
//    data.data=links[18].JointAngle;
//    pub18.publish(data);
//    data.data=links[19].JointAngle;
//    pub19.publish(data);
//    data.data=links[20].JointAngle;
//    pub20.publish(data);
//    data.data=links[21].JointAngle;
//    pub21.publish(data);
//    data.data=links[22].JointAngle;
//    pub22.publish(data);
//    data.data=links[23].JointAngle;
//    pub23.publish(data);
//    data.data=links[24].JointAngle;
//    pub24.publish(data);
//    data.data=links[25].JointAngle;
//    pub25.publish(data);
//    data.data=links[26].JointAngle;
//    pub26.publish(data);
//    data.data=links[27].JointAngle;
//    pub27.publish(data);
//    data.data=links[28].JointAngle;
//    pub28.publish(data);
//}




void  SendGazebo(vector<double> ctrl){

    std_msgs::Float64 data;
    data.data=ctrl[1];
    pub1.publish(data);
    data.data=ctrl[2];
    pub2.publish(data);
    data.data=ctrl[3];
    pub3.publish(data);
    data.data=ctrl[4];
    pub4.publish(data);
    data.data=ctrl[5];
    pub5.publish(data);
    data.data=ctrl[6];
    pub6.publish(data);
    data.data=ctrl[7];
    pub7.publish(data);
    data.data=ctrl[8];
    pub8.publish(data);
    data.data=ctrl[9];
    pub9.publish(data);
    data.data=ctrl[10];
    pub10.publish(data);
    data.data=ctrl[11];
    pub11.publish(data);
    data.data=ctrl[12];
    pub12.publish(data);
    data.data=0;
    pub13.publish(data);
    data.data=0;
    pub14.publish(data);
    data.data=0;
    pub15.publish(data);
    data.data=0-10*M_PI/180;
    pub16.publish(data);
    data.data=0;
    pub17.publish(data);
    data.data=0;
    pub18.publish(data);
    data.data=0;
    pub19.publish(data);
    data.data=0;
    pub20.publish(data);
    data.data=0;
    pub21.publish(data);
    data.data=0;
    pub22.publish(data);
    data.data=0+10*M_PI/180;
    pub23.publish(data);
    data.data=0;
    pub24.publish(data);
    data.data=0;
    pub25.publish(data);
    data.data=0;
    pub26.publish(data);
    data.data=0;
    pub27.publish(data);
    data.data=0;
    pub28.publish(data);



}


void RecievIMU_w(const sensor_msgs::Imu & msg)
{

    for (int var = n_f-1; var > 0; --var) {
        w_y_s[var]=w_y_s[var-1];
    }
    w_y= msg.angular_velocity.y;
    w_y_s[0]=w_y;
    double sum=0;
    for (int var = 0; var < n_f; ++var) {
        sum=sum+w_y_s[var];
    }
    w_y_f=sum/n_f;
}

void RecievIMU_pitch(const xsens_msgs::orientationEstimate & msg)
{
    pitch=msg.pitch;
}



int main(int argc, char **argv)
{
    n_f=5;
    for (int var = 0; var < n_f; ++var) {
        w_y_s[var]=0;
    }
    vector<double> cntrl(13);
    vector<double> push_state(13);
    QCgenerator QC;
    //check _timesteps
    QElapsedTimer timer;
    Robot SURENA;
    TaskSpaceOffline SURENAOffilneTaskSpace;
    QList<LinkM> links;
    MatrixXd PoseRoot;
    MatrixXd PoseRFoot;
    MatrixXd PoseLFoot;
    double dt;

    double k_ankle=0;
    double k_knee=-.3;
    double k_hip=0;





    double StartTime=0;

    double  DurationOfStartPhase=6;
    double DurationOfPitchBias=.1;
    double DurationOfPushPhase=30;
    double  DurationOfBacktoKnee=2;
    double  DurationOfendPhase=6;
    //SURENAOffilneTaskSpace.GetAccVelPos();
    bool startPhase=true;

    PoseRoot.resize(6,1);
    PoseRFoot.resize(6,1);
    PoseLFoot.resize(6,1);

    //*******************This part of code is for initialization of joints of the robot for walking**********************************
    int count = 0;

    ros::init(argc, argv, "myNode");

    ros::NodeHandle nh;
    ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",1000);


    ros::Subscriber  IMU_xsense = nh.subscribe("/mti/sensor/imu", 100, RecievIMU_w);
   // ros::Subscriber  IMU_xsense = nh.subscribe("/mti/filter/orientation", 100, RecievIMU_pitch);


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

    //pid1= nh.advertise<std_msgs::Float64>("rrbot/joint28_position_controller/pid/parameter_updates",1000);
    double p_hip ,i_hip ,d_hip;
    double p_knee ,i_knee, d_knee;
    double p_ankle ,i_ankle ,d_ankle;



    ros::Rate loop_rate(100);
    std_msgs::Int32MultiArray msg;
    std_msgs::MultiArrayDimension msg_dim;

    msg_dim.label = "joint_position";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);
    pitch_bias=0;

    bool b;
    int bias_count=0;
    bool pitch_bias_ok;
    while (ros::ok())
    {
        // qDebug()<<StartTime;




        //start phase
        if (startPhase==true && StartTime<=DurationOfStartPhase) {
            MinimumJerkInterpolation Coef;
            MatrixXd ZPosition(1,2);
            ZPosition<<0.95100,0.9000;
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
            links = SURENA.GetLinks();




            cntrl[0]=0.0;
            cntrl[1]=links[1].JointAngle;
            cntrl[2]=links[2].JointAngle;
            cntrl[3]=links[3].JointAngle;
            cntrl[4]=links[4].JointAngle;
            cntrl[5]=links[5].JointAngle;//+teta_motor_R;//pitch
            cntrl[6]=links[6].JointAngle;//roll
            cntrl[7]=links[7].JointAngle;
            cntrl[8]=links[8].JointAngle;
            cntrl[9]=links[9].JointAngle;
            cntrl[10]=links[10].JointAngle;
            cntrl[11]=links[11].JointAngle;
            cntrl[12]=links[12].JointAngle;




            b=true;
            ROS_INFO("wait!");
            // ROS_INFO("%f\t%f\t%f\t%f\t%f\t%f\t",cntrl[3],cntrl[4],cntrl[5],cntrl[9],cntrl[10],cntrl[11]);
            //tested for orientation
//            p_hip=.007;//.007;//.01;//.007;//.005;//.01;(jumping)//.02;(jumping)//008;
//            d_hip=0;//.00000000001;//.02;//.006;//.002;//.001;//0;//0.00006;

            //tested for angular velocity
            p_hip=.02;
            d_hip=0;
            i_hip=.0;



            push_recovery_hip.Init(.01,0.3,-.3,p_hip,d_hip,i_hip);


//            p_knee=.007;//.007;//.007;//.01;//.007;//.005;//.01;(jumping)//.03;(jumping)//.016;
//            d_knee=0;//.00000000001;//.04;//.02;//.01;//.002;//.001;//0;//0.00016;
            p_knee=.04;//.02;
            d_knee=0;
            i_knee=.0;

            push_recovery_knee.Init(.01,0.3,-.3,p_knee,d_knee,i_knee);


            p_ankle=.001;//.005;//.01;(jumping)//.02;(jumping)//.004;
            i_ankle=0.0;
            d_ankle=0;//.002;//.001;//0;//0.00003;
            push_recovery_ankle.Init(.01,0.3,-.3,p_ankle,d_ankle,i_ankle);



        }
        //bias phase
        if( StartTime>DurationOfStartPhase && StartTime<=DurationOfStartPhase+DurationOfPitchBias){
            StartTime=StartTime+SURENAOffilneTaskSpace._timeStep;
            pitch_bias+=pitch;
            bias_count++;
            pitch_bias_ok=true;
        }
        //push phase
        if( StartTime>DurationOfStartPhase+DurationOfPitchBias && StartTime<DurationOfStartPhase+DurationOfPitchBias+DurationOfPushPhase) {
            StartTime=StartTime+SURENAOffilneTaskSpace._timeStep;
            if(pitch_bias_ok){
                pitch_bias/=double(bias_count);
                pitch_bias_ok=false;
            }
            //ROS_INFO("push!\tpitch_bias=%f\tpitch=%f\tpitch-pitch_bias=%f",pitch_bias,pitch,pitch-pitch_bias);
            ROS_INFO("push!\twy=%f",w_y);


//            double p_r_ankle=push_recovery_ankle.Calculate(0,pitch-pitch_bias);
//            double p_r_knee=push_recovery_knee.Calculate(0,pitch-pitch_bias);
//            double p_r_hip=push_recovery_hip.Calculate(0,pitch-pitch_bias);
            double p_r_ankle=push_recovery_ankle.Calculate(0,w_y_f);
            double p_r_knee=push_recovery_knee.Calculate(0,w_y_f);
            double p_r_hip=push_recovery_hip.Calculate(0,w_y_f);
            cntrl[0]=0.0;
            cntrl[1]=links[1].JointAngle;
            cntrl[2]=links[2].JointAngle;
            cntrl[3]=links[3].JointAngle+p_r_hip;//+k_hip*w_y_f;
            cntrl[4]=links[4].JointAngle+p_r_knee;//;//-push_recovery_knee.Calculate(0,w_y_f);//k_knee*w_y_f;
            cntrl[5]=links[5].JointAngle+p_r_ankle;//+k_ankle*w_y_f;
            cntrl[6]=links[6].JointAngle;
            cntrl[7]=links[7].JointAngle;
            cntrl[8]=links[8].JointAngle;
            cntrl[9]=links[9].JointAngle+p_r_hip;//+k_hip*w_y_f;
            cntrl[10]=links[10].JointAngle+p_r_knee;//-push_recovery_knee.Calculate(0,w_y_f);//k_knee*w_y_f;
            cntrl[11]=links[11].JointAngle+p_r_ankle;//+k_ankle*w_y_f;
            cntrl[12]=links[12].JointAngle;
            //            ROS_INFO("pitch = %f",pitch);
            ROS_INFO("w_y=%f\tw_y_filtered=%f\t",w_y,w_y_f);
            ROS_INFO("hip=%f\tkee=%f\tankle=%f\t",p_r_hip,p_r_knee,p_r_ankle);

            ROS_INFO("%f\t%f\t%f\t%f\t%f\t%f\t",cntrl[3],cntrl[4],cntrl[5],cntrl[9],cntrl[10],cntrl[11]);
        }
        //back to knee phase
        if (StartTime>=(DurationOfStartPhase+DurationOfPitchBias+DurationOfPushPhase) && StartTime<=DurationOfBacktoKnee+DurationOfStartPhase+DurationOfPushPhase+DurationOfPitchBias) {
            StartTime=StartTime+SURENAOffilneTaskSpace._timeStep;
            ROS_INFO("STOP pushing!");
            cntrl[0]=0.0;
            cntrl[1]=links[1].JointAngle;
            cntrl[2]=links[2].JointAngle;
            cntrl[3]=links[3].JointAngle;
            cntrl[4]=links[4].JointAngle;
            cntrl[5]=links[5].JointAngle;
            cntrl[6]=links[6].JointAngle;
            cntrl[7]=links[7].JointAngle;
            cntrl[8]=links[8].JointAngle;
            cntrl[9]=links[9].JointAngle;
            cntrl[10]=links[10].JointAngle;
            cntrl[11]=links[11].JointAngle;
            cntrl[12]=links[12].JointAngle;

            ROS_INFO("%f\t%f\t%f\t%f\t%f\t%f\t",cntrl[3],cntrl[4],cntrl[5],cntrl[9],cntrl[10],cntrl[11]);


        }
        //stand up phase
        if (StartTime>=(DurationOfStartPhase+DurationOfPitchBias+DurationOfPushPhase+DurationOfBacktoKnee) && StartTime<=DurationOfBacktoKnee+DurationOfendPhase+DurationOfStartPhase+DurationOfPushPhase+DurationOfPitchBias) {
            MinimumJerkInterpolation Coef;
            MatrixXd ZPosition(1,2);
            ZPosition<<0.90,0.95100;
            MatrixXd ZVelocity(1,2);
            ZVelocity<<0.000,0.000;
            MatrixXd ZAcceleration(1,2);
            ZAcceleration<<0.000,0.000;


            MatrixXd Time(1,2);
            Time<<DurationOfStartPhase+DurationOfPitchBias+DurationOfPushPhase+DurationOfBacktoKnee,DurationOfStartPhase+DurationOfPitchBias+DurationOfPushPhase+DurationOfBacktoKnee+DurationOfendPhase;
            MatrixXd CoefZStart =Coef.Coefficient(Time,ZPosition,ZVelocity,ZAcceleration);

            double zStart=0;
            double yStart=0;
            double xStart=0;
            StartTime=StartTime+SURENAOffilneTaskSpace._timeStep;

            MatrixXd outputZStart= SURENAOffilneTaskSpace.GetAccVelPos(CoefZStart,StartTime,DurationOfStartPhase+DurationOfPitchBias+DurationOfPushPhase+DurationOfBacktoKnee,5);
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


            links = SURENA.GetLinks();


            cntrl[0]=0.0;
            cntrl[1]=links[1].JointAngle;
            cntrl[2]=links[2].JointAngle;
            cntrl[3]=links[3].JointAngle;
            cntrl[4]=links[4].JointAngle;
            cntrl[5]=links[5].JointAngle;//+teta_motor_R;//pitch
            cntrl[6]=links[6].JointAngle;//roll
            cntrl[7]=links[7].JointAngle;
            cntrl[8]=links[8].JointAngle;
            cntrl[9]=links[9].JointAngle;
            cntrl[10]=links[10].JointAngle;
            cntrl[11]=links[11].JointAngle;
            cntrl[12]=links[12].JointAngle;
        }



        vector<int> qref(12);
        qref=QC.ctrldata2qc(cntrl);
        msg.data.clear();

        for(int  i = 0;i < 12;i++)
        {
            msg.data.push_back(qref[i]);
        }
        ROS_INFO("%f\t%f\t%f\t%f\t%f\t%f\t",cntrl[3],cntrl[4],cntrl[5],cntrl[9],cntrl[10],cntrl[11]);



        // std::string varAsString = std::to_string(qref[i-1]);
        // msg.data =varAsString;
        SendGazebo(cntrl);
        // SendGazeboPID();
        chatter_pub.publish(msg);
        //  ROS_INFO("t={%d} c={%d}",timer.elapsed(),count);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;

    }
    return 0;
}

