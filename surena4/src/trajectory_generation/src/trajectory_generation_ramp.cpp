#include "ros/ros.h"
#include "std_msgs/String.h"
#include"Eigen/Dense"
#include <vector>
#include <iostream>
#include <QString>
#include <QList>
#include "Robot.h"
//#include"TaskSpace.h"
#include"taskspaceofflineRamp.h"
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

//ros::Publisher pid1 ;





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
    vector<double> cntrl(13);
    QCgenerator QC;
    //check _timesteps
    QElapsedTimer timer;
    Robot SURENA;
    TaskSpaceOfflineRamp SURENAOffilneTaskSpaceRamp;
    QList<LinkM> links;
    MatrixXd PoseRoot;
    MatrixXd PoseRFoot;
    MatrixXd PoseLFoot;
    double dt;

    double k1;
    double k2;
    double k3;
    double k4;

    bool aState=false;
    bool bState=false;
    bool cState=false;
    bool dState=false;
    bool LeftFootLanded=false;

    double teta_motor_L=0;
    double teta_motor_R=0;
    double phi_motor_L=0;
    double phi_motor_R=0;

    int numberOfLeftFootSensorData=0;
    double footSensorSaturation=90;//if all sensors data are bigger than this amount, this means the foot is landed on the ground
    double footSensorthreshold=4;
    double hipRoll=0;
    double StartTime=0;
    double WalkTime=0;
    double RollTime=0;
    double  DurationOfStartPhase=6;
    double  DurationOfendPhase=6;
    //SURENAOffilneTaskSpaceRamp.GetAccVelPos();
    bool startPhase=true;
    bool endPhase=true;
    bool walk=true;
    PoseRoot.resize(6,1);
    PoseRFoot.resize(6,1);
    PoseLFoot.resize(6,1);

    //*******************This part of code is for initialization of joints of the robot for walking**********************************
    int count = 0;

    ros::init(argc, argv, "myNode");

    ros::NodeHandle nh;
    ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",1000);

    //ros::Subscriber sub = nh.subscribe("/foot", 1000, receiveFootSensor);

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





    ros::Rate loop_rate(100);
    std_msgs::Int32MultiArray msg;
    std_msgs::MultiArrayDimension msg_dim;

    msg_dim.label = "joint_position";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);
    k1=0.00009;
    k2=0.00009;
    k3=0.00001;
    k4=0.00001;



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


            double zStart=0;
            double yStart=0;
            double xStart=0;
            StartTime=StartTime+SURENAOffilneTaskSpaceRamp._timeStep;

            MatrixXd outputZStart= SURENAOffilneTaskSpaceRamp.GetAccVelPos(CoefZStart,StartTime,0,5);
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



        if (StartTime>DurationOfStartPhase && StartTime<(DurationOfStartPhase+SURENAOffilneTaskSpaceRamp.MotionTime)){

            bool walk=true;
            double m1;
            double m2;
            double m3;
            double m4;
            double m5;
            double m6;
            double m7;
            double m8;
            StartTime=StartTime+SURENAOffilneTaskSpaceRamp._timeStep;
            //qDebug()<<StartTime;
            MatrixXd P;
            if(walk==true){
                MatrixXd m=SURENAOffilneTaskSpaceRamp.AnkleTrajectory(SURENAOffilneTaskSpaceRamp.globalTime);
                m1=m(0,0);
                m2=m(1,0);
                m3=m(2,0);
                m4=m(3,0);
                m5=m(4,0);
                m6=m(5,0);
                m7=m(6,0);
                m8=m(7,0);

                P=SURENAOffilneTaskSpaceRamp.PelvisTrajectory (SURENAOffilneTaskSpaceRamp.globalTime);

                // just some samples for plotting!!!
                //                SURENAOffilneTaskSpaceRamp.CoMXVector.append(P(0,0));
                //                SURENAOffilneTaskSpaceRamp.timeVector.append(SURENAOffilneTaskSpaceRamp.globalTime);
                //                SURENAOffilneTaskSpaceRamp.LeftFootXTrajectory.append(m1);



                SURENAOffilneTaskSpaceRamp.globalTime=SURENAOffilneTaskSpaceRamp.globalTime+SURENAOffilneTaskSpaceRamp._timeStep;

                if (round(SURENAOffilneTaskSpaceRamp.globalTime)<=round(SURENAOffilneTaskSpaceRamp.MotionTime)){



                    //cout<<SURENAOffilneTaskSpaceRamp.TSS<<endl;
                    //cout<<RollTime<<endl;

                    PoseRoot<<P(0,0),
                            P(1,0),
                            P(2,0),
                            0,
                            0,
                            0;

                    PoseRFoot<<m5,
                            m6,
                            m7,
                            0,
                            1*m8,
                            0;

                    PoseLFoot<<m1,
                            m2,
                            m3,
                            0,
                            1*m4,
                            0;





                    SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot);
                    SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot);

                }

            }
        }

        if (endPhase==true &&  StartTime>=(DurationOfStartPhase+SURENAOffilneTaskSpaceRamp.MotionTime) && StartTime<=DurationOfendPhase+DurationOfStartPhase+SURENAOffilneTaskSpaceRamp.MotionTime) {
            MinimumJerkInterpolation Coef;
            MatrixXd ZPosition(1,2);
            ZPosition<<0.830,0.95100;
            MatrixXd ZVelocity(1,2);
            ZVelocity<<0.000,0.000;
            MatrixXd ZAcceleration(1,2);
            ZAcceleration<<0.000,0.000;


            MatrixXd Time(1,2);
            Time<<DurationOfStartPhase+SURENAOffilneTaskSpaceRamp.MotionTime,DurationOfStartPhase+SURENAOffilneTaskSpaceRamp.MotionTime+DurationOfendPhase;
            MatrixXd CoefZStart =Coef.Coefficient(Time,ZPosition,ZVelocity,ZAcceleration);

            double zStart=0;
            double yStart=0;
            double xStart=0;
            StartTime=StartTime+SURENAOffilneTaskSpaceRamp._timeStep;

            MatrixXd outputZStart= SURENAOffilneTaskSpaceRamp.GetAccVelPos(CoefZStart,StartTime,DurationOfStartPhase+SURENAOffilneTaskSpaceRamp.MotionTime,5);
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

        //        ROS_INFO("I heard motor pitch: [%f]",teta_motor_L);
        //        ROS_INFO("I heard motor roll: [%f]",phi_motor_L);
        msg.data.clear();




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






        vector<int> qref(12);
        qref=QC.ctrldata2qc(cntrl);


        for(int  i = 0;i < 12;i++)
        {
            msg.data.push_back(qref[i]);

        }
        // std::string varAsString = std::to_string(qref[i-1]);
        // msg.data =varAsString;
        SendGazebo(links);
        // SendGazeboPID();
        chatter_pub.publish(msg);
        //  ROS_INFO("t={%d} c={%d}",timer.elapsed(),count);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}

