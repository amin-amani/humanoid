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
#include <gazebo_msgs/LinkStates.h>
#include "QTime"

using namespace  std;
using namespace  Eigen;
//data of left foot sensor
int a;
int b;
int c;
int d;
int e;
int f;
int g;
int h;




void receiveFootSensor(const std_msgs::Int32MultiArray& msg)
{
    if (msg.data.size()!=8) {
        qDebug("the size of sensor data is in wrong");
        return;
    }

    ROS_INFO("I heard: [%d  %d %d %d %d  %d %d %d]", (int)msg.data[0],(int)msg.data[1],(int)msg.data[2],(int)msg.data[3],(int)msg.data[4],(int)msg.data[5],(int)msg.data[6],(int)msg.data[7]);
    double temp[8];
    int tempInt[8];
    //Left Foot
    temp[0]=msg.data[0]-1023;
    temp[1]=-1*(msg.data[1]-929);
    temp[2]=msg.data[2]-3035;
    temp[3]=-1*(msg.data[3]-3099);
    //Right Foot
    temp[4]=msg.data[4]-3041;
    temp[5]=-1*(msg.data[5]-3005);
    temp[6]=msg.data[6]-1138;
    temp[7]=-1*(msg.data[7]-1006);

    //normalizing data of sensors
    temp[0]=temp[0]*(100.0/105);
    temp[1]=temp[1]*(100.0/100);
    temp[2]=temp[2]*(100.0/117);
    temp[3]=temp[3]*(100.0/118);
    temp[4]=temp[4]*(100.0/109);
    temp[5]=temp[5]*(100.0/115);
    temp[6]=temp[6]*(100.0/107);
    temp[7]=temp[7]*(100.0/107);

    tempInt[0]=temp[0];
    tempInt[1]=temp[1];
    tempInt[2]=temp[2];
    tempInt[3]=temp[3];
    tempInt[4]=temp[4];
    tempInt[5]=temp[5];
    tempInt[6]=temp[6];
    tempInt[7]=temp[7];

    a=tempInt[0];
    b=tempInt[1];
    c=tempInt[2];
    d=tempInt[3];
    e=tempInt[4];
    f=tempInt[5];
    g=tempInt[6];
    h=tempInt[7];
    ROS_INFO("I heard: [%d  %d %d %d %d %d %d %d]", a,b,c,d,e,f,g,h);
    //deleting data with negative sign
    if (a<0) {
        a=0;

    }
    if (b<0) {
        b=0;

    }
    if (c<0) {
        c=0;

    }
    if (d<0) {
        d=0;

    }
    if (e<0) {
        e=0;

    }
    if (f<0) {
        f=0;

    }
    if (g<0) {
        g=0;

    }
    if (h<0) {
        h=0;

    }

    // ROS_INFO("I heard: [%d  %d %d %d]", tempInt[0],tempInt[1],tempInt[2],tempInt[3]);
}






//int a,b,c,d,e,f,g,h;

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
temp=R_left*vec_A_E;
//A=temp(2)+z_left;
a=int(((.02-temp(2)-z_left)+abs(.02-temp(2)-z_left))*5000/2);
temp=R_left*vec_B_F;
//B=temp(2)+z_left;
b=int(((.02-temp(2)-z_left)+abs(.02-temp(2)-z_left))*5000/2);
temp=R_left*vec_C_G;
//C=temp(2)+z_left;
c=int(((.02-temp(2)-z_left)+abs(.02-temp(2)-z_left))*5000/2);
temp=R_left*vec_D_H;
//D=temp(2)+z_left;
d=int(((.02-temp(2)-z_left)+abs(.02-temp(2)-z_left))*5000/2);
temp=R_right*vec_A_E;
//E=temp(2)+z_right;
e=int(((.02-temp(2)-z_right)+abs(.02-temp(2)-z_right))*5000/2);
temp=R_right*vec_B_F;
//F=temp(2)+z_right;
f=int(((.02-temp(2)-z_right)+abs(.02-temp(2)-z_right))*5000/2);
temp=R_right*vec_C_G;
//G=temp(2)+z_right;
g=int(((.02-temp(2)-z_right)+abs(.02-temp(2)-z_right))*5000/2);
temp=R_right*vec_D_H;
//H=temp(2)+z_right;
h=int(((.02-temp(2)-z_right)+abs(.02-temp(2)-z_right))*5000/2);


 //ROS_INFO("z_left=%f  A=%d,B=%d,C=%d,D=%d\tz_right=%f  E=%d,F=%d,G=%d,H=%d",z_left,a,b,c,d,z_right,e,f,g,h);

//    ROS_INFO("x_left=%f,x_right=%f,y_left=%f,y_right=%fz_left=%f,z_right=%f",x_left,x_right,y_left,y_right,z_left,z_right);

}





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
    //QTime myTimer2;
    //myTimer2.start();
    TaskSpaceOffline SURENAOffilneTaskSpace;
    //ROS_INFO("time2 = %d",myTimer2.elapsed());
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
    //SURENAOffilneTaskSpace.GetAccVelPos();
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

    ros::Subscriber sub = nh.subscribe("/surena/bump_sensor_state", 1000, receiveFootSensor);

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

    ros::Rate loop_rate(200);
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
        //QTime myTimer;
        //myTimer.start();
        //-------------for detecting the full contact of foot with ground-------------//
        if ((a)>=footSensorSaturation && (b)>=footSensorSaturation && (c)>=footSensorSaturation && (d)>=footSensorSaturation){
            qDebug("swing foot landing is successful");
            aState=true;
            bState=true;
            cState=true;
            dState=true;
          //  LeftFootLanded=true;//this variable is used for flag up when the left foot have a full contact with the ground

            //do nothing all sensors are on the ground
            teta_motor_L=teta_motor_L;
            phi_motor_L=phi_motor_L;

            teta_motor_R=teta_motor_R;
            phi_motor_R=phi_motor_R;
            //exit(0);//will finish the code but during walking is not true
        }


        else {//---------when the four sensors are not saturated-->>>>during landing------//

            LeftFootLanded=false;
            if ((a)>=footSensorthreshold ){aState=true;}else {aState=false;}
            if ((b)>=footSensorthreshold ){bState=true;}else {bState=false;}
            if ((c)>=footSensorthreshold ){cState=true;}else {cState=false;}
            if ((d)>=footSensorthreshold ){dState=true;}else {dState=false;}


            //-----------------Pitch left ankle motor control---------------//
            if (abs(b-a)>=abs(c-d)) {
                if (abs(a-b)<100) {
                    teta_motor_L=1*teta_motor_L+k1*(a-b);
                }
                else {
                    teta_motor_L=teta_motor_L;
                }
            }
            else {
                if (abs(d-c)<100) {
                    teta_motor_L=1*teta_motor_L+k1*(d-c);
                }
                else {
                    teta_motor_L=teta_motor_L;
                }
            }


            //----------------Roll left ankle motor control---------------//
            if (abs(c-b)>=abs(d-a)) {
                if (abs(c-b)<100) {
                    phi_motor_L=1*phi_motor_L+k2*(c-b);
                }
                else {
                    phi_motor_L=phi_motor_L;
                }
            }
            else {
                if (abs(a-d)<100) {
                    phi_motor_L=1*phi_motor_L+k2*(d-a);
                }
                else {
                    phi_motor_L=phi_motor_L;
                }
            }
        }



        //------------------------saturation of ankle motors----------------------------//
        if ((abs(phi_motor_L))>0.9) {
            phi_motor_L=0.9;
        }
        if ((abs(teta_motor_L))>0.9) {
            teta_motor_L=0.9;
        }
        if ((abs(phi_motor_R))>0.9) {
            phi_motor_L=0.9;
        }
        if ((abs(teta_motor_R))>0.9) {
            teta_motor_R=0.9;
        }


        // ROS_INFO("I heard data of sensors : [%f %f %f %f]",a,b,c,d);


        //-----------------------------------------------------------------------------------------------------//
        //-----------------start phase--initializing the height of pelvis for walking--------------------------//
        //-----------------------------------------------------------------------------------------------------//
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


        //-----------------------------------------------------------------------------------------------------//
        //------------------------------- main loop of cyclic walking -----------------------------------------//
        //-----------------------------------------------------------------------------------------------------//

        if (StartTime>DurationOfStartPhase && StartTime<(DurationOfStartPhase+SURENAOffilneTaskSpace.MotionTime)){

            double m1;
            double m2;
            double m3;
            double m4;
            double m5;
            double m6;
            double m7;
            double m8;

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
                m7=m(6,0);
                m8=m(7,0);

                P=SURENAOffilneTaskSpace.PelvisTrajectory (SURENAOffilneTaskSpace.globalTime);
                SURENAOffilneTaskSpace.globalTime=SURENAOffilneTaskSpace.globalTime+SURENAOffilneTaskSpace._timeStep;

                if (round(SURENAOffilneTaskSpace.globalTime)<=round(SURENAOffilneTaskSpace.MotionTime)){

                    //--------------------- hip roll modification is commented and is not active --------------------------//
                    //--------------------- hip roll modification is commented and is not active --------------------------//
//                    if (SURENAOffilneTaskSpace.DoubleSupport!=true) {
//                        //For modifying the angle of roll during single support
//                        RollTime=RollTime+SURENAOffilneTaskSpace._timeStep;
//                        MinimumJerkInterpolation Coef;
//                        MatrixXd RollAngle(1,3);
//                        RollAngle<<0,0.0,0;
//                        MatrixXd RollAngleVelocity(1,3);
//                        RollAngleVelocity<<0.000,INFINITY,0.000;
//                        MatrixXd RollAngleAcceleration(1,3);
//                        RollAngleAcceleration<<0,INFINITY,0;
//                        MatrixXd Time22(1,3);
//                        Time22<<0,(SURENAOffilneTaskSpace.TSS/2),SURENAOffilneTaskSpace.TSS;
//                        MatrixXd CoefRoll =Coef.Coefficient(Time22,RollAngle,RollAngleVelocity,RollAngleAcceleration);
//                        //StartTime=StartTime+SURENAOffilneTaskSpace._timeStep;
//                        MatrixXd outputRollAngle;
//                        if (RollTime<=(SURENAOffilneTaskSpace.TSS/2)) {
//                            outputRollAngle= SURENAOffilneTaskSpace.GetAccVelPos(CoefRoll.row(0),RollTime,0,5);
//                            hipRoll=outputRollAngle(0,0);
//                        }
//                        else {
//                            outputRollAngle =SURENAOffilneTaskSpace.GetAccVelPos(CoefRoll.row(1),RollTime,SURENAOffilneTaskSpace.TSS/2,5);
//                            hipRoll=outputRollAngle(0,0);
//                        }
//                    }
//                    else {
//                        hipRoll=0;
//                        RollTime=0;
//                    }


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
                            -1*m8*(M_PI/180),
                            0;

                    PoseLFoot<<m1,
                            m2,
                            m3,
                            0,
                            -1*m4*(M_PI/180),
                            0;


                    //// hip roll modification
                    //                    if (SURENAOffilneTaskSpace.LeftSupport==true && SURENAOffilneTaskSpace.HipRollModification==true){
                    //                        SURENA.doIKhipRollModify("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot,-1*hipRoll);
                    //                        SURENA.doIKhipRollModify("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot,0);
                    //                    }

                    //                    else if (SURENAOffilneTaskSpace.HipRollModification==true && SURENAOffilneTaskSpace.LeftSupport!=true )  {
                    //                        SURENA.doIKhipRollModify("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot,0);
                    //                        SURENA.doIKhipRollModify("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot,0*hipRoll);
                    //                    }

                    SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot);
                    SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot);
                }
            }
        }

        //-----------------------------------------------------------------------------------------------------//
        //------------------- end phase-- finializing height of pelvis to home position -----------------------//
        //-----------------------------------------------------------------------------------------------------//

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

        if (LeftFootLanded==true) {//else if is for situation that foot landed successfully on the ground
            cntrl[0]=0.0;
            cntrl[1]=cntrl[1];
            cntrl[2]=cntrl[2];
            cntrl[3]=cntrl[3];
            cntrl[4]=cntrl[4];
            cntrl[5]=cntrl[5];
            cntrl[6]=cntrl[6];//roll
            cntrl[7]=cntrl[7];
            cntrl[8]=cntrl[8];
            cntrl[9]=cntrl[9];
            cntrl[10]=cntrl[10];
            cntrl[11]=cntrl[11];
            cntrl[12]=cntrl[12];

            walk=false;
            endPhase=false;
            startPhase=false;
        }


        else {//else  is for situation that sensor is contacting and adapting ground
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
            cntrl[11]=+0*teta_motor_L+links[11].JointAngle;
            cntrl[12]=+0*phi_motor_L+links[12].JointAngle;
        }


        vector<int> qref(12);
        qref=QC.ctrldata2qc(cntrl);


        for(int  i = 0;i < 12;i++)
        {
           // msg.data.push_back(qref[i]);
            msg.data.push_back(0);
        }

        SendGazebo(links);
        chatter_pub.publish(msg);
        //  ROS_INFO("t={%d} c={%d}",timer.elapsed(),count);
//        ROS_INFO("time = %d", myTimer.elapsed());

        ros::spinOnce();
        loop_rate.sleep();
        //ROS_INFO("time = %d", myTimer.elapsed());
        ++count;
    }

    return 0;
}

