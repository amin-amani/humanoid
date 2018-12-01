
//#include <cnoid/SimpleController>
#include <vector>
#include <iostream>
#include <QString>
#include <QList>
#include "Headers/Robot.h"
#include"Headers/TaskSpace.h"
#include"Headers/taskspaceofflineRamp.h"
#include <qmath.h>
#include <cstring>
#include <cnoid/BodyLoader>
#include "src/BodyPlugin//BodyItem.h"
#include <cnoid/Sensor>
#include <QApplication>
#include<Headers/mainwindow.h>
//#include <cnoid/MainWindow>
//#include <ros/node_handle.h>
//#include <sensor_msgs/JointState.h>
//#include <sensor_msgs/image_encodings.h>

using namespace cnoid;
using namespace std;

const double pgain[] = {
    10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0,
    10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0,
    8000.0, 8000.0, 3000.0,3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0,
    3000.0,3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0 };

const double dgain[] = {
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0};

QApplication a();

    MainWindow miladplot;
class SURENA4Offline : public SimpleController
{



    BodyPtr ioBody;
    Robot SURENA;
    TaskSpaceOfflineRamp SURENAOffilneTaskSpaceRamp;
    QList<LinkM> links;
    MatrixXd PoseRoot;
    MatrixXd PoseRFoot;
    MatrixXd PoseLFoot;
    //BodyItemPtr SurenaItemBody;
    //BodyItem* mil;
    vector<double> qref;
    vector<double> qold;
    double footPitchOld;
    double footRollOld;
      //  milad();

    double dt;
    SimpleControllerIO* tempIO;
public:

    double StartTime=0;
    double RollTime=0;
    double WalkTime=0;
    double  DurationOfStartPhase=6;
    double  DurationOfendPhase=6;





    BodyLoader bodyloader;

    RateGyroSensorPtr waistGyro;
    RateGyroSensorPtr ankleRightGyro;
    RateGyroSensorPtr ankleLefttGyro;

    ForceSensorPtr ankleRightForce;
    ForceSensorPtr ankleLeftForce;

    AccelerationSensor* waistAccelSensor;
    AccelerationSensor* ankleLeftAccelSensor;
    AccelerationSensor* ankleRightAccelSensor;


    virtual bool initialize(SimpleControllerIO* io) override
    {
       // milad();
miladplot.show();
        // std::string model_path("model/SR1.body");
        // BodyPtr robot = bodyloader.load(model_path.c_str());
        ioBody = io->body();
        io->setLinkInput(io->body()->link("LLeg_AnkleR_J6"), LINK_POSITION);
        //            cout << "dof: " <<ioBody->numJoints() << endl;
        //            cout << "base link name: " << ioBody->rootLink()->name() << endl;
        //            cout << "base link pos: \n" << ioBody->rootLink()->p() << endl;
        cout << "base link rot: \n" << ioBody->rootLink()->R() << endl;

        dt = io->timeStep();
        tempIO=io;


        waistAccelSensor = ioBody->findDevice<AccelerationSensor>("WaistAccelSensor");
        io->enableInput(waistAccelSensor);


        for(auto joint : ioBody->joints()){
            joint->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint);
            qref.push_back(joint->q());
        }
        qold = qref;
        return true;
    }

    virtual bool control() override
    {
        bool startPhase=true;
        bool endPhase=true;
        PoseRoot.resize(6,1);
        PoseRFoot.resize(6,1);
        PoseLFoot.resize(6,1);


        //*******************This part of code is for initialization of joints of the robot for walking**********************************
        if (startPhase==true && StartTime<=DurationOfStartPhase) {
            MinimumJerkInterpolation Coef;
            MatrixXd ZPosition(1,2);
            ZPosition<<0.95100,0.860;
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
            StartTime=StartTime+0.002;SURENAOffilneTaskSpaceRamp._timeStep;

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
                SURENAOffilneTaskSpaceRamp.globalTime=SURENAOffilneTaskSpaceRamp.globalTime+SURENAOffilneTaskSpaceRamp._timeStep;

                if (round(SURENAOffilneTaskSpaceRamp.globalTime)<=round(SURENAOffilneTaskSpaceRamp.MotionTime)){




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
            ZPosition<<0.860,0.95100;
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
        //   const Vector3 zmp;

        //      zmp[0] = 0.5;
        //      zmp[1] = 0.5;
        //   zmp[2] = 0;
        //mil->setZmp(zmp);
        //SurenaItemBody->setZmp(zmp);

        Vector3 dv = waistAccelSensor->dv();
        // cout << "dv(" << dv(0) <<"," << dv(1) <<","<< dv(2) << ")" << endl << flush;

        links = SURENA.GetLinks();
        for(int  i = 1;i < 29;i++)
        {
            qref[i-1] = links[i].JointAngle;
            //cout << qref[i-1] <<" , "<<flush;
        }
        //cout<<ioBody->numJoints();

        for(int i=0; i < ioBody->numJoints(); ++i){
            Link* joint = ioBody->joint(i);
            double u;
            double q = joint->q();
            double dq = (q - qold[i]) / dt;
            u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];//note:the joint velocity should not be zero
            qold[i] = q;
            joint->u() = u;
        }

        return true;

    }


//Plot.show();

};

//CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(SURENA4Offline)
