#ifndef TASKSPACEONLINE3_H
#define TASKSPACEONLINE3_H
#include "Eigen/Dense"
#include <qdebug.h>
#include <qmath.h>
#include <Eigen/Geometry>
#include <iostream>
#include <cstdlib>
#include <link.h>
#include "Eigen/eiquadprog.h"
#include "Eigen/Core"
#include "Eigen/Cholesky"
#include "Eigen/LU"
#include<math.h>
#include<qtimer.h>
#include <QElapsedTimer>
#include "MinimumJerkInterpolation.h"
#include <QTime>
using namespace std;
using namespace Eigen;
class TaskSpaceOnline3
{
public:

    double LeftHipRollModification;
    double RightHipRollModification;
    double FirstHipRollModification;
    double HipPitchModification;
    double PelvisRollRange;

    double YpMax;//Rm*0.5*_pelvisLength*1.2;
    double Yd;//1*Rd*YpMax
    double YStMax;// Start motion parameter in y direction
    double YEndMax;//End motion parameter in y direction
    double Xe;// Distance of pelvis and rear ankle in DS Xe=Sc*StepLength/(Rse+1)*1.2;
    double Xs;// Distance of pelvis and front ankle in DSP Xs=Rse*Xe*.90;

    double NStride;
    double NStep;
    int StepNumber=1;

    double TStart=6;
    double TEnd=8;//6

    double StepLength;//0.0840000;
    double DesiredVelocity;//0.050;

    double _lengthOfThigh=0.3700;
    double _lengthOfShank=0.3600;
    double _lenghtOfAnkle=0.112;
    double _lengthOfHip=0.10900;
    double _pelvisLength=0.23;

    double ReferencePelvisHeight;
    double InitialPelvisHeight=0.95100;
    double OldPelvisZ;
    double NewPlevisZ;
    double T_end_of_SS;
    double T_end_of_first_SS;
    double T_end_of_last_SS;
    double h_end_of_SS;
    double timeStepT;

    double T_beta;

    double Tm;
    double Tc;
    double TDs;
    double TSS;
    double TGait;
    double MotionTime;
    double TMinPelvisY; // The time that pelvis reaches its minimum distance in y direction
    double TMaxAnkle;//0.53 % The time that ankle reaches its maximum distance in z direction
    double TMaxPelvisY; // The time that pelvis reaches its maximum distance in y direction
    double TMinPelvisZ; // The time that pelvis reaches its minimum distance in z direction
    double TMaxPelvisZ; // The time that pelvis reaches its maximum distance in z direction


    // last step: timing parameter of pelvis motion
    double T_end_p_sx_rel;
    double T_end_p_sx;
    double T_end_p_sy;
    double T_end_p_dy;
    double T_end_p_ey;
    // last step: timing parameter of ankle motion
    double T_end_a_s;
    double T_end_a_e;
    double T_end_a_d;
    // first step: timing parameter of pelvis motion
    double T_st_p_sy;
    double T_st_p_dy;
    double T_st_p_ey;
    double T_st_p_sx;
    // first step: timing parameter of ankle motion

    double T_st_a_d;
    double T_end_p_sz;
    double T_end_p_dz;
    double T_end_p_ez;
    double T_end_p_sq;
    double  T_st_p_sz;
    double T_st_p_dz;
    double T_st_p_ez;
    double T_st_p_sq;
    double T_s_st;

    MinimumJerkInterpolation CoefOffline;


//    bool LeftSupport;
//    bool Leftmoves;
//    bool LeftSS;
//    bool Rightmoves;
//    bool RightSS;
//    bool RightSupport;
    bool DoubleSupport;
//    bool RightSensorActive;
//    bool LeftSensorActive;

    bool toeOff;

    int footIndex;
//    int kbumpL;
//    int kbumpR;
    double currentRightFootZ;
    double currentRightFootX2;
    double currentRightFootY2;

    double currentLeftFootZ;
    double currentLeftFootX2;
    double currentLeftFootY2;

//    double oldLeftFootZ;
//    double oldLeftFootX2;
//    double oldLeftFootY2;

//    double oldRightFootZ;
//    double oldRightFootX2;
//    double oldRightFootY2;

    bool HipRollModification;
//    double HeelLandingAnglePitch;

    bool RightFootOrientationAdaptator;
    bool LeftFootOrientationAdaptator;
//    double ToeOffAnglePitch;

    double YOffsetOfAnkletrajectory;

//    double RollTimeSS;
//    double RollTimeDs;

//    bool firstStep;

//    double xa_st_m;
//    double er;


//    double localTime;
//    double localTime1;
    double localTiming;
    int localtimingInteger;
//    double Delta;
//    double L_2leg_Ds;
//    double Rzp;

    double XofAnkleMaximumHeight;

    double AnkleMaximumHeight;
//    double MinHeightPelvis;
//    double MaxHeightPelvis;

//    double za_st_m;
//    double za_end_m;
//    double xa_end_m;



//    double Rqa;
//    double  Ra_i;
//    double  Ra_f;
//    double  Rla_i;
//    double  Rla_f;


//    double Ra_st_i;
//    double  Ra_st_f;


//    double  Ra_end_i;
//    double Ra_end_f;


//    double Rla_st_i;
//    double  Rla_st_f;

//    double  Rla_end_i;
//    double  Rla_end_f;
    double globalTime;
    double time;
    double _timeStep=.005;
    QVector<double> timeVector;



//    QVector<double> RightFootXTrajectory;
//    QVector<double> RightFootYTrajectory;
//    QVector<double> RightFootZTrajectory;

//    QVector<double> RightFootAlphaTrajectory;
//    QVector<double> RightFootBethaTrajectory;
//    QVector<double> RightFootGamaTrajectory;

//    QVector<double> LeftFootXTrajectory;
//    QVector<double> LeftFootYTrajectory;
//    QVector<double> LeftFootZTrajectory;

//    QVector<double> LeftFootAlphaTrajectory;
//    QVector<double> LeftFootBethaTrajectory;
//    QVector<double> LeftFootGamaTrajectory;

//    QVector<double> RightFootXVelocity;
//    QVector<double> RightFootYVelocity;
//    QVector<double> RightFootZVelocity;

//    QVector<double> LeftFootXVelocity;
//    QVector<double> LeftFootYVelocity;
//    QVector<double> LeftFootZVelocity;

//    QVector<double> RightFootXAcceleration;
//    QVector<double> RightFootYAcceleration;
//    QVector<double> RightFootZacceleration;

//    QVector<double> LeftFootXAcceleration;
//    QVector<double> LeftFootYAcceleration;
//    QVector<double> LeftFootZAcceleration;




    MatrixXd Cx;
    MatrixXd Cz;
    MatrixXd Cx_st;
    MatrixXd Cz_st;
    MatrixXd Cx_end;
    MatrixXd Cz_end;
    MatrixXd C_st_pitch_al;
    MatrixXd C_Ds_pitch_al;
    MatrixXd C_Ds_pitch_ar;
    MatrixXd C_Ds2_pitch_ar;
    MatrixXd C_Ds2_pitch_al;
    MatrixXd C_ss_pitch_ar;
    MatrixXd C_end_pitch_ar;
    MatrixXd C_Dsf_pitch_ar;
    MatrixXd C_st_x_al;
    MatrixXd  C_st_z_al;
    MatrixXd C_st_z_al_end_of_SS;
    MatrixXd C_cy_x_al;
    MatrixXd C_cy_z_ar;
    MatrixXd C_cy_z_ar_end_of_SS;
    MatrixXd C_cy_y_ar;
    MatrixXd C_cy_y_al;
    MatrixXd C_st_y_al;
    MatrixXd C_end_y_ar;
    MatrixXd C_end_z_ar;
    MatrixXd C_end_z_ar_end_of_SS;
    MatrixXd C_end_x_ar;
    MatrixXd C_cy_x_ar;
    MatrixXd Cx_st_p;
    MatrixXd Cx_end_p;
    MatrixXd Cx_p_i;
    MatrixXd Cy_p_i;
//    QVector<double> CoMXVector;
//    QVector<double> CoMYVector;
//    QVector<double> CoMZVector;

//    QVector<double> CoMXVelocityVector;
//    QVector<double> CoMYVelocityVector;
//    QVector<double> CoMZVelocityVector;

//    QVector<double> CoMXAccelerationVector;
//    QVector<double> CoMYAccelerationVector;
//    QVector<double> CoMZAccelerationVector;

    MatrixXd Cx_p;
    MatrixXd Cy_p;
    MatrixXd Cy_end_pa;
    MatrixXd Cy_end_pb;
    MatrixXd Cy_st_pa;
    MatrixXd Cy_st_pb;
    MatrixXd Cz_p;
    MatrixXd Cz_st_b;
    MatrixXd Cz_st_a;
    MatrixXd Cz_end_b;
    MatrixXd Cz_end_a;
    MatrixXd Cz_mod_p;
    MatrixXd Cz_mod_st_p;
 bool side_extra_step_length=false;

    TaskSpaceOnline3();

    void SetParameters();

    void CoeffArrayAnkle();
    void CoeffArrayPelvis();
    void CoeffSideStartEnd();
    MatrixXd PelvisTrajectory(double time);
    //void CoeffArrayFootAngle();
   // MatrixXd ModificationOfPelvisHeight(double time, int n, double localtiming, bool RFT_state, bool LFT_state, bool LastDSIndex);
    void CoeffArrayPelvisZMod();

    double RollCharge(double t, double t_start, double t_end, double magnitude);
    double RollDecharge(double t, double t_start, double t_end, double magnitude);
    MatrixXd RollAngleModification(double time);

    MatrixXd AnkleTrajectory(double time, int n, double localtiming);
    double PelvisRoll(double time);
};

#endif // TASKSPACEONLINE3_H
