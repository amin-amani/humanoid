#ifndef TASKSPACEOFFLINE_H
#define TASKSPACEOFFLINE_H

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
using namespace std;
using namespace Eigen;

class TaskSpaceOffline
{  MinimumJerkInterpolation CoefOffline;
    double _lengthOfThigh;
    double _lengthOfShank;
    double _lenghtOfAnkle;
    double _lengthOfHip;
    double _pelvisLength;
    double _heelLength;
    double _toeLength;
public:
    TaskSpaceOffline();
    bool LeftSupport;
    bool RightSupport;
    bool DoubleSupport;
    double HeelLandingAnglePitch;
    double ToeOffAnglePitch;
    double TStart;
    double TEnd;
    double timeStepT;
    double TDs;
    double TGait;
    double T_beta;
    double TSS;
    double YOffsetOfAnkletrajectory;

    double xa_st_m;
    double er;
    double TMinPelvisZ; // The time that pelvis reaches its minimum distance in z direction
    double TMinPelvisY; // The time that pelvis reaches its minimum distance in y direction
    double TMaxAnkle;// The time that ankle reaches its maximum distance in z direction
    double TMaxPelvisZ; // The time that pelvis reaches its maximum distance in z direction
    double TMaxPelvisY;
    double Tc;
    double NStride;
    double MotionTime;
    double Rm;
    double YpMax;
    double Rd;
    double Yd;
    double YStMax;// Start motion parameter in y direction
    double YEndMax;//End motion parameter in y direction
    double Xe;// Distance of pelvis and rear ankle in DS
    double Sc;
    double Rse;
    double StepLength;
    double Xs;// Distance of pelvis and front ankle in DSP
    double DesiredVelocity;
    double Delta;
    double L_2leg_Ds;
    double Rzp;
    double AnkleMaximumHeight;
    double XofAnkleMaximumHeight;
    double za_c;
    double MinHeightPelvis;
    double MaxHeightPelvis;
    double ReferencePelvisHeight;
    double za_st_m;
    double za_end_m;
    double xa_end_m;
    double T_end_p_sx;
    double T_end_p_sy;
    double T_end_p_dy;
    double T_end_p_ey;

    double T_end_p_sz;
    double T_end_p_dz;
    double T_end_p_ez;

    double T_end_p_sq;

    double T_end_a_s;
    double T_end_a_e;
    double T_end_a_d;


    double T_st_p_sy;
    double T_st_p_dy;
    double T_st_p_ey;

    double T_st_p_sx;

    double  T_st_p_sz;
    double T_st_p_dz;
    double T_st_p_ez;

    double T_st_p_sq;

    double T_st_a_s;
    double  T_st_a_d;
    double T_s_st;

    double Rqa;
    double  Ra_i;
    double  Ra_f;
    double  Rla_i;
    double  Rla_f;


    double Ra_st_i;
    double  Ra_st_f;


    double  Ra_end_i;
    double Ra_end_f;


    double Rla_st_i;
    double  Rla_st_f;

    double  Rla_end_i;
    double  Rla_end_f;
double globalTime;
double time;
double _timeStep;
    QVector<double> timeVector;

bool toeOff;
 bool HipRollModification;

    QVector<double> RightFootXTrajectory;
    QVector<double> RightFootYTrajectory;
    QVector<double> RightFootZTrajectory;

    QVector<double> RightFootAlphaTrajectory;
    QVector<double> RightFootBethaTrajectory;
    QVector<double> RightFootGamaTrajectory;

    QVector<double> LeftFootXTrajectory;
    QVector<double> LeftFootYTrajectory;
    QVector<double> LeftFootZTrajectory;

    QVector<double> LeftFootAlphaTrajectory;
    QVector<double> LeftFootBethaTrajectory;
    QVector<double> LeftFootGamaTrajectory;

    QVector<double> RightFootXVelocity;
    QVector<double> RightFootYVelocity;
    QVector<double> RightFootZVelocity;

    QVector<double> LeftFootXVelocity;
    QVector<double> LeftFootYVelocity;
    QVector<double> LeftFootZVelocity;

    QVector<double> RightFootXAcceleration;
    QVector<double> RightFootYAcceleration;
    QVector<double> RightFootZacceleration;

    QVector<double> LeftFootXAcceleration;
    QVector<double> LeftFootYAcceleration;
    QVector<double> LeftFootZAcceleration;
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
    MatrixXd C_st_z_al;
    MatrixXd C_cy_x_al;
    MatrixXd C_cy_z_ar;
    MatrixXd C_cy_y_ar;
    MatrixXd C_cy_y_al;
    MatrixXd C_st_y_al;
    MatrixXd C_end_y_ar;
    MatrixXd C_end_z_ar;
    MatrixXd C_end_x_ar;
    MatrixXd C_cy_x_ar;
    MatrixXd Cx_st_p;
    MatrixXd Cx_end_p;
    MatrixXd Cx_p_i;
    MatrixXd Cy_p_i;
    QVector<double> CoMXVector;
    QVector<double> CoMYVector;
    QVector<double> CoMZVector;

    QVector<double> CoMXVelocityVector;
    QVector<double> CoMYVelocityVector;
    QVector<double> CoMZVelocityVector;

    QVector<double> CoMXAccelerationVector;
    QVector<double> CoMYAccelerationVector;
    QVector<double> CoMZAccelerationVector;

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


    void SetParameters();
    MatrixXd AnkleTrajectory(double time);
    MatrixXd GetAccVelPos(MatrixXd Coef, double time, double ti, int PolynomialOrder);
    void CoeffArrayAnkle();
    void CoeffArrayPelvis();
    MatrixXd PelvisTrajectory(double time);
    void CoeffArrayFootAngle();
};

#endif // TASKSPACEOFFLINE_H
