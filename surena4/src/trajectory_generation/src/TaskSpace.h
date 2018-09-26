#ifndef TASKSPACE_H
#define TASKSPACE_H
#include "Eigen/Dense"
#include <qdebug.h>
#include <qmath.h>
#include <Eigen/Geometry>
#include <iostream>
#include <cstdlib>
#include <link.h>
//#include "Eigen/QuadProg.h"
//#include "Eigen/testQPsolvers.hpp"
#include "Eigen/eiquadprog.h"
#include "Eigen/Core"
#include "Eigen/Cholesky"
#include "Eigen/LU"
#include<math.h>
//#include <unsupported/Eigen/Polynomials>
#include<qtimer.h>
#include <QElapsedTimer>
#include "MinimumJerkInterpolation.h"
using namespace std;
using namespace Eigen;

class TaskSpace
{
public:
    TaskSpace();
    double _maxXDCMOffset ;
    MatrixXd _previousCoef;
    bool _RightStart;
    bool _LeftStart;
    double _Xoffset;
    double _SpatialStepSize;
    double _gravity;
    double _PelvisHeight;
    double _XDesiredVelocity;
    double _YDesiredVelocity;
    double _omega;
    double _timeStep;
    double _stepDuration;
    int _nSteps;
    double _boundMaxFootHeight;
    double _desiredMAxFootHeight;
    double _desiredStepLength;
    double _desiredStepWidth;
    double _MaxStepLength;
    double _MaxStepWidth;
    double _MaxStepDuration;
    double _MinStepLength;
    double _MinStepWidth;
    double _MinStepDuration;
    double _desiredStepDuration;
    double _ankleLength;
    double _thighLength;
    double _shankLength;
    double _XMiddleOfStep;
    double _legLength;
    MatrixXd _xHip;
    MatrixXd _zHip;
    Vector2d  _measuredDCM;
    MatrixXd _sigma;
    double _sigmaDesired;
    double _X;
    double _xdelta;
    double _xdeltaNom;
    double _xdeltaMin;
    double _xdeltaMax;

    double _ydelta;
    double _ydeltaNom;
    double _ydeltaMin;
    double _ydeltaMax;

    double _xDCMOffset;
    double _yDCMOffset;

    double _xDCMOffsetNom;
    double _yDCMOffsetNom;
    MatrixXd _tempTigh;
    MatrixXd _tempShank;
    double _alpha1x;
    double _alpha2x;
    double _alpha3x;
    double _alphaT;
    double _alpha1y;
    double _alpha2y;
    double _alpha3y;
    int _n;

    MatrixXd _DCM;
     MatrixXd _CoM;
    double _XCoPDisplacementSS;
    double _YCoPDisplacementSS;
    double _mass;
    double _XError;
    double _YError;
    bool _walkstate;
    int _stepNumber;
    MatrixXd _CoPHeel;
    MatrixXd _CoPToe;
    MatrixXd _CoP;
    MatrixXd _gama;
    MatrixXd _CoPDisplacement;
    MatrixXd InitialCoP;
    MatrixXd InitialCoM;
    MatrixXd InitialDCM;
    MatrixXd _error;
    double _pelvisLength;
public:

    QVector<double> timeVector;
    QVector<double> DCMXVector;
    QVector<double> DCMYVector;
    QVector<double> InitCoPXVector;
    QVector<double> InitCoPYVector;
    QVector<double> EndCoPXVector;
    QVector<double> EndCoPYVector;
    MinimumJerkInterpolation Coef;
    double time;
    double globalTime;
    MatrixXd Input;
    QVector<double> RightFootXTrajectory;
    QVector<double> RightFootYTrajectory;
    QVector<double> RightFootZTrajectory;

    QVector<double> LeftFootXTrajectory;
    QVector<double> LeftFootYTrajectory;
    QVector<double> LeftFootZTrajectory;

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

    void SetDesiredMAxFootHeight(double desiredMaxFootHeight);
    void SetWalkState(bool walkState);
    void SetStepTimingGain(double alphaT4);
    void SetStepPositionXGain(double alpha1);
    void SetDeltaXGain(double alpha3);
    void SetDCMOffsetXGain(double alpha4);

    void SetStepPositionYGain(double alpha1);
    void SetDeltaYGain(double alpha3);
    void SetDCMOffsetYGain(double alpha4);
    MatrixXd GetAccVelPos(MatrixXd Coef, double time, double ti, int PolynomialOrder);
    MatrixXd QPController(int StepNumber, MatrixXd CoPDisplacementSS);
    void GetDesiredParameter();
    MatrixXd CoMDynamics(int stepNumber, double GamaX, MatrixXd CoPDisplacementSS);
    MatrixXd RightFoot();
      MatrixXd LeftFoot();
private:

};

#endif // TASKSPACE_H
