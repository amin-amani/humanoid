#include "surenastateestimation.h"
#include "Eigen/Dense"
#include <qdebug.h>
#include <vector>
#include <qmath.h>
#include <cstring>
#include<qdebug.h>

using namespace  Eigen;

SurenaStateEstimation::SurenaStateEstimation()
{
}

MatrixXd SurenaStateEstimation::MatrixRotationPelvis(double q0, double q1, double q2, double q3)
{
    double q0s, q1s, q2s, q3s;
    q0s = q0*q0;
    q1s = q1*q1;
    q2s = q2*q2;
    q3s = q3*q3;
    MatrixXd rotationMatrix(3,3);
    rotationMatrix << 1-2*(q2s+q3s), 2*(q1*q2-q0*q3), 2*(q0*q2-q1*q3),
            2*(q1*q2+q0*q3), 1-2*(q1s+q3s), 2*(q2*q3-q0*q1),
            2*(-q0*q2+q1*q3), 2*(q0*q1+q2*q3), 1-2*(q1s+q2s);
    return rotationMatrix;
}

MatrixXd SurenaStateEstimation::PelvisLocalChange(MatrixXd rotationMatrix, MatrixXd localPosition)
{
    MatrixXd localHeightChange(3,1);
    localHeightChange = rotationMatrix*localPosition;
    return localHeightChange;
}
