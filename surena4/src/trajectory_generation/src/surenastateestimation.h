#ifndef SURENASTATEESTIMATION_H
#define SURENASTATEESTIMATION_H
#include "Eigen/Dense"
#include <qdebug.h>
#include <vector>
#include <qmath.h>
#include <cstring>
#include<qdebug.h>

using namespace  Eigen;

class SurenaStateEstimation
{
public:
    SurenaStateEstimation();
    MatrixXd MatrixRotationPelvis(double q0, double q1, double q2, double q3);
    MatrixXd PelvisLocalChange(MatrixXd RotationMatrix, MatrixXd localPosition);
    MatrixXd MatrixRotationPelvisRPY(double roll, double pitch, double yaw);
};

#endif // SURENASTATEESTIMATION_H
