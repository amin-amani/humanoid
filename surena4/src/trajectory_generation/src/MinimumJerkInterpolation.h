#ifndef MINIMUMJERKINTERPOLATION_H
#define MINIMUMJERKINTERPOLATION_H

#include <iostream>
#include <fstream>
#include "Eigen/Dense"
#include <qdebug.h>
#include <qmath.h>
using namespace Eigen;
using namespace std;
class MinimumJerkInterpolation
{
public:
    MinimumJerkInterpolation();
    MatrixXd Coefficient(MatrixXd time, MatrixXd p, MatrixXd dp, MatrixXd dpp);
void PrintArray(MatrixXd array, QString filename);
    MatrixXd diff(MatrixXd E);
    int length(MatrixXd in);
    QVector<int> findIsinf(MatrixXd in,bool inf);
    MatrixXd MatrixIndex(MatrixXd in, QVector<int> indexes);
    MatrixXd AddtoIndexes(MatrixXd in, MatrixXd val, QVector<int> indexes);
    MatrixXd Coefficient1(MatrixXd xp, MatrixXd ord, MatrixXd con, double fig_res);
    MatrixXd PMaker(double t);
    MatrixXd isinf(MatrixXd in);
    MatrixXd isnan(MatrixXd in);
    double SUM1(int first, int last, MatrixXd Mat);
    MatrixXd GetAccVelPos(MatrixXd Coef, double time, double ti, int PolynomialOrder);
};

#endif // MINIMUMJERKINTERPOLATION_H
