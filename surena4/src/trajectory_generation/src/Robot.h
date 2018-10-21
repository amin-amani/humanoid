#ifndef ROBOT_H
#define ROBOT_H
#include "Eigen/Dense"
#include "LinkM.h"
#include <Eigen/Geometry>
#include <iostream>
#include <cstdlib>
#include <QObject>
#include <QFile>
#include <QFileInfo>
#include <QDebug>
#include <QtMath>
#include <QList>
#include <QDir>

using namespace std;
using namespace Eigen;

class Robot
{
    bool FileExists(QString path);
    MatrixXd _leftLegRoute;
    MatrixXd _rightLegRoute;
    QList<QByteArray> GetContentOfRobot(QString name, QByteArray content);
    MatrixXd ExtractionOfMatrix(QByteArray data);
    MatrixXd Rodrigues(MatrixXd omega, double angle);
    QList<LinkM> CreateRobotLinks(QByteArray content);
    void ForwardKinematic(int input);
    MatrixXi _route;
    void SetJointAngle(MatrixXd JointAngle, MatrixXi Route);
    MatrixXi FindRoutfromRoot2Destination(QString Destination);
    void MoveJoints(MatrixXi route, MatrixXd deltaJointAngle);
    Vector3d Rot2omega(Matrix3d rotation);
    MatrixXd CalcTaskSpaceError(LinkM Target, QString current);
    MatrixXd CalcJacobian(MatrixXi route);
    double IKLevenbergMarquardt(LinkM Target, QString current);
public:
    Robot();
    QList<LinkM> Links;
    QList<LinkM> GetLinks();
    MatrixXi Getroute();
    QHash <QString,int> MapingName2ID;
    QHash <int,QString> MapingID2Name;
    MatrixXi GetLeftLegRoute();
    void SetLeftLegRoute(MatrixXd leftLegRoute );
    MatrixXi GetRightLegRoute();
    void SetRightLegRoute(MatrixXd rightLegRoute);

    QByteArray content;
    void doIK1(int var, QString link);
    void doIK(QString link, MatrixXd PoseLink, QString root, MatrixXd PoseRoot);
    MatrixXd IKAnalytical(LinkM Body, double D, double E, double A, double B, LinkM Foot);
    MatrixXd RPitch(double theta);
    MatrixXd RRoll(double phi);
private:
    int Sign(double v);
    void ForwardKinematicPrimary(int input);
};

#endif // ROBOT_H
