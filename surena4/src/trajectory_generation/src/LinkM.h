#ifndef LINK_H
#define LINK_H
#include "Eigen/Dense"
#include <string>
#include <QString>
#include <QObject>
#include <QList>
#include <iostream>

using namespace Eigen;
using namespace std;

class LinkM
{
public:

    QString GetName();
    void SetName(QString name);
    int GetID();
    void SetID(int ID);
    int GetSisterID();
    void SetSisterID(int SisterID);
    int GetMotherID();
    void SetMother(int MotherID);
    int GetChildID();
    void SetChildID(int childID);
    double GetMass();
    void SetMass(double Mass);
    MatrixXd GetMomentumOfInertia();
    void SetMomentumOfInertia(MatrixXd momentumOfInertia);
    Vector3d GetJointAxisVectorLocal();
    void SetJointAxisVectorLocal(Vector3d jointAxisVectorLocal);
    Vector3d GetJointPositionRelative2Parent();
    void SetJointPositionRelative2Parent(Vector3d jointPositionRelative2Parent);
    Vector3d GetCenterOfMassLocal();
    void SetCenterOfMassLocal(Vector3d centerOfMassLocal);

    LinkM(){cout << "LinkM" <<endl<<flush;}
    LinkM(QString name, int IDofLink, int IDofSyster, int IDofChild, int IDofMother, double AngleOfJoint, MatrixXd LocalAxisVectorOfJoint, MatrixXd PositionRelative2ParentOfJoint);

    double JointAngle;
    double JointVelocity;
    double JointAcceleration;
    Vector3d PositionInWorldCoordinate;
    MatrixXd AttitudeInWorldCoordinate;
    Vector3d LinearVelocityInWorldCoordinate;
    Vector3d AngularVelocityInWorldCoordinate;

private:
    QString _name;
    int _ID ;
    int _sisterID;
    int _childID;
    int _motherID;
    double _mass;
    MatrixXd _momentumOfInertia;
    Vector3d _jointAxisVectorLocal;
    Vector3d _jointPositionRelative2Parent;
    Vector3d _centerOfMassLocal;
};

#endif // LINK_H
