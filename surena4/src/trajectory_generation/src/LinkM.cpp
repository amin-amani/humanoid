#include "LinkM.h"

QString LinkM::GetName()
{
    return _name;
}

void LinkM::SetName(QString name)
{
    _name=name;

}

int LinkM::GetID()
{
    return _ID;
}

void LinkM::SetID(int ID)
{
    _ID=ID;
}

int LinkM::GetSisterID()
{
    return _sisterID;
}

void LinkM::SetSisterID(int SisterID)
{
    _sisterID=SisterID;
}

int LinkM::GetMotherID()
{
    return _motherID;
}

void LinkM::SetMother(int MotherID)
{
    _motherID=MotherID;
}

int LinkM::GetChildID()
{
    return _childID;
}

void LinkM::SetChildID(int childID)
{
    _childID=childID;
}

double LinkM::GetMass()
{
    return _mass;
}

void LinkM::SetMass(double Mass)
{
    _mass=Mass;
}

MatrixXd LinkM::GetMomentumOfInertia()
{
    return _momentumOfInertia;
}

void LinkM::SetMomentumOfInertia(MatrixXd momentumOfInertia)
{
    _momentumOfInertia=momentumOfInertia;
}

Vector3d LinkM::GetJointAxisVectorLocal()
{
    return _jointAxisVectorLocal;
}

void LinkM::SetJointAxisVectorLocal(Vector3d jointAxisVectorLocal)
{
    _jointAxisVectorLocal=jointAxisVectorLocal;
}

Vector3d LinkM::GetJointPositionRelative2Parent()
{
    return _jointPositionRelative2Parent;
}

void LinkM::SetJointPositionRelative2Parent(Vector3d jointPositionRelative2Parent)
{
    _jointPositionRelative2Parent=jointPositionRelative2Parent;
}

Vector3d LinkM::GetCenterOfMassLocal()
{
    return _centerOfMassLocal;
}

void LinkM::SetCenterOfMassLocal(Vector3d centerOfMassLocal)
{
    _centerOfMassLocal=centerOfMassLocal;
}


LinkM::LinkM(QString name, int IDofLink, int IDofSyster, int IDofChild, int IDofMother, double AngleOfJoint, MatrixXd LocalAxisVectorOfJoint, MatrixXd PositionRelative2ParentOfJoint)
{
     JointAngle= AngleOfJoint;
    _name=name;
    _ID=IDofLink;
    _sisterID=IDofSyster;
    _childID=IDofChild;
    _motherID=IDofMother;
    _jointAxisVectorLocal=LocalAxisVectorOfJoint ;
    _jointPositionRelative2Parent=PositionRelative2ParentOfJoint;
}


