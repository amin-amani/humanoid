#ifndef QCGENERATOR_H
#define QCGENERATOR_H

#include "std_msgs/String.h"
#include"Eigen/Dense"
#include <vector>
#include <iostream>
#include <QString>
#include <QList>
#include "Robot.h"
//#include"TaskSpace.h"
#include"taskspaceoffline.h"
#include <qmath.h>
#include <cstring>
#include<qdebug.h>
#include <Eigen/Geometry>
#include <cstdlib>
#include "LinkM.h"
#include "Eigen/eiquadprog.h"
#include "Eigen/Core"
#include "Eigen/Cholesky"
#include "Eigen/LU"
#include<std_msgs/Int32MultiArray.h>
#include<std_msgs/Float32MultiArray.h>
#include<math.h>
#include<sensor_msgs/Imu.h>
#include<std_msgs/Float64.h>
#include "pidcontroller.h"
#include<rosgraph_msgs/Clock.h>
#include <std_msgs/Empty.h>
#include "std_srvs/Empty.h"


class QCgenerator
{
public:
    QCgenerator();
    vector<int> data2qc(QList<LinkM> links, vector<double> cntrl);
    vector<int> ctrldata2qc(vector<double> cntrl);
    vector<int> trajdata2qc(QList<LinkM> links);
};

#endif // QCGENERATOR_H
