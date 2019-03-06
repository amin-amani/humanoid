#ifndef ROBOT_H
#define ROBOT_H

#include <QObject>
#include <QDebug>
#include <QTimer>

#include "qnode.h"
#include "epos.h"
#include "pingmodel.h"
#include "pidcontroller.h"

class Robot : public QObject
{
    Q_OBJECT
    Epos Epos4;
    QNode *_rosNode;
    QTimer timer;
    QTimer _hommingTimer;
    QTimer _initialTimer;
    QTimer _statusCheckTimer;
     QList<int> _motorPosition;
    bool Initialized=false;
    float CurrentAbsPositions[40];
    int HommingState=0;
    float CurrentIncPositions[40];

//    const double offset[12]={ -95 ,-151, 300, 394, 683, -277, -3 ,419, -275, 580, 732, 287};
//    const double offset[12]={ -93 ,-136, 294, 454, 687, -288, -3 ,332, -263, 604, 765, 352};
//    const double offset[12]={ -78 ,-99, 325, 449, 636, -274, -12 ,366, -264, 583, 741, 353};
//    const double offset[12]={ -1314 ,-123+42, 326, 449, 666+42, -273, -13 ,365, -265, 586, 744, 353};//42 is becouse of offset behind ankle pitch mechanism
    const double offset[12]={ -1304, -86, 315, 433, 614, -266, -33, 374, -277, 593, 748, 339};

    const double ratio[12]={ 1,-1,1,-1,1,1,-1,1,-1,-1,-1,-1};
    //const double Direction[12]={1,-1,1,-1,1,1,-1,1,-1,1,1,-1};
        const double Direction[12]={1,1,1,1,1,1,1,1,1,1,1,1};
    const int HomeOrder[12]={0,1,2,3,8,9,5,4,6,7,10,11};
    int currentHomeIndex=0;

    int pos;
    bool dir=false;
    PIDController pid;
public:

    //=================================================================================================
    explicit Robot(QObject *parent ,int argc, char **argv);
    //=================================================================================================

    bool ReadAllInitialPositions();

signals:


public slots:
        void ReadErrors();
    void StatusCheck();
    //=================================================================================================
    void Initialize();
    //=================================================================================================
    void Home();
    //=================================================================================================
    void CleanAndExit();
    //=================================================================================================
    void NewjointDataReceived( );
    //=================================================================================================
    void Timeout();
    //=================================================================================================
    void HommingLoop();
    //=================================================================================================
  //  void FeedBackReceived(QList<int16_t>ft,QList<int32_t>positionAbs,QList<int32_t>positionInc);
   // void FeedBackReceived(QList<int16_t> ft, QList<int32_t> positions, QList<int32_t> positionsInc, QList<int16_t> bump_sensor_list, QList<float> imu_data_list);
    void  FeedBackReceived(QList<int16_t> ft, QList<int32_t> positions,QList<int32_t> positionsInc,QList<uint16_t> bump_sensor_list,QList<float> imu_data_list);

    //=================================================================================================
    void ActiveCSP();
    //=================================================================================================
    void ResetAllNodes();
    //=================================================================================================
};

#endif // ROBOT_H
