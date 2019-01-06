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

    const double offset[12]={ -44 ,-47, 166, 216, 319, -127, 1 ,200, -149, 9, -976, 143};
    const double ratio[12]={ 1,-1,1,-1,1,1,-1,1,-1,1,1,-1};
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
