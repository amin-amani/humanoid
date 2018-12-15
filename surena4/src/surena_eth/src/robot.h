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
    bool Initialized=false;
    float CurrentAbsPositions[40];
    int HommingState=0;
    float CurrentIncPositions[40];
    const double offset[12]={ -68 ,-77, 119, 131, 339, -139, 4 ,224, -174, -48, -1004, 141};
    const double ratio[12]={ 1,-1,1,-1,1,1,-1,1,-1,1,1,-1};
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
    void FeedBackReceived(QList<int16_t>ft,QList<int32_t>positionAbs,QList<int32_t>positionInc);
    //=================================================================================================
    void ActiveCSP();
    //=================================================================================================
    void ResetAllNodes();
    //=================================================================================================
};

#endif // ROBOT_H
