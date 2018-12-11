#include "robot.h"
//=================================================================================================
Robot::Robot(QObject *parent, int argc, char **argv)
{
    _rosNode=new QNode(argc,argv);
    if(!_rosNode->init())exit(0);

        if(Epos4.Init()==OK)
          //  if(true)
             {
            qDebug()<<"Device connected.";
            connect(&Epos4,SIGNAL(FeedBackReceived(QList<int16_t>,QList<int32_t>,QList<int32_t>)),this,SLOT(FeedBackReceived(QList<int16_t>,QList<int32_t>,QList<int32_t>)));
            connect(&timer,SIGNAL(timeout()),this,SLOT(Timeout()));


            Epos4.StartFeedBack();
            timer.start(5);
             }
             else
             {
            qDebug()<<"Device does not exist!";

             }


    connect(_rosNode,SIGNAL(rosShutdown()),this,SLOT(CleanAndExit()));
    connect(_rosNode,SIGNAL(NewjointDataReceived()),this,SLOT(NewjointDataReceived()));
    connect(_rosNode,SIGNAL(SetActiveCSP()),this,SLOT(ActiveCSP()));
    connect(_rosNode,SIGNAL(DoResetAllNodes()),this,SLOT(ResetAllNodes()));
      connect(_rosNode,SIGNAL(SetHome()),this,SLOT(Home()));

    pos=0;
}
//=================================================================================================
void Robot::NewjointDataReceived()
{
    qDebug()<<"get new data..."<<_rosNode->JointsData.data.at(0);
}
//=================================================================================================
void Robot::FeedBackReceived(QList<int16_t>ft,QList<int32_t>positionAbs,QList<int32_t>positionInc)
{


//_rosNode->ActualJointState.position.clear();
//_rosNode->IncJointState.position.clear();
    for(int i=0;i<12;i++){
CurrentAbsPositions[i]=positionAbs[i];
CurrentIncPositions[i]=positionInc[i];
    _rosNode->ActualPositions[i+1]=(positionAbs[i]-offset[i])*ratio[i]*2*M_PI/4096;
    _rosNode->IncPositions[i+1]=positionInc[i];
  //  _rosNode->ActualJointState.position.push_back( _rosNode->ActualPositions[i+1]);
  //  _rosNode->IncJointState.position.push_back(positionInc[i]);
    }

//    qDebug()<<"pose="   <<positionAbs[0]<<positionAbs[1]<<positionAbs[2]<<positionAbs[3]
//                        <<positionAbs[4]<<positionAbs[5]<<positionAbs[6]<<positionAbs[7]
//                        <<positionAbs[8]<<positionAbs[9]<<positionAbs[10]<<positionAbs[11];

////    qDebug()<<"pose="<<positionInc[0]<<positionInc[1]<<positionInc[2]<<positionInc[3]
//                <<positionInc[4]<<positionInc[5]<<positionInc[6]<<positionInc[7]
//                  <<positionInc[8]<<positionInc[9]<<positionInc[10]<<positionInc[11];

}
//=================================================================================================
void Robot::ActiveCSP()
{
        qDebug()<<"active csp slot...";
        _rosNode->teststr="OK";

            timer.stop();
   QThread::msleep(5);
       Epos4.ActiveAllCSP();
            }
//=================================================================================================
void Robot::ResetAllNodes()
{
        qDebug()<<"Reset all slot...";
        _rosNode->teststr="OK";
        for(int i=0;i<12;i++)
        { Epos4.ResetNode(i);
            QThread::msleep(5);
        }

            timer.start(5);

          Epos4.StartFeedBack();

}
//=================================================================================================
void Robot::Home()
{
   timer.stop();
   pid.Init(5,10,-10,.1,0,0);
   _hommingTimer.disconnect();
  connect(&_hommingTimer,SIGNAL(timeout()),this,SLOT(HommingLoop()));
   Epos4.ActiveAllCSP();
   _hommingTimer.start(5);

}
//=================================================================================================
void Robot::Timeout()
{
    //qDebug()<<"Tick";
         QList<int> _motorPosition;

//    if(dir)pos++;
//    else pos--;
//    if(pos>1000){dir=!dir;}
//    if(pos<-1000){dir=!dir;}
   // qDebug()<<pos;

   // _motorPosition.clear();
     for(int ii=0; ii<12 ; ii++)
     {
         _motorPosition.append(0);
     }

     Epos4.SetAllPositionCST(_motorPosition,12);
}
//=================================================================================================

void Robot::HommingLoop()
{
  // qDebug()<<"Homming";
   QList<int> _motorPosition;
   float error =CurrentAbsPositions[2]-offset[2];


 for(int ii=0; ii<12 ; ii++)
{
   _motorPosition.append(0);
}

 _motorPosition[2]=(int)pid.Calculate(150,CurrentAbsPositions[2])+CurrentIncPositions[2];
  qDebug()<<"sp=150 "<<"current abs="<<CurrentAbsPositions[2]<<"pid="<<_motorPosition[2]<<" current inc="<<CurrentIncPositions[2];

Epos4.SetAllPositionCST(_motorPosition,12);



}
//=================================================================================================
void Robot::CleanAndExit()
{

    exit(0);

}
//=================================================================================================
