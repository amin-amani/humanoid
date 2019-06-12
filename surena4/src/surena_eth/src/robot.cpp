#include "robot.h"
//#include"QsLog/QsLogDisableForThisFile.h"
//=================================================================================================
Robot::Robot(QObject *parent, int argc, char **argv)
{

    _rosNode=new QNode(argc,argv);
    if(!_rosNode->Init())
    {
    QLOG_FATAL()<<"Ros node init error please check ros master!";
        exit(0);
    }
    QLOG_TRACE()<<"Ros node successfuly created";
    _rosNode->RobotStatus="Initialize";
//if(Epos4.Init(2)==OK){qDebug()<<"ok";return;}
 //   connect(&_statusCheckTimer,SIGNAL(timeout()),this,SLOT(StatusCheck()));

//_statusCheckTimer.start(1000);
    pos=0;
    connect(_rosNode,SIGNAL(rosShutdown()),this,SLOT(CleanAndExit()));
    connect(_rosNode,SIGNAL(NewjointDataReceived()),this,SLOT(NewjointDataReceived()));
    connect(_rosNode,SIGNAL(SetActiveCSP(int)),this,SLOT(ActiveCSP(int)));
    connect(_rosNode,SIGNAL(DoResetAllNodes(int)),this,SLOT(ResetAllNodes(int)));
    connect(_rosNode,SIGNAL(DoResetHands()),this,SLOT(ResetHands()));
    connect(_rosNode,SIGNAL(UpdateAllPositions()),this,SLOT(ReadAllInitialPositions()));
    connect(_rosNode,SIGNAL(DoActivateHands()),this,SLOT(ActivateHands()));
    connect(_rosNode,SIGNAL(DoReadError()),this,SLOT(ReadErrors()));
    connect(_rosNode,SIGNAL(SetHome(int)),this,SLOT(Home(int)));
    connect(&Epos4,SIGNAL(FeedBackReceived(QList<int16_t>,QList<int32_t>,QList<int32_t>,QList<uint16_t>,QList<float>)),this,SLOT(FeedBackReceived(QList<int16_t>,QList<int32_t>,QList<int32_t>,QList<uint16_t>,QList<float>)));
    connect(&_initialTimer,SIGNAL(timeout()),this,SLOT(Initialize()));


    _initialTimer.start(2000);

     QLOG_TRACE()<<"Start initialize...";

}
//=================================================================================================
void Robot::Initialize()
{
//qDebug()<<"z check...";
//if(!Epos4.CheckZynq())return;
//qDebug()<<"init...";
//if(Epos4.Init(2)!=OK)
//    return;
//QElapsedTimer t;
//t.start();
//int32_t result=0;
//QList<int32_t> alldata;
//for(int i=0;i<1;i++){
//Epos4.ReadRegister(0x1000,0,1,i,result,20,1);
//alldata.append(result);
//}
//qDebug()<<"elapsed:"<<t.elapsed();
//for(int i=0;i<alldata.length();i++)
//qDebug()<<QString::number( alldata[i],16);
ReadAllInitialPositions();


////Epos4.ReadAllRegisters(0x60e4,1,1,results,10);
//qDebug()<<"------------------------------start";
//int32_t result;
//result=Epos4.ReadRegister(0x1000,0,1,2,10,1);
//(int index,int subIndex,int canID, int devID,int timeout,int trycount)
//for(int i=0;i<4;i++){
//result=   Epos4.ReadRegister(0x60e4,2,i+1,12,10,1);
//qDebug()<<"get val="<<QString::number( result,16)<<result;
//result=   Epos4.ReadRegister(0x60e4,2,i+1,13,10,1);
//qDebug()<<"get val="<<QString::number( result,16)<<result;

//}
//qDebug()<<"get val="<<QString::number( result,16)<<result;
//QList<int32_t> results;
//Epos4.ReadAllRegisters(0x1000,0,1,results,10); //this function socks
//qDebug()<<"------------------------------end";
//if(results.count()>2)qDebug()<<QString::number( results[2],16);
//return;



//qDebug()<<"foot init...OK";
//if(Epos4.HandsInit(2)!=OK)
//{
// return;
//}


//if(! ReadAllInitialPositions())
//{return; }


//_initialTimer.stop();

//   Epos4.ActivePPMPDO(13,1);
//   Epos4.ActivePPMPDO(13,2);
//   Epos4.ActivePPMPDO(13,3);
//   Epos4.ActivePPMPDO(13,4);

//Epos4.ActivePPMPDO(12,1);
//Epos4.ActivePPMPDO(12,2);
//Epos4.ActivePPMPDO(12,3);
//Epos4.ActivePPMPDO(12,4);

qDebug()<<"hands init...OK";
_initialTimer.stop();
connect(&timer,SIGNAL(timeout()),this,SLOT(Timeout()));
Initialized=true;
//Epos4.ActiveJoint(2);
//Epos4.ActiveAllHands();
//Epos4.ActiveCSP(255);
//Epos4.StartFeedBack();
  //timer.start(5);
    _rosNode->RobotStatus="Ready";
     QLOG_TRACE()<<"initialize Completed";
}
//=================================================================================================
bool Robot::ReadAllInitialPositions()
{
    QLOG_TRACE()<<"Read inc and absoulute encoders";
    int32_t result;
    //result=Epos4.ReadRegister(0x1000,0,1,2,10,1);
    //(int index,int subIndex,int canID, int devID,int timeout,int trycount)
    for(int i=0;i<4;i++){
    result=   Epos4.ReadRegister(0x60e4,2,i+1,12,10,1);
    CurrentAbsPositions[12+i]=result;
    _rosNode->ActualPositions[i+12+1]=result;
    result=   Epos4.ReadRegister(0x6064,0,i+1,12,10,1);
    CurrentIncPositions[12+i]=result;
     _rosNode->IncPositions[i+12+1]=result;
    //qDebug()<<"get val="<<QString::number( result,16)<<result;
    result=   Epos4.ReadRegister(0x60e4,2,i+1,13,10,1);//pos act
    CurrentAbsPositions[20+i]=result;
    _rosNode->ActualPositions[i+20+1]=result;
    result=   Epos4.ReadRegister(0x6064,0,i+1,13,10,1);
    CurrentIncPositions[12+i]=result;
    _rosNode->IncPositions[i+20+1]=result;
   // qDebug()<<"get val="<<QString::number( result,16)<<result;

    }

/////////////////// end hands
int numberOfSuccess=0;
    for(int i=0;i<12;i++){
   int32_t result=0;


    result=Epos4.ReadRegister(0x60e4,1,1,i,10,3);
    //{
    CurrentIncPositions[i]=result;
    //  positionInc.append(result);
    numberOfSuccess++;
    qDebug()<<"inc "<<i<<"=" << result;
    _rosNode->IncPositions[i+1]=(CurrentIncPositions[i]);

    }
    //////////////////////
    for(int i=0;i<12;i++){
        int32_t result=0;
   // if(Epos4.ReadRegister(0x60e4,2,1,i,10,3)==OK)
    //{
        result=Epos4.ReadRegister(0x60e4,2,1,i,10,3);
CurrentAbsPositions[i]=result;
    _rosNode->ActualPositions[i+1]=(CurrentAbsPositions[i]-offset[i])*ratio[i]*2*M_PI/8192;
      //  positionInc.append(result);
      numberOfSuccess++;
        qDebug()<<"ABS "<<i<<"=" << result;

    }

    return true;

}
//=================================================================================================
void Robot::WaitMs(int ms)
{
    QEventLoop q;
    QTimer tT;
    tT.setSingleShot(true);
    connect(&tT, SIGNAL(timeout()), &q, SLOT(quit()));
    tT.start(ms); // 5s timeout
    q.exec();
    if(tT.isActive()){
        // download complete
        tT.stop();
    } else {

    }
}
//=================================================================================================
void Robot::StatusCheck()
{
    qDebug()<<"init :"<<Initialized;
}
//=================================================================================================
void Robot::NewjointDataReceived()
{
    // qDebug()<<"get new data..."<<_rosNode->JointsData.data.at(20);
    QList<int> _motorPosition;
for(int ii=0; ii<28 ; ii++)
{
    _motorPosition.append(_rosNode->JointsData.data.at(ii));
}
// motor offset dynamixel
_motorPosition[24]+=3000;
_motorPosition[25]+=(2050-300);
_motorPosition[26]+=2050;
_motorPosition[16]+=2300;
_motorPosition[17]+=2050;
_motorPosition[18]+=2050;
Epos4.SetAllPositionCST(_motorPosition);
}
//=================================================================================================
void  Robot::FeedBackReceived(QList<int16_t> ft, QList<int32_t> positions,QList<int32_t> positionsInc,QList<uint16_t> bump_sensor_list,QList<float>
                              _data_list)
{
    for(int i=0;i<12;i++){
CurrentAbsPositions[i]=positions[i];
CurrentIncPositions[i]=positionsInc[i];
    _rosNode->ActualPositions[i+1]=(positions[i]-offset[i])*ratio[i]*2*M_PI/8192;
    _rosNode->IncPositions[i+1]=positionsInc[i];  //  _rosNode->ActualJointState.position.push_back( _rosNode->ActualPositions[i+1]);
    }
  if(bump_sensor_list.count()==8)
 for(int i=0;i<8;i++){
  _rosNode->BumpSensor[i]= bump_sensor_list[i];
 }
_rosNode->imuSesnsorMsg=Epos4.IMU;
_rosNode->RightFtSensorMessage=Epos4.ForceTorqSensorRight;
_rosNode->LeftFtSensorMessage=Epos4.ForceTorqSensorLeft;

}
//=================================================================================================
void Robot::ActiveCSP(int id)
{
//ReadAllInitialPositions();
    ////////////hand test
    _rosNode->RobotStatus="Motor Activating";
    qDebug()<<"active csp slot..."<<id;
    _rosNode->teststr="OK";
    timer.stop();

//    if(id==255){
//    QThread::msleep(5);
//    Epos4.ActiveAllCSP();
//    QThread::msleep(5);
//    //Epos4.ActiveHand();
//    }
//    else
//    {
    Epos4.ActiveJoint(id);
  //  }


    _rosNode->OperationCompleted(0);
    _rosNode->RobotStatus="Ready";
            }
//=================================================================================================
void Robot::ReadErrors()
{
int32_t result=0;
QString errorstr="";
for(int i=0;i<12;i++){

 errorstr+= "M"+QString::number(i)+"->"+ Epos4.ReadCurrentError(1,i)+" ";

}
//qDebug()<<errorstr;
_rosNode->RobotStatus=errorstr;
_rosNode->OperationCompleted(0); //all good

}
void Robot::ResetHands(void)
{
    try{
   qDebug()<<"Reset hands...";
   //12 15
   //20 //23
   for(int i=12;i<16;i++)
   Epos4.ResetNode(i);
   for(int i=20;i<24;i++)
   Epos4.ResetNode(i);

   _rosNode->OperationCompleted(0);
   _rosNode->RobotStatus="Ready";
      qDebug()<<"operation completed!";
}
    catch(const std::runtime_error e)
    {
        _rosNode->OperationCompleted(-1);
        //  timer.start(5);
        _rosNode->RobotStatus="Ready";
    }
}

void Robot::ActivateHands(void)
{
    _rosNode->RobotStatus="Hand Motor Activating";
    qDebug()<<"activating hands";
    _rosNode->teststr="OK";
    timer.stop();
    for(int i=12;i<16;i++)
        Epos4.ActiveJoint(i);
    for(int i=20;i<24;i++)
        Epos4.ActiveJoint(i);

    _rosNode->OperationCompleted(0);
    _rosNode->RobotStatus="Ready";


//    try{
//   qDebug()<<"Activate hands...";
//   //12 15
//   //20 //23
//   for(int i=12;i<15;i++)
//   Epos4.ActiveJoint(i);
//   for(int i=20;i<23;i++)
//   Epos4.ActiveJoint(i);

//   _rosNode->OperationCompleted(0);
//   _rosNode->RobotStatus="Ready";
//      qDebug()<<"operation completed!";
//}
//    catch(const std::runtime_error e)
//    {
//        _rosNode->OperationCompleted(-1);
//        //  timer.start(5);
//        _rosNode->RobotStatus="Ready";
//    }


}

//=================================================================================================
void Robot::ResetAllNodes(int id)
{
    try{
    _hommingTimer.stop();
    _rosNode->RobotStatus="Reseting Nodes";
    qDebug()<<"Reset all slot...";
    _rosNode->teststr="OK";
Epos4.ResetNode(id);

//}
//    for(int i=0;i<12;i++){
//        //checking status word
//          int32_t val;
//          if( OK!=Epos4.ReadRegister(0x6041,0,1,i,val,100,10))
//          {
//                //cant read status word
//              qDebug()<<"cant read status word in id="<<i;
//                  _rosNode->OperationCompleted(-1);
//                  return ;
//          }
//          //status word value is unexpected
//          if(val!=0x1052da0)
//          {
//            qDebug()<<QString::number( val,16);
//              _rosNode->OperationCompleted(-1);
//              return ;
//          }

//    }

    _rosNode->OperationCompleted(0);
    //  timer.start(5);
    _rosNode->RobotStatus="Ready";
    // Epos4.StartFeedBack();
}
    catch(const std::runtime_error e)
    {
        _rosNode->OperationCompleted(-1);
        //  timer.start(5);
        _rosNode->RobotStatus="Ready";
    }
}
//=================================================================================================
void Robot::Home(int id)
{

        _rosNode->RobotStatus="Homming";
    if(! ReadAllInitialPositions())
    {
            QLOG_ERROR()<<"Home error ->read positions error";

            _rosNode->OperationCompleted(-1);
        return;
    }
  _motorPosition.clear();
    for(int ii=0; ii<12 ; ii++)
   {
      _motorPosition.append( CurrentIncPositions[ii]);
   }
    for(int ii=0; ii<16 ; ii++)
   {
      _motorPosition.append(0);
   }
    currentHomeIndex=0;
    timer.stop();
   //void PIDController::Init(double dt, double max, double min, double Kp, double Kd, double Ki)
    pid.Init(5,10,-10,1.5,0,0.0);
    _hommingTimer.disconnect();
    connect(&_hommingTimer,SIGNAL(timeout()),this,SLOT(HommingLoop()));
    Epos4.ActiveCSP(255);
    //Epos4.ActiveHand();
    _hommingTimer.start(5);
    _rosNode->RobotStatus="Ready";

}
//=================================================================================================
void Robot::Timeout()
{
     QList<int> _motorPosition;
   // qDebug()<<"Tick";


    if(dir)pos+=1;
    else pos-=1;
    if(pos>300){dir=!dir;}
    if(pos<-300){dir=!dir;}
    //qDebug()<<pos;

    //_motorPosition.clear();


    for(int ii=0; ii<28 ; ii++)
     {
         _motorPosition.append( pos);
     }
     Epos4.SetAllPositionCST(_motorPosition);
}
//=================================================================================================
void Robot::HommingLoop()
{
int j=0;
double max=20.0;
double kp=2000*2*M_PI/8192;//230400/2/3.14/2;
  j=HomeOrder[currentHomeIndex];
 if(abs((offset[j]-CurrentAbsPositions[j])*kp*Direction[j])<=max){_motorPosition[j]+= ((offset[j]-CurrentAbsPositions[j]))*kp*Direction[j];}
     if(((offset[j]-CurrentAbsPositions[j])*kp*Direction[j])>max){_motorPosition[j]+=(max);}
     if(((offset[j]-CurrentAbsPositions[j])*kp*Direction[j])<-max){_motorPosition[j]-=(max);}

     qDebug()<<"offset"<<offset[j]<<"position["<<j<<"]="<<CurrentAbsPositions[j]<<"command"<<_motorPosition[j];
     //// here we receive home
     if(offset[j]-CurrentAbsPositions[j]==0){

      currentHomeIndex++;

     }
       //  qDebug()<<"offset";
     if(currentHomeIndex>11){
         qDebug()<<"Home Finished!";
         _hommingTimer.stop();
             _rosNode->OperationCompleted(0);
//         ResetAllNodes();

     }
     ///
 ///

_motorPosition[12]=0;
_motorPosition[13]=0;
_motorPosition[14]=0;
_motorPosition[15]=0;
_motorPosition[20]=0;
_motorPosition[21]=0;
_motorPosition[22]=0;
_motorPosition[23]=0;
_motorPosition[24]=0;
Epos4.SetAllPositionCST(_motorPosition);
//qDebug()<<"addad"<<_motorPosition.count();

}
//=================================================================================================
void Robot::CleanAndExit()
{

    _hommingTimer.stop();
    _initialTimer.stop();
    timer.stop();
//Epos4.~Epos();
QLOG_DEBUG()<<"Clean and exit\n";
WaitMs(2000);
exit(0);

}
//=================================================================================================
