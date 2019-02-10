#include "robot.h"
//=================================================================================================
Robot::Robot(QObject *parent, int argc, char **argv)
{
    for(int ii=0; ii<12+8+8 ; ii++)
   {
      _motorPosition.append(100);
   }
    _rosNode=new QNode(argc,argv);
    if(!_rosNode->Init())exit(0);


//if(Epos4.Init(2)==OK){qDebug()<<"ok";return;}
 //   connect(&_statusCheckTimer,SIGNAL(timeout()),this,SLOT(StatusCheck()));

//_statusCheckTimer.start(1000);


    pos=0;
    connect(_rosNode,SIGNAL(rosShutdown()),this,SLOT(CleanAndExit()));
    connect(_rosNode,SIGNAL(NewjointDataReceived()),this,SLOT(NewjointDataReceived()));
    connect(_rosNode,SIGNAL(SetActiveCSP()),this,SLOT(ActiveCSP()));
    connect(_rosNode,SIGNAL(DoResetAllNodes()),this,SLOT(ResetAllNodes()));
    connect(_rosNode,SIGNAL(SetHome()),this,SLOT(Home()));
    connect(&Epos4,SIGNAL(FeedBackReceived(QList<int16_t>,QList<int32_t>,QList<int32_t>,QList<uint16_t>,QList<float>)),this,SLOT(FeedBackReceived(QList<int16_t>,QList<int32_t>,QList<int32_t>,QList<uint16_t>,QList<float>)));



    connect(&_initialTimer,SIGNAL(timeout()),this,SLOT(Initialize()));
          _initialTimer.start(2000);

         return;







}

//=================================================================================================

void Robot::Initialize()
{
//qDebug()<<"z check...";
//if(!Epos4.CheckZynq())return;
//qDebug()<<"init...";
//if(Epos4.Init(2)!=OK)
//    return;

//qDebug()<<"foot init...OK";
//if(Epos4.HandsInit(2)!=OK)
//{
// return;
//}



if(! ReadAllInitialPositions())
{return; }
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
//Epos4.StartFeedBack();
//timer.start(5);


}
//=================================================================================================
/*
bool Robot::ReadAllInitialPositions()
{
    int numberOfSuccess=0;
//    for(int i=0;i<12;i++){
//        int32_t result=0;
//    if(Epos4.ReadRegister(0x60e4,1,1,i,result,10,3)==OK)
//    {
//CurrentIncPositions[i]=result;
//      //  positionInc.append(result);
//      numberOfSuccess++;
//        //qDebug()<<"OK " << result;
//    }
//    else
//    {
//     qDebug()<<"error num=" << i;
//    }

//    }
 QList<int32_t> results;
 if(Epos4.ReadAllRegisters(0x60e4,1,1,results,20)!=OK)//rerad all incremental positions

 //if(Epos4.ReadAllRegisters(0x1000,0,1,results,10)!=OK)
 {
     qDebug()<<"Read Inc error";
return false;
 }

 for(int i=0;i<results.count();i++)
 {
     CurrentIncPositions[i]=results[i];
qDebug()<<"inc"<<i<<"="<<results[i];
     _rosNode->IncPositions[i+1]=results[i];
 }
 results.clear();
 if(Epos4.ReadAllRegisters(0x60e4,2,1,results,10)!=OK)//rerad all Abs positions
 {

     qDebug()<<"Read Abs error";
return false;

 }

 for(int i=0;i<results.count();i++)
 {
     CurrentAbsPositions[i]=results[i];
              _rosNode->ActualPositions[i+1]=(results[i]-offset[i])*ratio[i]*2*M_PI/8192;
              qDebug()<<"abs"<<i<<results[i];

 }

  //////////////
//    for(int i=0;i<12;i++){
//        int32_t result=0;
//    if(Epos4.ReadRegisterValue(0x60e4,2,1,i,result,10)==OK)
//    {
//CurrentAbsPositions[i]=result;
//      //  positionInc.append(result);
//      numberOfSuccess++;
//      //  qDebug()<<"OK " << result;
//    }
//    else
//    {
//     qDebug()<<"error read pos=" << i;
//    }
//                            }


//    if(numberOfSuccess!=24)
//    return false;

    return true;

}
*/
//=================================================================================================
bool Robot::ReadAllInitialPositions()
{
    int numberOfSuccess=0;
    for(int i=0;i<12;i++){
        int32_t result=0;
    if(Epos4.ReadRegister(0x60e4,1,1,i,result,10,3)==OK)
    {
CurrentIncPositions[i]=result;
      //  positionInc.append(result);
      numberOfSuccess++;
          qDebug()<<"inc "<<i<<"=" << result;
              _rosNode->IncPositions[i+1]=(CurrentIncPositions[i]);
    }
    else
    {
     qDebug()<<"error num=" << i;
              return false;
    }

    }
    for(int i=0;i<12;i++){
        int32_t result=0;
    if(Epos4.ReadRegister(0x60e4,2,1,i,result,10,3)==OK)
    {
CurrentAbsPositions[i]=result;
    _rosNode->ActualPositions[i+1]=(CurrentAbsPositions[i]-offset[i])*ratio[i]*2*M_PI/8192;
      //  positionInc.append(result);
      numberOfSuccess++;
        qDebug()<<"ABS "<<i<<"=" << result;
    }
    else
    {
     qDebug()<<"error num=" << i;
         return false;
    }

    }

    return true;

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
// moto offset dynamixel
_motorPosition[24]+=3000;
_motorPosition[25]+=2050;
_motorPosition[26]+=2050;
_motorPosition[16]+=2300;
_motorPosition[17]+=2050;
_motorPosition[18]+=2050;
Epos4.SetAllPositionCST(_motorPosition,12);
}
//=================================================================================================
void  Robot::FeedBackReceived(QList<int16_t> ft, QList<int32_t> positions,QList<int32_t> positionsInc,QList<uint16_t> bump_sensor_list,QList<float> imu_data_list)
{
    //if(bump_sensor_list.count()==8)
//qDebug()<<"bump="<<bump_sensor_list[0]<<bump_sensor_list[1]<<bump_sensor_list[2]<<bump_sensor_list[3]<<bump_sensor_list[4]<<bump_sensor_list[5]<<bump_sensor_list[6]<<bump_sensor_list[7];
//        if(ft.count()==12)
//qDebug()<<"ft="<<ft[0]<<ft[1]<<ft[2]<<ft[3]<<ft[4]<<ft[5]<<ft[6]<<ft[7]<<ft[8]<<ft[9]<<ft[10]<<ft[11];
      // if(imu_data_list.count()>3)
//qDebug()<<"imu ->"<<imu_data_list[0]<<imu_data_list[1]<<imu_data_list[2];
//_rosNode->ActualJointState.position.clear();
//_rosNode->IncJointState.position.clear();
    for(int i=0;i<12;i++){
CurrentAbsPositions[i]=positions[i];
CurrentIncPositions[i]=positionsInc[i];
    _rosNode->ActualPositions[i+1]=(positions[i]-offset[i])*ratio[i]*2*M_PI/8192;
    _rosNode->IncPositions[i+1]=positionsInc[i];  //  _rosNode->ActualJointState.position.push_back( _rosNode->ActualPositions[i+1]);


    //  _rosNode->IncJointState.position.push_back(positionInc[i]);
    }
  if(bump_sensor_list.count()==8)
 for(int i=0;i<8;i++){
  _rosNode->BumpSensor[i]= bump_sensor_list[i];
 }
_rosNode->imuSesnsorMsg=Epos4.IMU;
_rosNode->RightFtSensorMessage=Epos4.ForceTorqSensorRight;
_rosNode->LeftFtSensorMessage=Epos4.ForceTorqSensorLeft;
_rosNode->imuRPYSensorMsg=Epos4.IMURPY;
}
//=================================================================================================
void Robot::ActiveCSP()
{

    ////////////hand test
        qDebug()<<"active csp slot...";
        _rosNode->teststr="OK";

            timer.stop();

       QThread::msleep(5);
       Epos4.ActiveAllCSP();
       QThread::msleep(5);
        Epos4.ActiveHand();
            }
//=================================================================================================
void Robot::ResetAllNodes()
{
        qDebug()<<"Reset all slot...";
        _rosNode->teststr="OK";
        for(int i=0;i<14;i++)
        { Epos4.ResetNode(i);
            QThread::msleep(5);
        }

          //  timer.start(5);

         // Epos4.StartFeedBack();

}
//=================================================================================================
void Robot::Home()
{


    if(! ReadAllInitialPositions())
    {qDebug()<<"error read positions";return; }

    for(int ii=0; ii<24 ; ii++)
   {
      _motorPosition[ii]=CurrentIncPositions[ii];
   }
     currentHomeIndex=0;
   timer.stop();
   //void PIDController::Init(double dt, double max, double min, double Kp, double Kd, double Ki)
   pid.Init(5,10,-10,1.5,0,0.0);
   _hommingTimer.disconnect();
  connect(&_hommingTimer,SIGNAL(timeout()),this,SLOT(HommingLoop()));
   Epos4.ActiveAllCSP();
      Epos4.ActiveHand();
   _hommingTimer.start(5);

}

//=================================================================================================
void Robot::Timeout()
{
    // QList<int> _motorPosition;
//    qDebug()<<"Tick";


//    if(dir)pos+=1;
//    else pos-=1;
//    if(pos>100){dir=!dir;}
//    if(pos<-100){dir=!dir;}
//    qDebug()<<pos;

   // _motorPosition.clear();
     for(int ii=0; ii<24 ; ii++)
     {
         _motorPosition[ii]=0;
     }
     Epos4.SetAllPositionCST(_motorPosition,12);
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
//if(abs(_motorPosition[j])>10000){
//    qDebug()<<"home error!";
//    break;
//}
     qDebug()<<"offset"<<offset[j]<<"position["<<j<<"]="<<CurrentAbsPositions[j]<<"command"<<_motorPosition[j];
     //// here we receive home
     if(offset[j]-CurrentAbsPositions[j]==0){

      currentHomeIndex++;

     }
     if(currentHomeIndex>11){
         qDebug()<<"Home Finished!";
         _hommingTimer.stop();
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
Epos4.SetAllPositionCST(_motorPosition,12);



}
//=================================================================================================
void Robot::CleanAndExit()
{

    exit(0);

}
//=================================================================================================
