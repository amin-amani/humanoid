#include "robot.h"
//=================================================================================================
Robot::Robot(QObject *parent, int argc, char **argv)
{
    _rosNode=new QNode(argc,argv);
    if(!_rosNode->Init())exit(0);

    for(int ii=0; ii<12 ; ii++)
   {
      _motorPosition.append(0);
   }

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
       //   _initialTimer.start(2000);

         return;







}

//=================================================================================================

void Robot::Initialize()
{
//qDebug()<<"z check...";
//if(!Epos4.CheckZynq())return;
//qDebug()<<"init...";
if(Epos4.Init(2)!=OK)return;
if(! ReadAllInitialPositions())
{return; }
_initialTimer.stop();
qDebug()<<"init...OK";

connect(&timer,SIGNAL(timeout()),this,SLOT(Timeout()));
Initialized=true;
Epos4.StartFeedBack();
timer.start(5);


}
//=================================================================================================
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
 if(Epos4.ReadAllRegisters(0x60e4,1,1,results,10)!=OK)//rerad all incremental positions
 {
     qDebug()<<"Read Inc error";
return false;
 }

 for(int i=0;i<results.count();i++)
 {
     CurrentIncPositions[i]=results[i];
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
//=================================================================================================
void Robot::StatusCheck()
{
    qDebug()<<"init :"<<Initialized;
}

//=================================================================================================
void Robot::NewjointDataReceived()
{
  //  qDebug()<<"get new data..."<<_rosNode->JointsData.data.at(0);
    QList<int> _motorPosition;
for(int ii=0; ii<12 ; ii++)
{
    _motorPosition.append(_rosNode->JointsData.data.at(ii));
}

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
    _rosNode->ActualPositions[i+1]=(positions[i]-offset[i])*ratio[i]*2*M_PI/4096;
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

    for(int ii=0; ii<12 ; ii++)
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
   _hommingTimer.start(5);

}

//=================================================================================================
void Robot::Timeout()
{
    // QList<int> _motorPosition;
    //qDebug()<<"Tick";


//    if(dir)pos++;
//    else pos--;
//    if(pos>1000){dir=!dir;}
//    if(pos<-1000){dir=!dir;}
   // qDebug()<<pos;

   // _motorPosition.clear();
     for(int ii=0; ii<12 ; ii++)
     {
         _motorPosition[ii];
     }

     Epos4.SetAllPositionCST(_motorPosition,12);
}
//=================================================================================================

void Robot::HommingLoop()
{

int j=0;
double max=20.0;
double kp=1000*2*M_PI/4096;//230400/2/3.14/2;

       j=HomeOrder[currentHomeIndex];

 if(abs((offset[j]-CurrentAbsPositions[j])*kp*Direction[j])<=max){_motorPosition[j]+= ((offset[j]-CurrentAbsPositions[j]))*kp*Direction[j];}
     if(((offset[j]-CurrentAbsPositions[j])*kp*Direction[j])>max){_motorPosition[j]+=(max);}
     if(((offset[j]-CurrentAbsPositions[j])*kp*Direction[j])<-max){_motorPosition[j]-=(max);}

     qDebug()<<"offset"<<offset[j]<<"position["<<j<<"]="<<CurrentAbsPositions[j]<<"command"<<_motorPosition[j];
     //// here we receive home
     if(offset[j]-CurrentAbsPositions[j]==0){

      currentHomeIndex++;

     }
     if(currentHomeIndex>11){
         qDebug()<<"Home Finished!";
         _hommingTimer.stop();
         ResetAllNodes();

     }
     ///
 ///


Epos4.SetAllPositionCST(_motorPosition,12);



}
//=================================================================================================
void Robot::CleanAndExit()
{

    exit(0);

}
//=================================================================================================
