#include "mainwindow.h"
#include <QApplication>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include<std_msgs/Int32MultiArray.h>
#include<std_msgs/Int32.h>
#include <stdlib.h>
#include "epose.h"
#include"surena_usb/active_csp.h"
#include "surena_usb/reset_node.h"
#include "surena_usb/home.h"

// =================================note=========================================================
//echo "source  /home/amin/Desktop/catkin_ws/devel/setup.sh" >> ~/.bashrc
//source  /home/amin/Desktop/catkin_ws/devel/setup.sh
//========================== Global variables

 Epose epos;

 int MotorOffset[12];//={-3645,2557,-2718,3659,66,-3242,-3085,1985,-2146,-2443,454,131};
 double GearRatio[12];//={28.125,28.125,14.0625,22.5,28.125,28.125,14.0625,-22.5,33.75,33.75,33.75,33.75};
 QMap<QString,int> MotorNames;


 void WaitMs(int ms)
 {
   QEventLoop q;
   QTimer tT;
   tT.setSingleShot(true);
   QObject::connect(&tT, SIGNAL(timeout()), &q, SLOT(quit()));
   tT.start(ms); // 5s timeout
   q.exec();
   if(tT.isActive()){
     // download complete
     tT.stop();
   } else {

   }


 }
 //====================================================================================

 /// ==========================================================================================
 /// \brief ActiveCSP
 /// \param req
 /// \param res
 /// \return
  /// ==========================================================================================

bool ActiveCSP(surena_usb::active_csp::Request  &req,surena_usb::active_csp::Response &res)
{

ROS_INFO("request: ipm init");
res.result=epos.ActiveCSP(req.nodeID);

}
/// ==========================================================================================
/// \brief Home
/// \param req
/// \param res
/// \return
/// ==========================================================================================
bool Home(surena_usb::home::Request  &req,surena_usb::home::Response &res)
{
    int32_t incEncoder[12];
    int32_t AbsEncoder[12];
    int32_t diffs[12];

ROS_INFO("request: home");
if(true!=epos.ActivePPM(1,255))
{
    qDebug()<<"ppm init error";

    return false ;
}
WaitMs(1000);
qDebug()<<"read all  positions";
for(int i=0;i<12;i++)
{
    //if(i==7)continue;///uncomment to enable
    if( !epos.GetIncPosition(i,incEncoder[i]))//read inc enc
    {
        qDebug()<<"error read enc from:"<<i;return false;

    }
    qDebug()<<"inc value:"<<incEncoder[i];
    if( !epos.GetAbsPosition(i,AbsEncoder[i]))//read abs enc
    {
        qDebug()<<"error read abs from:"<<i;return false;

    }
    qDebug()<<"abs value:"<<AbsEncoder[i]<< " target:"<<MotorOffset[i];

    diffs[i]=(incEncoder[i]+(MotorOffset[i]-AbsEncoder[i])*GearRatio[i]);
    qDebug()<<"id:"<<i<<" diff="<<diffs[i]<<" Gear:"<<GearRatio[i];



}
///========================moves all motors

        WaitMs(100);
        epos.SetPosition(1,req.nodeID,diffs[req.nodeID],50);
        WaitMs(100);
        epos.SetPosition(1,req.nodeID,diffs[req.nodeID],50);
        WaitMs(100);

    for(int i=0;i<4;i++){
    WaitMs(100);
    epos.SetPosition(1,i,diffs[i],50);
    WaitMs(100);
    epos.SetPosition(1,i,diffs[i],50);
    WaitMs(100);
}
for(int i=8;i<10;i++){
    WaitMs(100);
    epos.SetPosition(1,i,diffs[i],50);
    WaitMs(100);
    epos.SetPosition(1,i,diffs[i],50);
    WaitMs(100);
}
for(int i=5;i>3;i--){
    WaitMs(100);
    epos.SetPosition(1,i,diffs[i],50);
    WaitMs(100);
    epos.SetPosition(1,i,diffs[i],50);
    WaitMs(100);
}
for(int i=6;i<8;i++){
    WaitMs(100);
    epos.SetPosition(1,i,diffs[i],50);
    WaitMs(100);
    epos.SetPosition(1,i,diffs[i],50);
    WaitMs(100);
}
for(int i=10;i<12;i++){
    WaitMs(100);
    epos.SetPosition(1,i,diffs[i],50);
    WaitMs(100);
    epos.SetPosition(1,i,diffs[i],50);
    WaitMs(100);
}
///========================moves all motors
qDebug()<<"wait move complete";
WaitMs(2000);
for(int i=0;i<12;i++)
{
    //if(i==7)continue;
    if( !epos.GetAbsPosition(i,AbsEncoder[i]))//read inc enc
    {
        qDebug()<<"error read enc from:"<<i;return false;

    }

    qDebug()<<"id:"<<i<<" pos abs:"<<AbsEncoder[i]<<" "<<MotorOffset[i]<<" diff="<<MotorOffset[i]-AbsEncoder[i];
}


}
/// ==========================================================================================
/// \brief ResetNode
/// \param req
/// \param res
/// \return
/// ==========================================================================================
bool ResetNode(surena_usb::reset_node::Request  &req,surena_usb::reset_node::Response &res)
{
epos.ResetNode(req.nodeID);
  ROS_INFO("request: reset node");
res.result=1;

}
/// ==========================================================================================
void SendDataToMotors(const std_msgs::Int32MultiArray & msg)
{
     QList<int> _motorPosition;
  _motorPosition.append(msg.data[0]);
  _motorPosition.append(msg.data[1]);
  _motorPosition.append(msg.data[2]);
  _motorPosition.append(msg.data[3]);
  _motorPosition.append(msg.data[4]);
  _motorPosition.append(msg.data[5]);
  _motorPosition.append(msg.data[6]);
  _motorPosition.append(msg.data[7]);
  _motorPosition.append(msg.data[8]);
  _motorPosition.append(msg.data[9]);
  _motorPosition.append(msg.data[10]);
  _motorPosition.append(msg.data[11]);
   epos.SetPositionIPM(_motorPosition);
  ROS_INFO("I heard:[%d]", (int)msg.data[2]);
 //  ROS_INFO("I heard");
}
/// ==========================================================================================
void Init(){

    ///init robot motors map
        MotorNames.insert("R1",0);
        MotorNames.insert("R2",1);
        MotorNames.insert("R3",2);
        MotorNames.insert("R4",3);
        MotorNames.insert("R5",9);
        MotorNames.insert("R6",8);

        MotorNames.insert("L1",5);
        MotorNames.insert("L2",4);
        MotorNames.insert("L3",6);
        MotorNames.insert("L4",7);
        MotorNames.insert("L5",10);
        MotorNames.insert("L6",11);
        MotorNames.insert("ALL",255);

        GearRatio[MotorNames["R1"]]=56.25;
        GearRatio[MotorNames["R2"]]=56.25;
        GearRatio[MotorNames["R3"]]=28.125;
        GearRatio[MotorNames["R4"]]=45;
        GearRatio[MotorNames["R5"]]=67.5;
        GearRatio[MotorNames["R6"]]=67.5;
        GearRatio[MotorNames["L1"]]=56.25;
        GearRatio[MotorNames["L2"]]=56.25;
        GearRatio[MotorNames["L3"]]=28.125;
        GearRatio[MotorNames["L4"]]=45;
        GearRatio[MotorNames["L5"]]=67.5;
        GearRatio[MotorNames["L6"]]=67.5;


        MotorOffset[MotorNames["R1"]]=-1384;
        MotorOffset[MotorNames["R2"]]=810;
        MotorOffset[MotorNames["R3"]]=-1352;
        MotorOffset[MotorNames["R4"]]=-1169;//-482;
        MotorOffset[MotorNames["R5"]]=-673;//-1393;
        MotorOffset[MotorNames["R6"]]=-1042;
        MotorOffset[MotorNames["L1"]]=1607;
        MotorOffset[MotorNames["L2"]]=1488;
        MotorOffset[MotorNames["L3"]]=-1548;
        MotorOffset[MotorNames["L4"]]=-523;
        MotorOffset[MotorNames["L5"]]=-387;//1683;
        MotorOffset[MotorNames["L6"]]=-1164;

}
/// ==========================================================================================

/// ==========================================================================================
/// \brief main
/// \param argc
/// \param argv
/// \return
/// ==========================================================================================
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Init();
    //Initializes ROS, and sets up a node
    ros::init(argc, argv, "receiver_node");
    ros::NodeHandle nh;

 if(epos.Init()==true)// .Init(0xc251,0x1c03,(HANDLE)winId()))
{
  ROS_INFO("usb conneted..");
 }
   else
{
   ROS_INFO("usb conneted error!");
 }
 ros::Subscriber sub = nh.subscribe("jointdata/qc", 1000, SendDataToMotors);
 ros::ServiceServer activeCSPService = nh.advertiseService("ActiveCSP", ActiveCSP);
 ros::ServiceServer resetNodeService = nh.advertiseService("ResetNode", ResetNode);
 ros::ServiceServer homeService = nh.advertiseService("Home", Home);


    ros::spin();
return 0;
   // return a.exec();
}
