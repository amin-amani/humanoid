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

// =================================note=========================================================
//echo "source  /home/amin/Desktop/catkin_ws/devel/setup.sh" >> ~/.bashrc
//source  /home/amin/Desktop/catkin_ws/devel/setup.sh
//========================== Global variables

 Epose epos;
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
/// \brief main
/// \param argc
/// \param argv
/// \return
/// ==========================================================================================
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
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
 ros::Subscriber sub = nh.subscribe("join_data/qc", 1000, SendDataToMotors);
 ros::ServiceServer activeCSPService = nh.advertiseService("ActiveCSP", ActiveCSP);
 ros::ServiceServer resetNodeService = nh.advertiseService("ResetNode", ResetNode);



    ros::spin();
return 0;
   // return a.exec();
}
