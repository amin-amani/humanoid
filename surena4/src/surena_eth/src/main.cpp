#include "ros/ros.h"
#include "std_msgs/String.h"

#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <QEventLoop>
#include <qdebug.h>
#include <QTimer>
#include "surena_eth/active_csp.h"
#include "surena_eth/reset_node.h"
#include "surena_eth/home.h"
#include <QApplication>
#include <std_msgs/Int32MultiArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include "visualize.h"
#include "VisualizeControl.h"
#include <tf/transform_datatypes.h>

#include "qnode.h"
#include "robot.h"




int _dataIndex=0;
QList<int> _motorPosition;
// visualization_msgs::Marker marker;
//   ros::Publisher marker_pub ;

  // VisualizeControl rviz;

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

 bool ActiveCSP(surena_eth::active_csp::Request  &req,surena_eth::active_csp::Response &res)
{

ROS_INFO("request: ipm init");
//DisplayText("CSP ACTIVE",3,"/base_link",0,0,2.3);

}
/// ==========================================================================================
/// \brief Home
/// \param req
/// \param res
/// \return
/// ==========================================================================================
bool Home(surena_eth::home::Request  &req,surena_eth::home::Response &res)
{
    ROS_INFO("request: ipm init");
}

// ==========================================================================================
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

UpdateRobotModel(_motorPosition);

}
// ==========================================================================================

void StateFeedback(const sensor_msgs::JointState & msg)
{

    qDebug()<<QString::fromStdString(msg.name[0])<<" "<<QString::number( msg.position[0]);

}
// ==========================================================================================

void FeedBackReceived(QList<int16_t> ft, QList<int32_t> positions)
{
    qDebug()<<"pos="<<positions[0]<<positions[1]<<positions[2]<<positions[3];
//    QString str="";
//    QList<double> ftResultsLeft,ftResultsRight;
//    for(int i=0;i<6;i++){
//    double result=(-ft[2*i]-OffsetLeft[2*i])*5000;
//    result=result/(65535.0*GainLeft[2*i]*SensitivityLeft[2*i]*LeftExtVoltage);
//    ftResultsLeft.append(result);
//     result=(ft[2*i+1]-OffsetRight[2*i+1])*5000;
//    result=result/(65535.0*GainRight[2*i+1]*SensitivityRight[2*i+1]*RightExtVoltage);
//    ftResultsRight.append(result);
    }

// ==========================================================================================
///QNode *mynode;

int main(int argc, char **argv)
{
    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
   qDebug()<<"start..";
   Robot w(nullptr,argc,argv);
   // w.show();
//   app.connect(&w, SIGNAL(, &app, SLOT(quit()));

   return app.exec();

    //return result;


    //    QApplication a(argc, argv);
   //ros::init(argc, argv, "surena_eth");
//    RobotController surena(argc, argv);
//    ros::NodeHandle nh;
//    //----------------services
//    ros::ServiceServer activeCSPService = nh.advertiseService("ActiveCSP", ActiveCSP);
//    ros::ServiceServer hommingService = nh.advertiseService("Home", Home);

//    //---------------subscribesrs
//    ros::Subscriber sub = nh.subscribe("jointdata/qc", 1000, SendDataToMotors);
//    ros::Subscriber statessub = nh.subscribe("/joint_states", 1000, StateFeedback);
//    //---------------publishers
//    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
//    VisualizeInit(&nh);


//    ros::Rate loop_rate(10);
//float t=0,tt=0;
//  while (ros::ok())
//    {

//        std_msgs::String msg;
//        msg.data = "hello world";

//        chatter_pub.publish(msg);
////DisplayLink();
////     DisplayText(QString::number(t+=.01),1,"/base_link",0,0,2);
////       DisplayText(QString::number(tt+=.05),2,"/base_link",0,0,2.6);
//DisplayError("",1,"LLeg_Hip_Thigh_Link",0,0,0);
//DisplayError("",2,"LARM_LINK4",0,0,0);

//        ros::spinOnce();

//        loop_rate.sleep();
//    }
//return a.exec();
//    return 0;
}
