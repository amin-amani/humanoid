/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date Sep 2018
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include <iostream>
#include "qnode.h"

using namespace std;

/*****************************************************************************
** Namespaces
*****************************************************************************/


/*****************************************************************************
** Implementation
*****************************************************************************/
QNode::QNode()
{

}
//=============================================================================================================================
QNode::QNode(int argc, char** argv ) :	init_argc(argc),	init_argv(argv)
	{}
//=============================================================================================================================
QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}
//=============================================================================================================================
bool QNode::Init(const std::string &master_url, const std::string &host_url) {
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"my_qt_gui_subscriber");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
   // _chatter_subscriber = n.subscribe("chatter", 1000, &QNode::myCallback, this);
    start();

    return true;
}
//=============================================================================================================================
bool QNode::Init() {
    ros::init(init_argc,init_argv,"surena");
	if ( ! ros::master::check() ) {
        qDebug()<<"init error!";
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

    VisualizeInit(&n);
	// Add your ros communications here.
    _jointsSubscriber = n.subscribe("jointdata/qc", 1000, &QNode::SendDataToMotors, this);
    chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    _rigthtFtPublisher= n.advertise<geometry_msgs::Wrench>("surena/ft_r_state", 1000);
    _leftFtPublisher= n.advertise<geometry_msgs::Wrench>("surena/ft_l_state", 1000);
    _imuPublisher =    n.advertise<sensor_msgs::Imu>("surena/imu_state", 1000);
    _imuRPYPublisher = n.advertise<sensor_msgs::Imu>("surena/imuRPY_state",1000);
    _jointPublisher =    n.advertise<sensor_msgs::JointState>("surena/abs_joint_state", 1000);
    _incJointPublisher = n.advertise<sensor_msgs::JointState>("surena/inc_joint_state", 1000);
    _bumpPublisher = n.advertise<std_msgs::Int32MultiArray>("surena/bump_sensor_state", 1000);

    _activeCSPService = n.advertiseService("ActiveCSP", &QNode::ActiveCSP, this);
    _hommingService = n.advertiseService("Home", &QNode::Home, this);
    _resetAllNodesService = n.advertiseService("ResetAllNodes", &QNode::ResetAllNodes, this);
     //   _activeCSPService = n.advertiseService("ActiveCSP", &QNode::ActiveCSP);
   //ros::Subscriber sub = nh.subscribe("jointdata/qc", 1000, SendDataToMotors);
    for (int i = 0; i < 13; i++) {
    ActualPositions.append(0);
    IncPositions.append(0);

    }

    start();
	return true;
}
//================================================================================================================================================================
bool QNode::ActiveCSP(surena_eth::active_csp::Request  &req,surena_eth::active_csp::Response &res)
{

teststr="";
    Q_EMIT       SetActiveCSP();
    //this sleep is mandatory if u want thread wait until function return
    QThread::msleep(1);
    qDebug()<<"Active CSP...> "<<teststr;
}
//================================================================================================================================================================
bool QNode::Home(surena_eth::home::Request  &req,surena_eth::home::Response &res)
{

    Q_EMIT SetHome();
    QThread::msleep(1);
    ROS_INFO("request: homming mode");

}
//================================================================================================================================================================
bool QNode::ResetAllNodes(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
teststr="";
    Q_EMIT DoResetAllNodes();
    QThread::msleep(1);
    res.success=true;
    res.message=teststr.toStdString();
    qDebug()<<"Reset all...> "<<teststr;
}
//================================================================================================================================================================
void QNode::SendDataToMotors(const std_msgs::Int32MultiArray & msg)
{

    JointsData=msg;
    Q_EMIT NewjointDataReceived();
}
//================================================================================================================================================================
void QNode::run() {
    ros::NodeHandle n;
    ros::Rate loop_rate(200);
    _jointsSubscriber = n.subscribe("jointdata/qc", 1000, &QNode::SendDataToMotors, this);
    //  std_msgs::Int32MultiArray msg;

    while ( ros::ok() ) {
    std_msgs::String msg;
    sensor_msgs::JointState ActualJointState,IncJointState;
    std_msgs::Int32MultiArray BumpSensorState;
    //sensor_msgs::Imu imuSesnsorMsg;
    //geometry_msgs::Wrench ftSensorMessage;
    msg.data = "salam";

    std_msgs::MultiArrayDimension msg_dim;
    msg_dim.label = "bump";
    msg_dim.size = 1;
    BumpSensorState.layout.dim.clear();
    BumpSensorState.layout.dim.push_back(msg_dim);

    //chatter_publisher.publish(msg); // publish the value--of type Float64-
 ActualJointState.header.stamp = ros::Time::now();
  IncJointState.header.stamp = ros::Time::now();
  imuSesnsorMsg.header.stamp= ros::Time::now();
  imuSesnsorMsg.header.frame_id="base_link";
  imuRPYSensorMsg.header.stamp = ros::Time::now();
  imuRPYSensorMsg.header.frame_id="base_link";

  for(int i=0 ;i<13;i++){
  ActualJointState.position.push_back(ActualPositions[i]);
  IncJointState.position.push_back(IncPositions[i]);
  }
  for(int i=0 ;i<8;i++){
      BumpSensorState.data.push_back(BumpSensor[i]);
  //BumpSensorState.data[i]=i;
  }

/////


_rigthtFtPublisher.publish(RightFtSensorMessage);
_leftFtPublisher.publish(LeftFtSensorMessage);

///

  _bumpPublisher.publish(BumpSensorState);
  _imuPublisher.publish(imuSesnsorMsg);
  _imuRPYPublisher.publish(imuRPYSensorMsg);

    _jointPublisher.publish(ActualJointState);
    _incJointPublisher.publish(IncJointState);
    UpdateRobotModel(ActualPositions);
    ros::spinOnce();
    loop_rate.sleep();
        }

    //std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
//================================================================================================================================================================
void QNode::Callback(const std_msgs::Float64& message_holder)
{
	//std::stringstream ss;
	//ss << message_holder.data;
    Log(Info, message_holder);
	ROS_INFO("=============received value is: %f===========",message_holder.data); 
    //Q_EMIT NewDataReceived();
  //really could do something interesting here with the received data...but all we do is print it 
} 
//================================================================================================================================================================
void QNode::Log( const LogLevel &level, const std_msgs::Float64 &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << "received value is: " << msg.data;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << "received value is: " << msg.data;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << "received value is: " << msg.data;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << "received value is: " << msg.data;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << "received value is: " << msg.data;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}
//================================================================================================================================================================
