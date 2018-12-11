/**
 * @file /include/my_qt_gui_subscriber/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef my_qt_gui_subscriber_QNODE_HPP_
#define my_qt_gui_subscriber_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <std_msgs/Float64.h>
#include <QThread>
#include <QDebug>
#include <QList>
#include <QStringListModel>
#include <std_msgs/Bool.h>
#include "std_msgs/String.h"
#include "surena_eth/active_csp.h"
#include "surena_eth/reset_node.h"
#include "surena_eth/home.h"
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <QEventLoop>
#include <QTimer>
#include <QApplication>
#include <std_msgs/Int32MultiArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include "visualize.h"
#include <tf/transform_datatypes.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

//namespace my_qt_gui_subscriber {

/*****************************************************************************
** Class
*****************************************************************************/
    using namespace visualization_msgs;
class QNode : public QThread {
    Q_OBJECT
public:
    /*********************
    ** Logging
    **********************/
    enum LogLevel {
             Debug,
             Info,
             Warn,
             Error,
             Fatal
     };
    QString teststr="";
     std_msgs::Int32MultiArray  JointsData;

QList<double> ActualPositions;
QList<double> IncPositions;

     //=================================================================================================
    QNode();
    //=================================================================================================
	QNode(int argc, char** argv );
    //=================================================================================================
	virtual ~QNode();
    //=================================================================================================
	bool init();
    //=================================================================================================
	bool init(const std::string &master_url, const std::string &host_url);
    //=================================================================================================
	void run();
    //=================================================================================================
	void myCallback(const std_msgs::Float64& message_holder);
    //=================================================================================================


//=================================================================================================
	QStringListModel* loggingModel() { return &logging_model; }
    //=================================================================================================
	void log( const LogLevel &level, const std_msgs::Float64 &msg);
    //=================================================================================================
    void SendDataToMotors(const std_msgs::Int32MultiArray &msg);
    //=================================================================================================
    bool ActiveCSP(surena_eth::active_csp::Request &req, surena_eth::active_csp::Response &res);
    //=================================================================================================
    bool Home(surena_eth::home::Request &req, surena_eth::home::Response &res);
    //=================================================================================================
    bool ResetAllNodes(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    //=================================================================================================

  Q_SIGNALS:
    //=================================================================================================
	void loggingUpdated();
    //=================================================================================================
    void rosShutdown();
    //=================================================================================================
    void NewjointDataReceived();
    //=================================================================================================
    void SetActiveCSP();
    //=================================================================================================
    void DoResetAllNodes();
    //=================================================================================================
    void SetHome();
    //=================================================================================================

private:
    //=================================================================================================
	int init_argc;
	char** init_argv;
    ros::Subscriber _jointsSubscriber;
    ros::Publisher chatter_publisher;
    ros::Publisher _jointPublisher,_incJointPublisher;
    QStringListModel logging_model;
    ros::ServiceServer _activeCSPService;
    ros::ServiceServer _hommingService;
    ros::ServiceServer _resetAllNodesService;

    //------------------------------------------------------------------------------------------

    //=================================================================================================
};

//}  // namespace my_qt_gui_subscriber

#endif /* my_qt_gui_subscriber_QNODE_HPP_ */
