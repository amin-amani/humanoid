#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <QObject>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "surena_eth//active_csp.h"
#include "surena_eth/reset_node.h"
#include "surena_eth/home.h"
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <QEventLoop>
#include <qdebug.h>
#include <QTimer>
#include <QApplication>
#include <std_msgs/Int32MultiArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include "visualize.h"
#include "VisualizeControl.h"
#include <tf/transform_datatypes.h>
#include <QTimer>
#include "epos.h"

#include "qnode.h"
using namespace Qt;
class RobotController : public QObject
{
    Q_OBJECT

private:

 //  QNode *qnode;
public:

    QTimer timer;
    Epos Epose4;
    int pos=0;
    bool dir=false;
        ros::NodeHandle nh;
          ros::ServiceServer activeCSPService;
          ros::ServiceServer hommingService;
        QList<int> _motorPosition;
 //   explicit RobotController(QObject *parent = nullptr);
    explicit RobotController(int argc, char** argv, QWidget *parent = 0);
    static bool ActiveCSP(surena_eth::active_csp::Request &req, surena_eth::active_csp::Response &res);
    static bool Home(surena_eth::home::Request &req, surena_eth::home::Response &res);
    static void SendDataToMotors(const std_msgs::Int32MultiArray &msg);
    void UpdateModel();
signals:

public slots:
    void FeedBackReceived(QList<int16_t> ft, QList<int32_t> positions);
    void Timeout();
};

#endif // ROBOTCONTROLLER_H
