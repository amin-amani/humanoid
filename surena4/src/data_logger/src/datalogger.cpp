#include "mainwindow.h"
#include <QApplication>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include<std_msgs/Int32MultiArray.h>
#include <stdlib.h>
#include "qnode.h"


int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    QNode *rosNode=new QNode(argc, argv);
    if(!rosNode->Init())
    {
       // QLOG_FATAL()<<"Ros node init error please check ros master!";
        exit(0);
    }
    //ros::init(argc, argv, "dataLogger_node");
    MainWindow w;
    QObject::connect(rosNode,SIGNAL(NewjointDataReceived()),&w,SLOT(NewJointdata()));
    w.show();

    return a.exec();
}
