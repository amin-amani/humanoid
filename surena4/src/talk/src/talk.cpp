#include <qapplication.h>
#include "ExportSO.h"
#include <iostream>
#include <QtMultimedia/QSound>
#include <qaudioformat.h>
#include "ros/ros.h"
#include <QDebug>
#include <qprocess.h>
#include <std_msgs/String.h>

ros::Subscriber TalkSubscriber;
QProcess *talker=new QProcess();
QString WorkingDirectory="/home/cast/speech-test/";

//===========================================================================
 void Talk(const std_msgs::String & msg)
 {
    static bool running=false;
    if(running){ROS_INFO("is running");return;}
    running=true;
    QStringList args;
    args.clear();
    args.append(QString::fromStdString(msg.data));
    talker->start(WorkingDirectory+"speech-test",args);
    talker->waitForFinished();
    running=false;
 }
 //===========================================================================
int main(int argc,char *argv[])
{
    QCoreApplication a(argc, argv);
    ros::init(argc, argv, "talk");
    ros::NodeHandle nh;
    TalkSubscriber = nh.subscribe("surena/talk", 1000,Talk);
    ros::Rate loop_rate(1);
    talker->setWorkingDirectory(WorkingDirectory);
    ros::spin();
    return 0;
}
