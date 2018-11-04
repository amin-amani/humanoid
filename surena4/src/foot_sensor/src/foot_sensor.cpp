#include <qapplication.h>
#include <QtSerialPort/QtSerialPort>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/Imu.h"
#include "std_srvs/Empty.h"
//:6 for getting all data
//:237 for serial number
QSerialPort _comport;
bool TareSensor(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    if(!_comport.isOpen())return false;
    _comport.write(":96\n");
    _comport.flush();
    ROS_INFO("request: tare ");
    return true;

}
int main(int argc, char **argv)
{
   int  offset[4]={1023, 928, 3035, 3098};

    std_msgs::Int32MultiArray msg;
    std_msgs::MultiArrayDimension msg_dim;
    QApplication a(argc, argv);

    _comport.setBaudRate(115200);
    _comport.setPortName(argv[1]);
    qDebug()<<"argv2 "<<argv[2];




    if(!_comport.open(QSerialPort::ReadWrite))
    {
        qDebug()<< "Comport open error! "<<argv[1];
        return 1;
    }
    qDebug()<< "Comport open ok "<<argv[1];


    ros::init(argc, argv, "foot_R");
    ros::NodeHandle nh;
   // ros::ServiceServer tareService = nh.advertiseService(tareName.toStdString(), TareSensor);
    ros::Publisher sensorPub = nh.advertise<std_msgs::Int32MultiArray>("/foot", 100);


    ros::Rate loop_rate(1000);
    msg_dim.label = "foot_sensor";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);
    //qDebug()<<command;
    while (ros::ok())
    {
//ROS_INFO("pub");
msg.data.clear();

//for(size_t i = 0; i < 4; i++)
//msg.data.push_back(i);
//pub.publish(msg);
//if(_comport.canReadLine())
//{

_comport.waitForReadyRead(-1);
if(_comport.canReadLine())
{
     QString str= _comport.readLine().trimmed();
      QStringList list=str.split(' ');
      if(list.count()==4)
      {
         // for(size_t i = 0; i < 4; i++)
         msg.data.push_back(list[0].toInt()-offset[0]);
         msg.data.push_back(-list[1].toInt()+offset[1]);
         msg.data.push_back(list[2].toInt()-offset[2]);
         msg.data.push_back(-list[3].toInt()+offset[3]);
      }
     // qDebug()<< list[0]<<" "<<list[1];
      sensorPub.publish(msg);
}

   //ROS_INFO("data=%s", _comport.readLine().toStdString());

//}

ros::spinOnce();
loop_rate.sleep();
    }
    _comport.close();
    return 0;
}
