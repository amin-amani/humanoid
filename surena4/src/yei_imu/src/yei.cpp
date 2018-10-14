#include <qapplication.h>
#include <QtSerialPort/QtSerialPort>
#include "ros/ros.h"
#include "std_msgs/String.h"
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
    QApplication a(argc, argv);
QString command=":1\n";
    _comport.setBaudRate(115200);
    _comport.setPortName(argv[1]);
    qDebug()<<"argv2 "<<argv[2];

    if(QString::fromLatin1( argv[2])=="q")
    {
      qDebug()<< "set to quartenion ";
      command=":0\n";
    }
    else
       { command=":1\n";
          qDebug()<< "set to rol pitch yaw ";
    }


    //_comport.setPortName("ttyUSB0");
    if(!_comport.open(QSerialPort::ReadWrite))
    {
        qDebug()<< "Comport open error! "<<argv[1];
        return 1;
    }
    qDebug()<< "Comport open ok "<<argv[1];
    _comport.write(":237\n");
    _comport.flush();
    _comport.waitForReadyRead(1000);
    QString sn= "yei"+_comport.readLine().trimmed();
    QString tareName = "Tare" + sn;

    ros::init(argc, argv, sn.toStdString());
    ros::NodeHandle nh;
    ros::ServiceServer tareService = nh.advertiseService(tareName.toStdString(), TareSensor);
    ros::Publisher sensorPub = nh.advertise<sensor_msgs::Imu>(sn.toStdString(), 100);
    sensor_msgs::Imu imuMsg;

    ros::Rate loop_rate(1000);
//    while (ros::ok())
//    {
//        _comport.write(":0\n");
//        _comport.flush();
//        _comport.waitForReadyRead(2);

//        QString str= _comport.readLine().trimmed();
//        QStringList list=str.split(',');
//               //   qDebug()<< str;
//        if(list.count()==4){

//            imuMsg.orientation.x=list[0].toDouble();
//            imuMsg.orientation.y=list[1].toDouble();
//            imuMsg.orientation.z=list[2].toDouble();
//            imuMsg.orientation.w=list[3].toDouble();
//            sensorPub.publish(imuMsg);
//        }

////357097060826653/01

//         //ROS_INFO("ans");
//        ros::spinOnce();

//      //  loop_rate.sleep();
//    }
    qDebug()<<command;
    while (ros::ok())
    {
        _comport.write(command.toLatin1());
        _comport.flush();
        _comport.waitForReadyRead(2);

        QString str= _comport.readLine().trimmed();
        QStringList list=str.split(',');

                  if(list.count()== 3 && command==":1\n"){
 //qDebug()<< "euler"<<str;
                      imuMsg.orientation.x=list[2].toDouble();//*180/3.14159236;
                      imuMsg.orientation.y=-1*list[0].toDouble();//*180/3.14159236;
                      imuMsg.orientation.z=-1*list[1].toDouble();//*180/3.14159236;
                      imuMsg.orientation.w=0;
                      sensorPub.publish(imuMsg);
                  }
        else if(list.count()==4  && command==":0\n"){
 //qDebug()<< "qua"<<str;
            imuMsg.orientation.x=list[0].toDouble();
            imuMsg.orientation.y=list[1].toDouble();
            imuMsg.orientation.z=list[2].toDouble();
            imuMsg.orientation.w=list[3].toDouble();
            sensorPub.publish(imuMsg);
        }

//357097060826653/01

         //ROS_INFO("ans");
        ros::spinOnce();

      //  loop_rate.sleep();
    }
    _comport.close();
    return 0;
}
