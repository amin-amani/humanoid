#include "robotcontroller.h"


//RobotController::RobotController(QObject *parent) : QObject(parent)
//{


//     //----------------services
//     activeCSPService = nh.advertiseService("ActiveCSP", RobotController::ActiveCSP);
//     hommingService = nh.advertiseService("Home", RobotController::Home);

//     //---------------subscribesrs
//     //ros::Subscriber sub = nh.subscribe("jointdata/qc", 1000, RobotController::SendDataToMotors);
//   //  ros::Subscriber statessub = nh.subscribe("/joint_states", 1000, StateFeedback);
//     //---------------publishers
//    // ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
//     VisualizeInit(&nh);


//     ros::Rate loop_rate(500);
// float t=0,tt=0;



//    if(Epose4.Init()==OK)// .Init(0xc251,0x1c03,(HANDLE)winId()))
//         {
//             qDebug()<<"Device connected.";
//             connect(&Epose4,SIGNAL(FeedBackReceived(QList<int16_t>,QList<int32_t>)),this,SLOT(FeedBackReceived(QList<int16_t>,QList<int32_t>)));
//       connect(&timer,SIGNAL(timeout()),this,SLOT(Timeout()));
//       timer.start(200);

//         }
//         else
//         {
//        qDebug()<<"Device does not exist!";

//         }
//   // Epose4.StartFeedBack();
////        Epose4.AllActiveCSP();
//// while (ros::ok())
////   {
////if(dir)pos++;
////else pos--;
////if(pos>1000){dir=!dir;}
////if(pos<-1000){dir=!dir;}
//////qDebug()<<pos;

////_motorPosition.clear();
//// for(int ii=0; ii<12 ; ii++)
//// {
////     _motorPosition.append(pos);
//// }


//// Epose4.SetAllPositionCST(_motorPosition,12);


////           ros::spinOnce();

////           loop_rate.sleep();
////       }
////     exit(0);

//}
RobotController::RobotController(int argc, char **argv, QWidget *parent)
{
//QNode mynode(argc,argv);

}

bool RobotController::ActiveCSP(surena_eth::active_csp::Request  &req,surena_eth::active_csp::Response &res)
{
        qDebug()<<"Active CSP ";
}
bool RobotController::Home(surena_eth::home::Request  &req,surena_eth::home::Response &res)
{
    ROS_INFO("request: ipm init");
}
void RobotController::UpdateModel()
{
     QList<double> _motorPosition;
     int32_t val=0;

     _motorPosition.append(0.0);

         for(int i=0;i<12;i++){
    if(Epose4.ReadRegister(0x60e4,0x02,1,i,val)!=OK)
    {
    qDebug()<<"error";
    }
        double v=val;
     v*=M_PI*2/4096;

       _motorPosition.append(v);

}
    // Epose4.WriteRegister(0x6065,0,1,1,10000);
//qDebug()<<"pos="<<val;
//  _motorPosition.append(msg.data[0]);
//  _motorPosition.append(msg.data[1]);
//  _motorPosition.append(msg.data[2]);
//  _motorPosition.append(msg.data[3]);
//  _motorPosition.append(msg.data[4]);
//  _motorPosition.append(msg.data[5]);
//  _motorPosition.append(msg.data[6]);
//  _motorPosition.append(msg.data[7]);
//  _motorPosition.append(msg.data[8]);
//  _motorPosition.append(msg.data[9]);
//  _motorPosition.append(msg.data[10]);
//  _motorPosition.append(msg.data[11]);

UpdateRobotModel(_motorPosition);

}
void RobotController::SendDataToMotors(const std_msgs::Int32MultiArray & msg)
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

void RobotController::FeedBackReceived(QList<int16_t> ft, QList<int32_t> positions)
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

void RobotController::Timeout()
{
    qDebug()<<"Tick";
}
