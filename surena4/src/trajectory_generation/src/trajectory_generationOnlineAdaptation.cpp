#include "ros/ros.h"
#include "std_msgs/String.h"
#include"Eigen/Dense"
#include <vector>
#include <iostream>
#include <QString>
#include <QList>
#include "Robot.h"
//#include"TaskSpace.h"
#include"taskspaceonline2.h"
#include <qmath.h>
#include <cstring>
#include<qdebug.h>
#include <Eigen/Geometry>
#include <cstdlib>
//#include <link.h>
#include "Eigen/eiquadprog.h"
#include "Eigen/Core"
#include "Eigen/Cholesky"
#include "Eigen/LU"
#include<std_msgs/Int32MultiArray.h>
#include<std_msgs/Float32MultiArray.h>
#include<math.h>
#include<sensor_msgs/Imu.h>
#include<std_msgs/Float64.h>
#include "qcgenerator.h"
#include<termios.h>
#include<gazebo_msgs/LinkStates.h>
#include<sensor_msgs/JointState.h>
#include"pidcontroller.h"




ros::Publisher pub1  ;
ros::Publisher pub2  ;
ros::Publisher pub3  ;
ros::Publisher pub4  ;
ros::Publisher pub5  ;
ros::Publisher pub6  ;
ros::Publisher pub7  ;
ros::Publisher pub8  ;
ros::Publisher pub9  ;
ros::Publisher pub10 ;
ros::Publisher pub11 ;
ros::Publisher pub12 ;
ros::Publisher pub13 ;
ros::Publisher pub14 ;
ros::Publisher pub15 ;
ros::Publisher pub16 ;
ros::Publisher pub17 ;
ros::Publisher pub18 ;
ros::Publisher pub19 ;
ros::Publisher pub20 ;
ros::Publisher pub21 ;
ros::Publisher pub22 ;
ros::Publisher pub23 ;
ros::Publisher pub24 ;
ros::Publisher pub25 ;
ros::Publisher pub26 ;
ros::Publisher pub27 ;
ros::Publisher pub28 ;

//ros::Publisher pid1 ;



int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}


void  SendGazebo(QList<LinkM> links,MatrixXd RollModifieds, double PitchModifieds, double theta_r, double phi_r, double theta_l, double phi_l){
    if(links.count()<28){qDebug()<<"index err";return;}
    std_msgs::Float64 data;

    data.data=links[1].JointAngle;
    pub1.publish(data);
    data.data=links[2].JointAngle+RollModifieds(0,0);
    pub2.publish(data);
    data.data=links[3].JointAngle+PitchModifieds;
    pub3.publish(data);
    data.data=links[4].JointAngle;
    pub4.publish(data);
    data.data=links[5].JointAngle+theta_r;
    pub5.publish(data);
    data.data=links[6].JointAngle+phi_r;
    pub6.publish(data);
    data.data=links[7].JointAngle;
    pub7.publish(data);
    data.data=links[8].JointAngle+RollModifieds(1,0);
    pub8.publish(data);
    data.data=links[9].JointAngle+PitchModifieds;
    pub9.publish(data);
    data.data=links[10].JointAngle;
    pub10.publish(data);
    data.data=links[11].JointAngle+theta_l;
    pub11.publish(data);
    data.data=links[12].JointAngle+phi_l;
    pub12.publish(data);
    data.data=links[13].JointAngle;
    pub13.publish(data);
    data.data=links[14].JointAngle;
    pub14.publish(data);
    data.data=links[15].JointAngle;
    pub15.publish(data);
    data.data=links[16].JointAngle;
    pub16.publish(data);
    data.data=links[17].JointAngle;
    pub17.publish(data);
    data.data=links[18].JointAngle;
    pub18.publish(data);
    data.data=links[19].JointAngle;
    pub19.publish(data);
    data.data=links[20].JointAngle;
    pub20.publish(data);
    data.data=links[21].JointAngle;
    pub21.publish(data);
    data.data=links[22].JointAngle;
    pub22.publish(data);
    data.data=links[23].JointAngle;
    pub23.publish(data);
    data.data=links[24].JointAngle;
    pub24.publish(data);
    data.data=links[25].JointAngle;
    pub25.publish(data);
    data.data=links[26].JointAngle;
    pub26.publish(data);
    data.data=links[27].JointAngle;
    pub27.publish(data);
    data.data=links[28].JointAngle;
    pub28.publish(data);



}






using namespace  std;
using namespace  Eigen;
//data of left foot sensor
int a;
int b;
int c;
int d;

int e;
int f;
int g;
int h;

bool RFT;//True Right Support Phase used in Taskspace online (in Ankle Trajectory)
bool LFT;//True Left Support Phase used in Taskspace online (in Ankle Trajectory)
bool KRtemp;// for online: when true-> current positon is updated _end of step)
bool KLtemp;// for online
bool aState;
bool bState;
bool cState;
bool dState;
bool LeftFootLanded;
bool RightFootLanded;
//bool FullContactDetected;

//angles of ankle adaptation
double teta_motor_L;//pitch
double teta_motor_R;
double phi_motor_L;//roll
double phi_motor_R;

//offsets after adaptation
double Offset_teta_L;
double Offset_teta_R;
double Offset_phi_L;
double Offset_phi_R;

int qc_offset[12];
double roll_absoulte[4];
bool qc_initial_bool;
bool qc_initial_bool_roll;
//
void receiveFootSensor(const std_msgs::Int32MultiArray& msg)
{
    if (msg.data.size()!=8) {
        qDebug("the size of sensor data is in wrong");
        return;
    }

    //ROS_INFO("I heard: [%d  %d %d %d %d  %d %d %d]", (int)msg.data[0],(int)msg.data[1],(int)msg.data[2],(int)msg.data[3],(int)msg.data[4],(int)msg.data[5],(int)msg.data[6],(int)msg.data[7]);
    int temp[8];
    int tempInt[8];


    temp[0]=msg.data[0]-1012;
    temp[1]=-1*(msg.data[1]-924);
    temp[2]=msg.data[2]-3038;
    temp[3]=-1*(msg.data[3]-3098);

    //normalizing data of sensors
    temp[0]=temp[0]*(100.0/(1097-1011));
    temp[1]=temp[1]*(100.0/(925-841));
    temp[2]=temp[2]*(100.0/(3131-3038));
    temp[3]=temp[3]*(100.0/(3098-3001));

    tempInt[0]=temp[0];
    tempInt[1]=temp[1];
    tempInt[2]=temp[2];
    tempInt[3]=temp[3];

    a=tempInt[0];
    b=tempInt[1];
    c=tempInt[2];
    d=tempInt[3];
    //ROS_INFO("I heard a b c d: [%d  %d %d %d]", a,b,c,d);



    temp[4]=msg.data[4]-3042;
    temp[5]=-1*(msg.data[5]-3008);
    temp[6]=msg.data[6]-1133;
    temp[7]=-1*(msg.data[7]-1016);

    //normalizing data of sensors
    temp[4]=temp[4]*(100.0/(3126-3042));
    temp[5]=temp[5]*(100.0/(3008-2914));
    temp[6]=temp[6]*(100.0/(1225-1133));
    temp[7]=temp[7]*(100.0/(1016-920));



    tempInt[4]=temp[4];
    tempInt[5]=temp[5];
    tempInt[6]=temp[6];
    tempInt[7]=temp[7];

    e=tempInt[4];
    f=tempInt[5];
    g=tempInt[6];
    h=tempInt[7];
    // ROS_INFO("I heard e f g h: [%d  %d %d %d]", e,f,g,h);

    //deleting data with negative sign
    if (a<0) {
        a=0;

    }
    if (b<0) {
        b=0;

    }
    if (c<0) {
        c=0;

    }
    if (d<0) {
        d=0;

    }

    if (e<0) {
        e=0;

    }
    if (f<0) {
        f=0;

    }
    if (g<0) {
        g=0;

    }
    if (h<0) {
        h=0;

    }
//     ROS_INFO("I heard a b c d: [%d  %d %d %d]", a,b,c,d);
//     ROS_INFO("I heard e f g h: [%d  %d %d %d]", e,f,g,h);

    // ROS_INFO("I heard: [%d  %d %d %d]", tempInt[0],tempInt[1],tempInt[2],tempInt[3]);
}


void qc_initial(const sensor_msgs::JointState & msg){
    if (qc_initial_bool){
        for (int i = 0; i < 12; ++i) {
            qc_offset[i]=int(msg.position[i+1]);

        }

        qc_initial_bool=false;
        //qc_initial_bool_roll=true;


    ROS_INFO("Offset=%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\nInitialized!",
             qc_offset[0],qc_offset[1],qc_offset[2],qc_offset[3],qc_offset[4],
            qc_offset[5],qc_offset[6],qc_offset[7],qc_offset[8],qc_offset[9],
            qc_offset[10],qc_offset[11]);}
 //getch();
}

void roll_absolute_correction(const sensor_msgs::JointState & msg){
    roll_absoulte[2]=roll_absoulte[0];
    roll_absoulte[3]=roll_absoulte[1];
    roll_absoulte[0]= -msg.position[10];
    roll_absoulte[1]= -msg.position[11];
}

//void roll_qc_init(const sensor_msgs::JointState & msg){
//    if (!qc_initial_bool && qc_initial_bool_roll){

//        qc_offset[9]=int(msg.position[10]);
//        qc_offset[10]=int(msg.position[11]);


//        ROS_INFO("Offset=%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\nInitialized!",
//                 qc_offset[0],qc_offset[1],qc_offset[2],qc_offset[3],qc_offset[4],
//                qc_offset[5],qc_offset[6],qc_offset[7],qc_offset[8],qc_offset[9],
//                qc_offset[10],qc_offset[11]);

//        qc_initial_bool_roll=false;
//        getch();
//    }
//}




//int a,b,c,d,e,f,g,h;

MatrixXd quater2rot(double w,double x,double y, double z){
    MatrixXd R(3,3);
    R<<w*w+x*x-y*y-z*z,2*x*y-2*w*z,2*x*z+2*w*y,
            2*x*y+2*w*z,w*w-x*x+y*y-z*z,2*y*z-2*w*x,
            2*x*z-2*w*y,2*y*z+2*w*x,w*w-x*x-y*y+z*z;
    return R;

}

void ankle_states(const gazebo_msgs::LinkStates& msg){
    double x_left, x_right,y_left, y_right,z_left, z_right;
Vector3d vec_A_E;
Vector3d vec_B_F;
Vector3d vec_C_G;
Vector3d vec_D_H;

vec_A_E<<-.085,
         -.08,
        -.11;
vec_B_F<<.135,
         -.08,
        -.11;
vec_C_G<<.135,
         .08,
        -.11;
vec_D_H<<-.085,
         .08,
        -.11;

    x_left=msg.pose[7].position.x;
    x_right=msg.pose[13].position.x;
    y_left=msg.pose[7].position.y;
    y_right=msg.pose[13].position.y;

    z_left=msg.pose[7].position.z;
    z_right=msg.pose[13].position.z;

    MatrixXd R_left(3,3);
    MatrixXd R_right(3,3);
    R_left=quater2rot(msg.pose[7].orientation.w,msg.pose[7].orientation.x,msg.pose[7].orientation.y,msg.pose[7].orientation.z);
    R_right=quater2rot(msg.pose[13].orientation.w,msg.pose[13].orientation.x,msg.pose[13].orientation.y,msg.pose[13].orientation.z);
Vector3d temp;
temp=R_left*vec_A_E;
//A=temp(2)+z_left;
a=int(((.02-temp(2)-z_left)+abs(.02-temp(2)-z_left))*5000/2);
temp=R_left*vec_B_F;
//B=temp(2)+z_left;
b=int(((.02-temp(2)-z_left)+abs(.02-temp(2)-z_left))*5000/2);
temp=R_left*vec_C_G;
//C=temp(2)+z_left;
c=int(((.02-temp(2)-z_left)+abs(.02-temp(2)-z_left))*5000/2);
temp=R_left*vec_D_H;
//D=temp(2)+z_left;
d=int(((.02-temp(2)-z_left)+abs(.02-temp(2)-z_left))*5000/2);
temp=R_right*vec_A_E;
//E=temp(2)+z_right;
e=int(((.02-temp(2)-z_right)+abs(.02-temp(2)-z_right))*5000/2);
temp=R_right*vec_B_F;
//F=temp(2)+z_right;
f=int(((.02-temp(2)-z_right)+abs(.02-temp(2)-z_right))*5000/2);
temp=R_right*vec_C_G;
//G=temp(2)+z_right;
g=int(((.02-temp(2)-z_right)+abs(.02-temp(2)-z_right))*5000/2);
temp=R_right*vec_D_H;
//H=temp(2)+z_right;
h=int(((.02-temp(2)-z_right)+abs(.02-temp(2)-z_right))*5000/2);


 //ROS_INFO("z_left=%f  A=%d,B=%d,C=%d,D=%d\tz_right=%f  E=%d,F=%d,G=%d,H=%d",z_left,a,b,c,d,z_right,e,f,g,h);

//    ROS_INFO("x_left=%f,x_right=%f,y_left=%f,y_right=%fz_left=%f,z_right=%f",x_left,x_right,y_left,y_right,z_left,z_right);

}















int main(int argc, char **argv)
{
double min_test=0;
double max_test=0;
    vector<double> cntrl(13);
    QCgenerator QC;
    for (int i = 0; i < 12; ++i) {
         qc_offset[i]=0;
         //qc_corretction[i]=0;
    }
    qc_initial_bool=!true;
    qc_initial_bool_roll=false;

    //check _timesteps
    QElapsedTimer timer;
    Robot SURENA;//model of robot & kinematics funcs(IK & FK)
    TaskSpaceOnline2 SURENAOnlineTaskSpace1;
    QList<LinkM> links;
    MatrixXd PoseRoot;//position of pelvis respected to global coordinate
    MatrixXd PoseRFoot;//position of right ankle joint respected to global coordinate
    MatrixXd PoseLFoot;//position of left ankle joint respected to global coordinate
    //double hipRoll=0;
   // double dt;

    //parameters of ankle adaptation
    double k1;
    double k2;
    double k3;
    double k4;

    SURENAOnlineTaskSpace1.RightSupport=true; //not changed with sensor data,
    SURENAOnlineTaskSpace1.LeftSupport=true;

    SURENAOnlineTaskSpace1.RightSensorActive=false; // is set true once when right leg is in swing mode, and immediately false to get out of if statement of bump sensor
    SURENAOnlineTaskSpace1.LeftSensorActive=false;


    SURENAOnlineTaskSpace1.oldLeftFootZ=SURENAOnlineTaskSpace1._lenghtOfAnkle;//expected foot z
    SURENAOnlineTaskSpace1.oldRightFootZ=SURENAOnlineTaskSpace1._lenghtOfAnkle;

    bool aState=false;
    bool bState=false;
    bool cState=false;
    bool dState=false;


    bool eState=false;
    bool fState=false;
    bool gState=false;
    bool hState=false;

    LeftFootLanded=false;//true when landing is detected by sensors
    RightFootLanded=false;
   // FullContactDetected=false;


    SURENAOnlineTaskSpace1.RightFootOrientationAdaptator=false;//adaptation occurs when true
    SURENAOnlineTaskSpace1.LeftFootOrientationAdaptator=false;


    teta_motor_L=0;
    teta_motor_R=0;
    phi_motor_L=0;
    phi_motor_R=0;


    Offset_teta_L=0;
    Offset_teta_R=0;
    Offset_phi_L=0;
    Offset_phi_R=0;



    double footSensorSaturation=75;//if all sensors data are bigger than this amount, this means the foot is landed on the ground
    double footSensorthreshold=4;// will start orientaition correction

    double StartTime=0;//starttime does not mean start time,but means global time
    double  DurationOfStartPhase=6;
    double  DurationOfendPhase=6;
    //SURENAOnlineTaskSpace.GetAccVelPos();
    bool startPhase=true;
    bool endPhase=true;
    bool walk=true;
      MatrixXd RollModified(2,1);//parameters for hip roll angles charge, for keep pelvis straight
     RollModified<<0,0;
     double pitchOffset=SURENAOnlineTaskSpace1.HipPitchModification;
     double PitchModified=0;
    PoseRoot.resize(6,1); //pelvis trajectory from taskspace_online,xyzrpy
    PoseRFoot.resize(6,1);//right ankle joint trajectory from taskspace_online,xyzrpy
    PoseLFoot.resize(6,1);//left ankle joint trajectory from taskspace_online,xyzrpy
    //QList<LinkM> links;
    bool indexLastDS=true;//used in last double support

    //*******************This part of code is for initialization of joints of the robot for walking**********************************
    int count = 0;

    ros::init(argc, argv, "myNode");

    ros::NodeHandle nh;
    ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",1000);


    ros::Publisher  contact_flag  = nh.advertise<std_msgs::Int32MultiArray>("contact_flag_timing",100);
 ros::Subscriber sub = nh.subscribe("/surena/bump_sensor_state", 1000, receiveFootSensor);
 ros::Subscriber qcinit = nh.subscribe("/surena/inc_joint_state", 1000, qc_initial);
ros::Subscriber roll_absolute_sub = nh.subscribe("/surena/abs_joint_state",1000,roll_absolute_correction);
//ros::Subscriber roll_qc_init_sub = nh.subscribe("/surena/abs_joint_num",1000,roll_qc_init);


//    pub1  = nh.advertise<std_msgs::Float64>("rrbot/joint1_position_controller/command",1000);
//    pub2  = nh.advertise<std_msgs::Float64>("rrbot/joint2_position_controller/command",1000);
//    pub3  = nh.advertise<std_msgs::Float64>("rrbot/joint3_position_controller/command",1000);
//    pub4  = nh.advertise<std_msgs::Float64>("rrbot/joint4_position_controller/command",1000);
//    pub5  = nh.advertise<std_msgs::Float64>("rrbot/joint5_position_controller/command",1000);
//    pub6  = nh.advertise<std_msgs::Float64>("rrbot/joint6_position_controller/command",1000);
//    pub7  = nh.advertise<std_msgs::Float64>("rrbot/joint7_position_controller/command",1000);
//    pub8  = nh.advertise<std_msgs::Float64>("rrbot/joint8_position_controller/command",1000);
//    pub9  = nh.advertise<std_msgs::Float64>("rrbot/joint9_position_controller/command",1000);
//    pub10 = nh.advertise<std_msgs::Float64>("rrbot/joint10_position_controller/command",1000);
//    pub11 = nh.advertise<std_msgs::Float64>("rrbot/joint11_position_controller/command",1000);
//    pub12 = nh.advertise<std_msgs::Float64>("rrbot/joint12_position_controller/command",1000);
//    pub13 = nh.advertise<std_msgs::Float64>("rrbot/joint13_position_controller/command",1000);
//    pub14 = nh.advertise<std_msgs::Float64>("rrbot/joint14_position_controller/command",1000);
//    pub15 = nh.advertise<std_msgs::Float64>("rrbot/joint15_position_controller/command",1000);
//    pub16 = nh.advertise<std_msgs::Float64>("rrbot/joint16_position_controller/command",1000);
//    pub17 = nh.advertise<std_msgs::Float64>("rrbot/joint17_position_controller/command",1000);
//    pub18 = nh.advertise<std_msgs::Float64>("rrbot/joint18_position_controller/command",1000);
//    pub19 = nh.advertise<std_msgs::Float64>("rrbot/joint19_position_controller/command",1000);
//    pub20 = nh.advertise<std_msgs::Float64>("rrbot/joint20_position_controller/command",1000);
//    pub21 = nh.advertise<std_msgs::Float64>("rrbot/joint21_position_controller/command",1000);
//    pub22 = nh.advertise<std_msgs::Float64>("rrbot/joint22_position_controller/command",1000);
//    pub23 = nh.advertise<std_msgs::Float64>("rrbot/joint23_position_controller/command",1000);
//    pub24 = nh.advertise<std_msgs::Float64>("rrbot/joint24_position_controller/command",1000);
//    pub25 = nh.advertise<std_msgs::Float64>("rrbot/joint25_position_controller/command",1000);
//    pub26 = nh.advertise<std_msgs::Float64>("rrbot/joint26_position_controller/command",1000);
//    pub27 = nh.advertise<std_msgs::Float64>("rrbot/joint27_position_controller/command",1000);
//    pub28 = nh.advertise<std_msgs::Float64>("rrbot/joint28_position_controller/command",1000);

    ros::Subscriber ankleStates = nh.subscribe("/gazebo/link_states", 10, ankle_states);



int32_t contact_flag_timing=1000;
int32_t contact_flag_sensor=1000;
int32_t contact_flag_sensor2=1000;

    ros::Rate loop_rate(200);
    std_msgs::Int32MultiArray msg;
std_msgs::Int32MultiArray roll_modif_msg;
    std_msgs::Int32MultiArray msg_contact_flag;
    std_msgs::MultiArrayDimension msg_dim;

    msg_dim.label = "joint_position";
    msg_dim.size = 1;

    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);

msg_contact_flag.data.clear();
msg_contact_flag.data.push_back(contact_flag_timing);

//msg_flag.data=-msg_flag.data;

    k1=0.000015;
    k2=0.000015;
    k3=0.000015;
    k4=0.000015;



    KRtemp=false;
    KLtemp=false;
    RFT=false;
    LFT=false;
    links=SURENA.GetLinks();
    SURENAOnlineTaskSpace1.StepNumber=1;

    MatrixXd P;
    MatrixXd Pz;



ROS_INFO("press any key to start!");
getch();
ROS_INFO("started!");


bool firstcontact=true;


//-------------------------------------------------


//while (ros::ok()) {
//    if (qc_initial_bool) {
//         ROS_INFO("qc is initializing!");
//            ros::spinOnce();
// continue;
//     }
//    msg.data.clear();
//    for(int  i = 0;i < 12;i++)
//    {
//        msg.data.push_back(qc_offset[i]);
//    }

//    for(int  i = 12;i < 28;i++)
//    {
//        msg.data.push_back(0);
//    }

//    chatter_pub.publish(msg);
//    ros::spinOnce();
//    loop_rate.sleep();
//}
// ROS_INFO("exit");

//exit(0);
// ROS_INFO("exit4");
 //------------------------------------------------------
    while (ros::ok())
    {

       //  for robot test musbe uncommented
       if (qc_initial_bool) {
            ROS_INFO("qc is initializing!");
               ros::spinOnce();
    continue;
        }



//
  //   ROS_INFO("a=%d b=%d c=%d d=%d e=%d f=%d g=%d h=%d",a,b,c,d,e,f,g,h);

       // ROS_INFO("%d\t%f",SURENAOnlineTaskSpace1.localtimingInteger,SURENAOnlineTaskSpace1.localTiming);
        //-------------flag to show expected contact according to timing-------------//
        //-------------sign of flag changes-------------//
        if(SURENAOnlineTaskSpace1.localTiming<.1){contact_flag_timing=-contact_flag_timing;}
       // ROS_INFO("globaltime=%f\tlocaltiming=%f  %d",StartTime,SURENAOnlineTaskSpace1.localTiming,contact_flag_timing);

//ROS_INFO("a=%d,  b=%d,  c=%d,  d=%d,  e=%d,  f=%d,  g=%d,  h=%d",a,b,c,d,e,f,g,h);
        //-------------for detecting the first contact of Left foot with ground-------------//
        //-------------flag to show contact detected by sensors-------------//
        //-------------sign of flag changes-------------//



        if (SURENAOnlineTaskSpace1.LeftSensorActive==true && firstcontact==true &&( (a)>=footSensorSaturation || (b)>=footSensorSaturation || (c)>=footSensorSaturation || (d)>=footSensorSaturation)){
            contact_flag_sensor2=-contact_flag_sensor2;
            firstcontact=false;
            ROS_INFO("shalap [%f  %f] a=%d b=%d c=%d d=%d", SURENAOnlineTaskSpace1.localTiming,SURENAOnlineTaskSpace1.globalTime,a,b,c,d);

        }



        //-------------for detecting the full contact of Left foot with ground-------------//
        //-------------flag to show contact detected by sensors-------------//
        //-------------sign of flag changes-------------//
        if (SURENAOnlineTaskSpace1.LeftSensorActive==true && (a)>=footSensorSaturation && (b)>=footSensorSaturation && (c)>=footSensorSaturation && (d)>=footSensorSaturation){
            contact_flag_sensor=-contact_flag_sensor;
            ROS_INFO("left swing foot landing is successful = [%f  %f] a=%d b=%d c=%d d=%d", SURENAOnlineTaskSpace1.localTiming,SURENAOnlineTaskSpace1.globalTime,a,b,c,d);
            aState=true;
            bState=true;
            cState=true;
            dState=true;
            LeftFootLanded=true;//this variable is used for flag up when the left foot have a full contact with the ground
            SURENAOnlineTaskSpace1.LeftSensorActive=false;
            firstcontact=true;
            SURENAOnlineTaskSpace1.LeftFootOrientationAdaptator=false;
            //do nothing all sensors are on the ground

            Offset_teta_L=teta_motor_L;
            Offset_phi_L=phi_motor_L;


            //exit(0);//will finish the code but during walking is not true
        }

        else if (/*SURENAOnlineTaskSpace1.LeftSensorActive==false*/true) {//---------when the four left foot sensors are not saturated-->>>>during landing------//

            LeftFootLanded=false;
            if ((a)>=footSensorthreshold ){aState=true;}else {aState=false;}
            if ((b)>=footSensorthreshold ){bState=true;}else {bState=false;}
            if ((c)>=footSensorthreshold ){cState=true;}else {cState=false;}
            if ((d)>=footSensorthreshold ){dState=true;}else {dState=false;}


            //-----------------Pitch left ankle motor control---------------//
            if (abs(b-a)>=abs(c-d)) {
                if (abs(a-b)<100) {
                    teta_motor_L=1*teta_motor_L+k1*(SURENAOnlineTaskSpace1.LeftFootOrientationAdaptator==true)*(a-b);
                }
                else {
                    teta_motor_L=teta_motor_L;
                }
            }
            else {
                if (abs(d-c)<100) {
                    teta_motor_L=1*teta_motor_L+k1*(SURENAOnlineTaskSpace1.LeftFootOrientationAdaptator==true)*(d-c);
                }
                else {
                    teta_motor_L=teta_motor_L;
                }
            }


            //----------------Roll left ankle motor control---------------//
            if (abs(c-b)>=abs(d-a)) {
                if (abs(c-b)<100) {
                    phi_motor_L=1*phi_motor_L+k2*(SURENAOnlineTaskSpace1.LeftFootOrientationAdaptator==true)*(c-b);
                }
                else {
                    phi_motor_L=phi_motor_L;
                }
            }
            else {
                if (abs(a-d)<100) {
                    phi_motor_L=1*phi_motor_L+k2*(SURENAOnlineTaskSpace1.LeftFootOrientationAdaptator==true)*(d-a);
                }
                else {
                    phi_motor_L=phi_motor_L;
                }
            }
        }

        //        ROS_INFO("Tc= [%d] ", SURENAOnlineTaskSpace1.Tc);
        //        ROS_INFO("Tds= [%f] ", SURENAOnlineTaskSpace1.TDs);

        //-------------for detecting the first contact of Right foot with ground-------------//
        //-------------flag to show contact detected by sensors-------------//
        //-------------sign of flag changes-------------//
        if (SURENAOnlineTaskSpace1.RightSensorActive==true && firstcontact==true && ( (e)>=footSensorSaturation || (f)>=footSensorSaturation || (g)>=footSensorSaturation || (h)>=footSensorSaturation)){
            contact_flag_sensor2=-contact_flag_sensor2;
            firstcontact=false;
           ROS_INFO("shooloop[%f  %f]  e=%d f=%d g=%d h=%d", SURENAOnlineTaskSpace1.localTiming,SURENAOnlineTaskSpace1.globalTime,e,f,g,h);
        }

        //-------------for detecting the full contact of Right foot with ground-------------//
        if (SURENAOnlineTaskSpace1.RightSensorActive==true && (e)>=footSensorSaturation && (f)>=footSensorSaturation && (g)>=footSensorSaturation && (h)>=footSensorSaturation){
            // qDebug("swing foot landing is successful");
            contact_flag_sensor=-contact_flag_sensor;
            ROS_INFO("Right swing foot landing is successful= [%f  %f]  e=%d f=%d g=%d h=%d", SURENAOnlineTaskSpace1.localTiming,SURENAOnlineTaskSpace1.globalTime,e,f,g,h);
            eState=true;
            fState=true;
            gState=true;
            hState=true;


            RightFootLanded=true;//this variable is used for flag up when the left foot have a full contact with the ground
            SURENAOnlineTaskSpace1.RightSensorActive=false;
            firstcontact=true;
            //do nothing all sensors are on the ground
            SURENAOnlineTaskSpace1.RightFootOrientationAdaptator=false;


            Offset_teta_R=teta_motor_R;
            Offset_phi_R=phi_motor_R;
            //exit(0);//will finish the code but during walking is not true
        }

        else if(/*SURENAOnlineTaskSpace1.RightSensorActive==false*/ true) {//---------when the four left foot sensors are not saturated-->>>>during landing------//

            RightFootLanded=false;
            if ((e)>=footSensorthreshold ){eState=true;}else {eState=false;}
            if ((f)>=footSensorthreshold ){fState=true;}else {fState=false;}
            if ((g)>=footSensorthreshold ){gState=true;}else {gState=false;}
            if ((h)>=footSensorthreshold ){hState=true;}else {hState=false;}


            //-----------------Pitch left ankle motor control---------------//
            if (abs(f-e)>=abs(g-h)) {
                if (abs(e-f)<100) {
                    teta_motor_R=1*teta_motor_R+k3*(SURENAOnlineTaskSpace1.RightFootOrientationAdaptator==true)*(e-f);
                }
                else {
                    teta_motor_R=teta_motor_R;
                }
            }
            else {
                if (abs(h-g)<100) {
                    teta_motor_R=1*teta_motor_R+k3*(SURENAOnlineTaskSpace1.RightFootOrientationAdaptator==true)*(h-g);
                }
                else {
                    teta_motor_R=teta_motor_R;
                }
            }


            //----------------Roll left ankle motor control---------------//
            if (abs(g-f)>=abs(h-e)) {
                if (abs(g-f)<100) {
                    phi_motor_R=1*phi_motor_R+k4*(SURENAOnlineTaskSpace1.RightFootOrientationAdaptator==true)*(g-f);
                }
                else {
                    phi_motor_R=phi_motor_R;
                }
            }
            else {
                if (abs(e-h)<100) {
                    phi_motor_R=1*phi_motor_R+k4*(SURENAOnlineTaskSpace1.RightFootOrientationAdaptator==true)*(h-e);
                }
                else {
                    phi_motor_R=phi_motor_R;
                }
            }
        }





        //------------------------saturation of ankle motors----------------------------//
        if ((abs(phi_motor_L))>0.9) {
            phi_motor_L=0.9;
        }
        if ((abs(teta_motor_L))>0.9) {
            teta_motor_L=0.9;
        }
        if ((abs(phi_motor_R))>0.9) {
            phi_motor_L=0.9;
        }
        if ((abs(teta_motor_R))>0.9) {
            teta_motor_R=0.9;
        }


        // ROS_INFO("I heard data of sensors : [%f %f %f %f]",a,b,c,d);


        //-----------------------------------------------------------------------------------------------------//
        //-----------------start phase--initializing the height of pelvis for walking--------------------------//
        //-----------------------------------------------------------------------------------------------------//
        if (startPhase==true && StartTime<=DurationOfStartPhase) {
            MinimumJerkInterpolation Coef;
            MatrixXd ZPosition(1,2);
           // ZPosition<<SURENAOnlineTaskSpace1.InitialPelvisHeight,0.8600;//0.86 is referencePelvisHeight in task space online
            ZPosition<<SURENAOnlineTaskSpace1.InitialPelvisHeight,SURENAOnlineTaskSpace1.ReferencePelvisHeight;
            MatrixXd ZVelocity(1,2);
            ZVelocity<<0.000,0.000;
            MatrixXd ZAcceleration(1,2);
            ZAcceleration<<0.000,0.000;


            MatrixXd Time(1,2);
            Time<<0,DurationOfStartPhase;
            MatrixXd CoefZStart =Coef.Coefficient(Time,ZPosition,ZVelocity,ZAcceleration);

            double zStart=0;
            double yStart=0;
            double xStart=0;
            StartTime=StartTime+SURENAOnlineTaskSpace1._timeStep;

            MatrixXd outputZStart= SURENAOnlineTaskSpace1.GetAccVelPos(CoefZStart,StartTime,0,5);
            zStart=outputZStart(0,0);

            PoseRoot<<xStart,yStart,zStart,0,0,0;

            PoseRFoot<<0,
                    -0.11500,
                    0.112000,
                    0,
                    0,
                    0;

            PoseLFoot<<0,
                    0.11500,
                    0.11200,
                    0,
                    0,
                    0;

            SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot);
            SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot);

MinimumJerkInterpolation CoefOffline;
                double D_pitch=-1*pitchOffset*(M_PI/180);
                double TstartofPitchModify=DurationOfStartPhase/6;
                double TendofPitchModify=DurationOfStartPhase;
                double D_time=TendofPitchModify-TstartofPitchModify;
            if (StartTime>=TstartofPitchModify && StartTime<=TendofPitchModify){
                    MatrixXd Ct_pitch_st(1,2);
                    Ct_pitch_st<<0 ,D_time;
                    MatrixXd Cp_pitch_st(1,2);
                    Cp_pitch_st<<0, D_pitch;
                    MatrixXd Cv_pitch_st(1,2);
                    Cv_pitch_st<<0 ,0;
                    MatrixXd Ca_pitch_st(1,2);
                    Ca_pitch_st<<0 ,0;
                    MatrixXd C_pitch_st=CoefOffline.Coefficient(Ct_pitch_st,Cp_pitch_st,Cv_pitch_st,Ca_pitch_st);

                                    MatrixXd output=SURENAOnlineTaskSpace1.GetAccVelPos(C_pitch_st,StartTime-(TstartofPitchModify),0,5);
                        PitchModified=output(0,0);

            }


        }



        int NumberOfTimeStep=(SURENAOnlineTaskSpace1.Tc/SURENAOnlineTaskSpace1._timeStep)+1;

        //-----------------------------------------------------------------------------------------------------//
        //------------------------------- main loop of cyclic walking -----------------------------------------//
        //-----------------------------------------------------------------------------------------------------//

        if (StartTime>DurationOfStartPhase && StartTime<(DurationOfStartPhase+SURENAOnlineTaskSpace1.MotionTime)){

            double m1; //Ankle Trajectory Replacement
            double m2;
            double m3;
            double m4;
            double m5;
            double m6;
            double m7;
            double m8;

            StartTime=StartTime+SURENAOnlineTaskSpace1._timeStep;
            //qDebug()<<StartTime;

            if(walk==true){



                if ((SURENAOnlineTaskSpace1.StepNumber==1) && (SURENAOnlineTaskSpace1.localTiming>=SURENAOnlineTaskSpace1.TStart) ) {
                    //ROS_INFO(" Contact detected time: [%f]", SURENAOnlineTaskSpace1.localTiming);
                    SURENAOnlineTaskSpace1.localTiming=SURENAOnlineTaskSpace1._timeStep;//0.001999999999000000;
                    SURENAOnlineTaskSpace1.localtimingInteger=1;
                    SURENAOnlineTaskSpace1.StepNumber=SURENAOnlineTaskSpace1.StepNumber+1;
                    //  cout<<"cooontaaaaaactttt deeeeeteeeecteeeeddddddd="<<SURENAOnlineTaskSpace1.localTiming<<" step number= "<<SURENAOnlineTaskSpace1.StepNumber<<endl;
                    KLtemp=false;
                    SURENAOnlineTaskSpace1.CoeffArrayPelvisZMod();
                    //SURENAOnlineTaskSpace1.OldPelvisZ=SURENAOnlineTaskSpace1.NewPlevisZ;




                  //  if(firststep_sensor_test){
                   // SURENAOnlineTaskSpace1.LeftSensorActive==true; // was uncomment
                  //  firststep_sensor_test=false;}




                }



                else if ((SURENAOnlineTaskSpace1.localtimingInteger>=NumberOfTimeStep) &&   (SURENAOnlineTaskSpace1.StepNumber>1    &&   SURENAOnlineTaskSpace1.StepNumber<(SURENAOnlineTaskSpace1.NStep+2))) {
                    SURENAOnlineTaskSpace1.StepNumber=SURENAOnlineTaskSpace1.StepNumber+1;
                    SURENAOnlineTaskSpace1.CoeffArrayPelvisZMod();
                    SURENAOnlineTaskSpace1.localTiming=SURENAOnlineTaskSpace1._timeStep;//0.001999999999000000;
                    SURENAOnlineTaskSpace1.localtimingInteger=1;

                    if ((SURENAOnlineTaskSpace1.StepNumber%2)==0) {
                        KLtemp=false;
                    }
                    else {
                        KRtemp=false;
                    }
                }
                else if (indexLastDS==true && SURENAOnlineTaskSpace1.localTiming>=SURENAOnlineTaskSpace1.TDs && SURENAOnlineTaskSpace1.StepNumber==SURENAOnlineTaskSpace1.NStep+2) {
                    // ROS_INFO(" Contact detected time: [%f]", SURENAOnlineTaskSpace1.localTiming);
                    SURENAOnlineTaskSpace1.localTiming=0.00200000000000;
                    SURENAOnlineTaskSpace1.localtimingInteger=1;
                    indexLastDS=false;
                    KLtemp=false;

                }

                else if (indexLastDS==false && SURENAOnlineTaskSpace1.localTiming>( 0.5*SURENAOnlineTaskSpace1.TEnd)) {
                    KRtemp=false;
                }



                if (KLtemp==false) {
                    KLtemp=true;
                    SURENAOnlineTaskSpace1.currentLeftFootX2=links[12].PositionInWorldCoordinate(0);
                    SURENAOnlineTaskSpace1.currentLeftFootY2=links[12].PositionInWorldCoordinate(1);
                    SURENAOnlineTaskSpace1.currentLeftFootZ=links[12].PositionInWorldCoordinate(2);
                }

                if (KRtemp==false) {
                    KRtemp=true;
                    SURENAOnlineTaskSpace1.currentRightFootX2=links[6].PositionInWorldCoordinate(0);
                    SURENAOnlineTaskSpace1.currentRightFootY2=links[6].PositionInWorldCoordinate(1);
                    SURENAOnlineTaskSpace1.currentRightFootZ=links[6].PositionInWorldCoordinate(2);
                }

//replace false with following commented for activing sensor
              //  if (  /*(RightFootLanded==true)*/ false &&  ( SURENAOnlineTaskSpace1.StepNumber==1 || SURENAOnlineTaskSpace1.localTiming>(SURENAOnlineTaskSpace1.TDs+SURENAOnlineTaskSpace1.TSS/2)   /*|| SURENAOnlineTaskSpace1.StepNumber==(SURENAOnlineTaskSpace1.NStep+2)*/)) {

                if (  (RightFootLanded==true)  &&  ( SURENAOnlineTaskSpace1.StepNumber==1 || SURENAOnlineTaskSpace1.localTiming>(SURENAOnlineTaskSpace1.TDs+SURENAOnlineTaskSpace1.TSS/2)   /*|| SURENAOnlineTaskSpace1.StepNumber==(SURENAOnlineTaskSpace1.NStep+2)*/)) {

                    SURENAOnlineTaskSpace1.oldRightFootX2= SURENAOnlineTaskSpace1.currentRightFootX2;
                    SURENAOnlineTaskSpace1.oldRightFootY2= SURENAOnlineTaskSpace1.currentRightFootY2;
                    SURENAOnlineTaskSpace1.oldRightFootZ= SURENAOnlineTaskSpace1.currentRightFootZ;

                    SURENAOnlineTaskSpace1.currentRightFootX2=links[6].PositionInWorldCoordinate(0);
                    SURENAOnlineTaskSpace1.currentRightFootY2=links[6].PositionInWorldCoordinate(1);
                    SURENAOnlineTaskSpace1.currentRightFootZ=links[6].PositionInWorldCoordinate(2);
                   // ROS_INFO("Right foot Z height early contact offset: [%f ]", (SURENAOnlineTaskSpace1.currentRightFootZ-SURENAOnlineTaskSpace1._lenghtOfAnkle));
                    // ROS_INFO("Right foot Z height early contact offset: [%f ]", (SURENAOnlineTaskSpace1.currentRightFootX2-SURENAOnlineTaskSpace1._lenghtOfAnkle));
                   //SURENAOnlineTaskSpace1.NewPlevisZ =SURENAOnlineTaskSpace1.OldPelvisZ+(SURENAOnlineTaskSpace1.currentRightFootZ-SURENAOnlineTaskSpace1.OldPelvisZ);
                     SURENAOnlineTaskSpace1.NewPlevisZ =SURENAOnlineTaskSpace1.OldPelvisZ+(SURENAOnlineTaskSpace1.currentRightFootZ-SURENAOnlineTaskSpace1.oldRightFootZ);
                   RFT=true;

                }
                else if (SURENAOnlineTaskSpace1.localTiming<(SURENAOnlineTaskSpace1.TDs+SURENAOnlineTaskSpace1.TSS/2) ) {
                    RFT=false;
                }


//replace false with following commented for activing sensro
              // if ((  /*(LeftFootLanded==true)*/ false && SURENAOnlineTaskSpace1.localTiming>(SURENAOnlineTaskSpace1.TDs+SURENAOnlineTaskSpace1.TSS/2) )) {
                    if ((  (LeftFootLanded==true) && SURENAOnlineTaskSpace1.localTiming>(SURENAOnlineTaskSpace1.TDs+SURENAOnlineTaskSpace1.TSS/2) )) {

                    SURENAOnlineTaskSpace1.oldLeftFootX2= SURENAOnlineTaskSpace1.currentLeftFootX2;
                    SURENAOnlineTaskSpace1.oldLeftFootY2= SURENAOnlineTaskSpace1.currentLeftFootY2;
                    SURENAOnlineTaskSpace1.oldLeftFootZ= SURENAOnlineTaskSpace1.currentLeftFootZ;

                    SURENAOnlineTaskSpace1.currentLeftFootX2=links[12].PositionInWorldCoordinate(0);
                    SURENAOnlineTaskSpace1.currentLeftFootY2=links[12].PositionInWorldCoordinate(1);
                    SURENAOnlineTaskSpace1.currentLeftFootZ=links[12].PositionInWorldCoordinate(2);
                    //SURENAOnlineTaskSpace1.NewPlevisZ =SURENAOnlineTaskSpace1.OldPelvisZ+(SURENAOnlineTaskSpace1.currentLeftFootZ-SURENAOnlineTaskSpace1.OldPelvisZ);
                   // ROS_INFO("left foot Z height early contact offset: [%f ]", (SURENAOnlineTaskSpace1.currentLeftFootZ-SURENAOnlineTaskSpace1._lenghtOfAnkle));
                     SURENAOnlineTaskSpace1.NewPlevisZ =SURENAOnlineTaskSpace1.OldPelvisZ+(SURENAOnlineTaskSpace1.currentLeftFootZ-SURENAOnlineTaskSpace1.oldLeftFootZ);
                    LFT=true;
                }
                else if ( SURENAOnlineTaskSpace1.localTiming<(SURENAOnlineTaskSpace1.TDs+SURENAOnlineTaskSpace1.TSS/2) ) {
                    LFT=false;
                }


                MatrixXd m=SURENAOnlineTaskSpace1.AnkleTrajectory(SURENAOnlineTaskSpace1.globalTime,SURENAOnlineTaskSpace1.StepNumber,SURENAOnlineTaskSpace1.localTiming,RFT,LFT,indexLastDS);
                m1=m(0,0);
                m2=m(1,0);
                m3=m(2,0);
                m4=m(3,0);
                m5=m(4,0);
                m6=m(5,0);
                m7=m(6,0);
                m8=m(7,0);


                Pz=SURENAOnlineTaskSpace1.ModificationOfPelvisHeight(SURENAOnlineTaskSpace1.globalTime,SURENAOnlineTaskSpace1.StepNumber,SURENAOnlineTaskSpace1.localTiming,RFT,LFT,indexLastDS);

                RollModified=SURENAOnlineTaskSpace1.RollAngleModification(SURENAOnlineTaskSpace1.globalTime,SURENAOnlineTaskSpace1.StepNumber,SURENAOnlineTaskSpace1.localTiming,indexLastDS);
                P=SURENAOnlineTaskSpace1.PelvisTrajectory (SURENAOnlineTaskSpace1.globalTime,SURENAOnlineTaskSpace1.StepNumber,SURENAOnlineTaskSpace1.localTiming,indexLastDS);

                SURENAOnlineTaskSpace1.globalTime=SURENAOnlineTaskSpace1.globalTime+SURENAOnlineTaskSpace1._timeStep;
                SURENAOnlineTaskSpace1.localTiming=SURENAOnlineTaskSpace1.localTiming+SURENAOnlineTaskSpace1._timeStep;
                SURENAOnlineTaskSpace1.localtimingInteger= SURENAOnlineTaskSpace1.localtimingInteger+1;


                if (round(SURENAOnlineTaskSpace1.globalTime)<=round(SURENAOnlineTaskSpace1.MotionTime)){


 //if you want to have modification of height of pelvis please active the Pz(0,0) instead of P(2,0)
                    PoseRoot<<P(0,0),
                            P(1,0),
                            Pz(0,0),
                            0,
                            0,
                            0;

                    PoseRFoot<<m5,
                            m6,
                            m7,
                            0,
                            -1*m8*(M_PI/180),
                            0;

                    PoseLFoot<<m1,
                            m2,
                            m3,
                            0,
                            -1*m4*(M_PI/180),
                            0;


                    SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot);
                    SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot);

                    SURENA.ForwardKinematic(1);
                }
            }
        }

        //-----------------------------------------------------------------------------------------------------//
        //------------------- end phase-- finializing height of pelvis to home position -----------------------//
        //-----------------------------------------------------------------------------------------------------//

        if (endPhase==true &&  StartTime>=(DurationOfStartPhase+SURENAOnlineTaskSpace1.MotionTime) && StartTime<=DurationOfendPhase+DurationOfStartPhase+SURENAOnlineTaskSpace1.MotionTime) {

            MinimumJerkInterpolation Coef;
            MatrixXd ZPosition(1,2);
            //ZPosition<<Pz(0,0),Pz(0,0)-SURENAOnlineTaskSpace1.ReferencePelvisHeight+SURENAOnlineTaskSpace1.InitialPelvisHeight;//this one should be edited
            ZPosition<<SURENAOnlineTaskSpace1.ReferencePelvisHeight,SURENAOnlineTaskSpace1.InitialPelvisHeight;//this one should be edited
            MatrixXd ZVelocity(1,2);
            ZVelocity<<0.000,0.000;
            MatrixXd ZAcceleration(1,2);
            ZAcceleration<<0.000,0.000;

            MatrixXd Time(1,2);
            Time<<DurationOfStartPhase+SURENAOnlineTaskSpace1.MotionTime,DurationOfStartPhase+SURENAOnlineTaskSpace1.MotionTime+DurationOfendPhase;
            MatrixXd CoefZStart =Coef.Coefficient(Time,ZPosition,ZVelocity,ZAcceleration);

            double zStart=0;
            double yStart=0;
            double xStart=0;
            StartTime=StartTime+SURENAOnlineTaskSpace1._timeStep;

            MatrixXd outputZStart= SURENAOnlineTaskSpace1.GetAccVelPos(CoefZStart,StartTime,DurationOfStartPhase+SURENAOnlineTaskSpace1.MotionTime,5);
            zStart=outputZStart(0,0);

            PoseRoot<<xStart,yStart,zStart,0,0,0;

            PoseRFoot<<0,
                    -0.11500,
                    SURENAOnlineTaskSpace1.currentRightFootZ,
                    0,
                    0,
                    0;

            PoseLFoot<<0,
                    0.11500,
                    SURENAOnlineTaskSpace1.currentLeftFootZ,
                    0,
                    0,
                    0;

            SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot);
            SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot);


            MinimumJerkInterpolation CoefOffline;
            double D_pitch=-1*pitchOffset*(M_PI/180);
            double TstartofPitchModify=DurationOfendPhase/6+DurationOfStartPhase+SURENAOnlineTaskSpace1.MotionTime;
            double TendofPitchModify=DurationOfendPhase*5/6+DurationOfStartPhase+SURENAOnlineTaskSpace1.MotionTime;
            double D_time=TendofPitchModify-TstartofPitchModify;
            if (StartTime>=TstartofPitchModify && StartTime<=TendofPitchModify){
                MatrixXd Ct_pitch_st(1,2);
                Ct_pitch_st<<-D_time, 0;
                MatrixXd Cp_pitch_st(1,2);
                Cp_pitch_st<<D_pitch, 0;
                MatrixXd Cv_pitch_st(1,2);
                Cv_pitch_st<<0 ,0;
                MatrixXd Ca_pitch_st(1,2);
                Ca_pitch_st<<0 ,0;
                MatrixXd C_pitch_st=CoefOffline.Coefficient(Ct_pitch_st,Cp_pitch_st,Cv_pitch_st,Ca_pitch_st);

                MatrixXd output=SURENAOnlineTaskSpace1.GetAccVelPos(C_pitch_st,StartTime-(TendofPitchModify),-D_time,5);
                PitchModified=output(0,0);

            }



        }


        links = SURENA.GetLinks();
        if(links[5].JointAngle<min_test){min_test=links[5].JointAngle;}
        if(links[5].JointAngle>max_test){max_test=links[5].JointAngle;}
        ROS_INFO("ankl pith min=%f,max=%f",min_test*180/M_PI,max_test*180/M_PI);




        //SURENAOnlineTaskSpace1.RightFootOrientationAdaptator=false;
        //SURENAOnlineTaskSpace1.LeftFootOrientationAdaptator=false;

        //        else {//else  is for situation that sensor is contacting and adapting ground
double ankle_adaptation_switch=0;// 1 for activating adaptation 0 for siktiring adaptation
double k_roll_corr=0;
        cntrl[0]=0.0;
        cntrl[1]=(links[1].JointAngle);
        cntrl[2]=(links[2].JointAngle+k_roll_corr*(links[2].JointAngle-roll_absoulte[0])+1*RollModified(0,0));
        cntrl[3]=links[3].JointAngle+1.25*PitchModified;
        cntrl[4]=links[4].JointAngle;
//        cntrl[5]=links[5].JointAngle+ankle_adaptation_switch*((SURENAOnlineTaskSpace1.RightFootOrientationAdaptator==true)*teta_motor_R+Offset_teta_R);//pitch
//        cntrl[6]=links[6].JointAngle+ankle_adaptation_switch*((SURENAOnlineTaskSpace1.RightFootOrientationAdaptator==true)*phi_motor_R+Offset_phi_R);//roll
        cntrl[5]=links[5].JointAngle+ankle_adaptation_switch*teta_motor_R;//pitch
        cntrl[6]=links[6].JointAngle+ankle_adaptation_switch*(phi_motor_R);//roll

        cntrl[7]=links[7].JointAngle;
        cntrl[8]=links[8].JointAngle+k_roll_corr*(links[8].JointAngle-roll_absoulte[1])+1*RollModified(1,0);
        cntrl[9]=links[9].JointAngle+1.25*PitchModified;
        cntrl[10]=links[10].JointAngle;
//        cntrl[11]=links[11].JointAngle+ankle_adaptation_switch*((SURENAOnlineTaskSpace1.LeftFootOrientationAdaptator==true)*teta_motor_L+Offset_teta_L);
//        cntrl[12]=links[12].JointAngle+ankle_adaptation_switch*((SURENAOnlineTaskSpace1.LeftFootOrientationAdaptator==true)*phi_motor_L+Offset_phi_L);
        cntrl[11]=links[11].JointAngle+ankle_adaptation_switch*teta_motor_L;
        cntrl[12]=links[12].JointAngle+ankle_adaptation_switch*(phi_motor_L);

        //        }
       // ROS_INFO("teta_R=%f,Offset_teta_R=%f,phi_R=%f,Offset_phi_R=%f,teta_L=%f,Offset_teta_L=%f,phi_L=%f,Offset_phi_L=%f",teta_motor_R,Offset_teta_R,phi_motor_R,Offset_phi_R,teta_motor_L,Offset_teta_L,phi_motor_L,Offset_phi_L);
//please uncomment /*SURENAOnlineTaskSpace1.LeftFootOrientationAdaptator==true* for activing sensor and also replace 0 with  1 in above code


        vector<int> qref(12);
        qref=QC.ctrldata2qc(cntrl);

        msg.data.clear();
        bool left_first=true;//right support in first step
        if(left_first){
            for(int  i = 0;i < 12;i++)
            {
                msg.data.push_back(qref[i]+qc_offset[i]);
            }}
        else{
            msg.data.push_back(-qref[5]+qc_offset[0]);
            msg.data.push_back(-qref[4]+qc_offset[1]);
            msg.data.push_back(-qref[6]+qc_offset[2]);
            msg.data.push_back(-qref[7]+qc_offset[3]);
            msg.data.push_back(-qref[1]+qc_offset[4]);
            msg.data.push_back(-qref[0]+qc_offset[5]);
            msg.data.push_back(-qref[2]+qc_offset[6]);
            msg.data.push_back(-qref[3]+qc_offset[7]);
            msg.data.push_back(-qref[11]+qc_offset[8]);
            msg.data.push_back(-qref[10]+qc_offset[9]);
            msg.data.push_back(-qref[9]+qc_offset[10]);
            msg.data.push_back(-qref[8]+qc_offset[11]);
        }

        for(int  i = 12;i < 28;i++)
        {
            msg.data.push_back(0);
        }

       // SendGazebo(links,RollModified,PitchModified,teta_motor_R+0*Offset_teta_R,phi_motor_R+0*Offset_phi_R,teta_motor_L+0*Offset_teta_L,phi_motor_L+0*Offset_phi_L);
        chatter_pub.publish(msg);

        msg_contact_flag.data.clear();
        msg_contact_flag.data.push_back(contact_flag_timing);
        msg_contact_flag.data.push_back(contact_flag_sensor);
        msg_contact_flag.data.push_back(contact_flag_sensor2);

        contact_flag.publish(msg_contact_flag);
        //  ROS_INFO("t={%d} c={%d}",timer.elapsed(),count);
       if(count%20==0){
//           ROS_INFO("right:des=%f,abs=%f,diff=%f (inc=%d),left:des=%f,abs=%f,diff=%f (inc=%d)",links[2].JointAngle,
//                   roll_absoulte[0],links[2].JointAngle-roll_absoulte[0],int((links[2].JointAngle-roll_absoulte[0])*120*2340/2/M_PI),
//                   links[8].JointAngle,roll_absoulte[1],
//                   links[8].JointAngle-roll_absoulte[1],int((links[8].JointAngle-roll_absoulte[1])*120*2340/2/M_PI));
       //ROS_INFO("time=%f\tRollModifiedRight=%f RollModifiedLeft=%f",StartTime,RollModified(0,0)*180/M_PI,RollModified(1,0)*180/M_PI);
//ROS_INFO("rightpitch=%f , leftpitch=%f",cntrl[3],cntrl[9]);
       }
    //    ROS_INFO("q=%d",qref[9]);
        //ROS_INFO("q=%d",qref[9]);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}


