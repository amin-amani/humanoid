#include "ros/ros.h"
#include "std_msgs/String.h"
#include"Eigen/Dense"
#include <vector>
#include <iostream>
#include <QString>
#include <QList>
#include "Robot.h"
//#include"TaskSpace.h"
#include"taskspaceoffline.h"
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
#include <gazebo_msgs/LinkStates.h>
#include<termios.h>
#include<QTime>
#include<geometry_msgs/Wrench.h>
#include<sensor_msgs/JointState.h>
#include "qcgenerator.h"
#include<termios.h>

using namespace  std;
using namespace Eigen;
QTime timer;
int count;


double saturate(double a, double min, double max){
    if(a<min){return min;}
    else if(a>max){return max;}
    else{return a;}
}

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

int qc_offset[12];
bool qc_initial_bool;


void qc_initial(const sensor_msgs::JointState & msg){
    if (qc_initial_bool){

        for (int i = 0; i < 12; ++i) {
            qc_offset[i]=int(msg.position[i+1]);

        }

        qc_initial_bool=false;

        ROS_INFO("Offset=%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\nInitialized!",
                 qc_offset[0],qc_offset[1],qc_offset[2],qc_offset[3],qc_offset[4],
                qc_offset[5],qc_offset[6],qc_offset[7],qc_offset[8],qc_offset[9],
                qc_offset[10],qc_offset[11]);}
    }


double Fxl,Fyr, Fzl,Fzr,Mxl,Myl,Myr,Mxr;
VectorXd CoPR(2), CoPL(2);
void FT_left_feedback(const geometry_msgs::Wrench &msg){
 Fzl=msg.force.z;
 Mxl=-msg.torque.y;
 Myl=msg.torque.x;
 CoPL << Myl/Fzl,
         Mxl/Fzl;

}

void FT_right_feedback(const geometry_msgs::Wrench &msg){
 Fzr=msg.force.z;
 Mxr=-msg.torque.x;
 Myr=-msg.torque.y;
 CoPR << Myr/Fzr,
         Mxr/Fzr;

}


double IMU_roll,IMU_pitch,IMU_yaw;

void IMU_feedback(const sensor_msgs::Imu &msg){
IMU_roll=msg.orientation.x;
IMU_pitch=msg.orientation.y;
IMU_yaw=msg.orientation.z;
}



int main(int argc, char **argv)
{
double d=M_PI/180/20;//0.05 degree
    qc_initial_bool=true;
    QCgenerator QC;
    for (int i = 0; i < 12; ++i) {
        qc_offset[i]=0;
    }


int count = 0;
double t=0;
double dt=.005;

ros::init(argc, argv, "myNode");
ros::NodeHandle nh;
ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",1000);
ros::Subscriber ft_left = nh.subscribe("/surena/ft_l_state",1000,FT_left_feedback);
ros::Subscriber ft_right = nh.subscribe("/surena/ft_r_state",1000,FT_right_feedback);
ros::Subscriber qcinit = nh.subscribe("/surena/inc_joint_state", 1000, qc_initial);
ros::Subscriber imu = nh.subscribe("/surena/imu_state",1000,IMU_feedback);

ros::Rate loop_rate(200);


std_msgs::Int32MultiArray msg;
std_msgs::MultiArrayDimension msg_dim;

msg_dim.label = "joint_position";
msg_dim.size = 1;
msg.layout.dim.clear();
msg.layout.dim.push_back(msg_dim);

vector<double> q(13);
for (int var = 0; var < 13; ++var) {
    q[var]=0;
}
double qkamar=0;
qDebug()<<"press any key to start!";
getch();
qDebug()<<"started!";

while (ros::ok())
{


    if (qc_initial_bool) {
        ROS_INFO_ONCE("qc is initializing!");
        ros::spinOnce();
        continue;
    }





count++;
int c=0;
//if(count>5) {
//    c= getch();
//    count=0;
//    ROS_INFO("ascii=%d",c);
//}
qDebug()<<"\nright: HY='6,y' HR='5,t' HP='4,r' K='3,e' AP='2,w' AR='1,q'\nleft: HY='h,n' HR='g,b' HP='f,v' K='d,c' AP='s,x' AR='a,z'\nWP='/,'\t'`' for quit!";
double Fz=Fzl+Fzr;
double My=Myl+Myr;



c= getch();
qDebug()<<c;

if(c==96){break;}

if(c==49){q[7-1]+=d;}if(c==113){q[7-1]-=d;}
if(c==50){q[7-2]+=d;}if(c==119){q[7-2]-=d;}
if(c==51){q[7-3]+=d;}if(c==101){q[7-3]-=d;}
if(c==52){q[7-4]+=d;}if(c==114){q[7-4]-=d;}
if(c==53){q[7-5]+=d;}if(c==116){q[7-5]-=d;}
if(c==54){q[7-6]+=d;}if(c==121){q[7-6]-=d;}

if(c==97) {q[19-7]+=d;}   if(c==122){q[19-7]-=d;}
if(c==115){q[19-8]+=d;}   if(c==120){q[19-8]-=d;}
if(c==100){q[19-9]+=d;}    if(c==99){q[19-9]-=d;}
if(c==102){q[19-10]+=d;}  if(c==118){q[19-10]-=d;}
if(c==103){q[19-11]+=d;}   if(c==98){q[19-11]-=d;}
if(c==104){q[19-12]+=d;}  if(c==110){q[19-12]-=d;}

if(c==39){qkamar+=d;}  if(c==47){qkamar-=d;}
if(c==46){q[7-4]+=d;q[19-10]+=d;}  if(c==59){q[7-4]-=d;q[19-10]-=d;}


ROS_INFO("\nright: HY=%f HR=%f HP=%f K=%f AP=%f AR=%f\nleft: HY=%f HR=%f HP=%f K=%f AP=%f AR=%f WP=%f",
         q[1]*180/M_PI,q[2]*180/M_PI,q[3]*180/M_PI,q[4]*180/M_PI,q[5]*180/M_PI,q[6]*180/M_PI,
        q[7]*180/M_PI,q[8]*180/M_PI,q[9]*180/M_PI,q[10]*180/M_PI,q[11]*180/M_PI,q[12]*180/M_PI,qkamar*180/M_PI);
ROS_INFO("CoPL=(%f\t%f),Mxl=%f,Myl=%f,Fzl=%f",CoPL(0),CoPL(1),Mxl,Myl,Fzl);
ROS_INFO("CoPR=(%f\t%f),MxR=%f,MyR=%f,Fzr=%f",CoPR(0),CoPR(1),Mxr,Myr,Fzr);
ROS_INFO("CoPx=%f,My=%f,Fz=%f",My/Fz,My,Fz);
ROS_INFO("roll=%f,pitch=%f,yaw=%f",IMU_roll,IMU_pitch,IMU_yaw);
vector<int> qref(12);
qref=QC.ctrldata2qc(q);

int qrefkamar=int(qkamar*(1/(2*M_PI))*(2304)*50);


 msg.data.clear();
for(int  i = 0;i < 12;i++)
{
    msg.data.push_back(qref[i]+qc_offset[i]);
}
for(int  i = 12;i < 28;i++)
{
    msg.data.push_back(0);
}
msg.data.push_back(qrefkamar);



    chatter_pub.publish(msg);





ros::spinOnce();
loop_rate.sleep();
++count;
t+=dt;
}


return 0;
}
