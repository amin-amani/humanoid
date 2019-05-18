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

QTime timer;
int count;


int getch()
{
//    timer.start();

//    while( timer.elapsed()<3){
        static struct termios oldt, newt;
        tcgetattr( STDIN_FILENO, &oldt);           // save old settings
        newt = oldt;
        newt.c_lflag &= ~(ICANON);                 // disable buffering
        tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

        int c = getchar();  // read character (non-blocking)

        tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
        return c;
//    }
//return 10000;
}



using namespace  std;

int main(int argc, char **argv)
{

int count = 0;
double t=0;
double dt=.005;

ros::init(argc, argv, "myNode");
ros::NodeHandle nh;
ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",1000);
ros::Rate loop_rate(200);

std_msgs::Int32MultiArray msg;
std_msgs::MultiArrayDimension msg_dim;

msg_dim.label = "joint_position";
msg_dim.size = 1;
msg.layout.dim.clear();
msg.layout.dim.push_back(msg_dim);

int q[8];
for (int var = 0; var < 8; ++var) {
    q[var]=0;
}

while (ros::ok())
{

 msg.data.clear();
int p=0;
//p=int(1000*sin(t*2*M_PI/4));



//int q;
// int dq;
//12*37.5;
int s=-int(6*75*(1-cos(t*2*M_PI/8)));
//dq=int(cos(t*2*M_PI/4)/abs(cos(t*2*M_PI/4)));
//if (dq==1){ROS_INFO("%d , positive",q);}
//if (dq==-1){ROS_INFO("%d , negative",q);}




count++;
int c=0;
if(count>20) {
    ROS_INFO("`");
     c= getch();
    count=0;
}

if(c==113){break;}
if(c==97) {q[0]+=4;}if(c==122){q[0]-=4;}
if(c==115){q[1]+=4;}if(c==120){q[1]-=4;}
if(c==100){q[2]+=4;} if(c==99){q[2]-=4;}
if(c==102){q[3]+=4;}if(c==118){q[3]-=4;}
if(c==103){q[4]+=4;} if(c==98){q[4]-=4;}
if(c==104){q[5]+=4;}if(c==110){q[5]-=4;}
if(c==106){q[6]+=4;}if(c==109){q[6]-=4;}
if(c==107){q[7]+=4;} if(c==44){q[7]-=4;}


for(int  i = 0;i < 12;i++)
{
    msg.data.push_back(p);
}
//right hand epose
msg.data.push_back(q[0]);//12 -y  a,z
msg.data.push_back(q[1]);//13 +x
msg.data.push_back(q[2]);//14 -z
//msg.data.push_back(q[3]);//15 +y
msg.data.push_back(q[3]);

//right hand dynamixel + fingers
msg.data.push_back(0);//16
msg.data.push_back(0);//17
msg.data.push_back(0);//18
msg.data.push_back(6);//19
//left hand epose
msg.data.push_back(q[4]);//20 +y
msg.data.push_back(q[5]);//21 +x
msg.data.push_back(q[6]);//22 -z
msg.data.push_back(q[7]);//23 -y
//left hand dynamixel + fingers
msg.data.push_back(0);//24
msg.data.push_back(0);//25
msg.data.push_back(0);//26
msg.data.push_back(6);//27

//for(int i= 12;i < 16;i++){

//    msg.data.push_back(q);
//}

//for(int  i = 16;i < 20;i++)
//{
//    msg.data.push_back(0);
//}

//for(int i= 20;i < 24;i++){

//    msg.data.push_back(q);
//}

//for(int  i = 24;i < 28;i++)
//{
//    msg.data.push_back(0);
//}

    chatter_pub.publish(msg);





ros::spinOnce();
loop_rate.sleep();
++count;
t+=dt;
}


return 0;
}
