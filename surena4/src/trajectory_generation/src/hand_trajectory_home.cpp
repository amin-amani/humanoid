
#include "ros/ros.h"
#include "std_msgs/String.h"
#include"Eigen/Dense"
#include <vector>
#include <iostream>
#include <QString>
#include <QList>
#include "Robot.h"
//#include"TaskSpace.h"
#include"MinimumJerkInterpolation.h"
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
#include "right_hand.h"
#include "left_hand.h"
#include <sensor_msgs/JointState.h>
#include <cstdlib>
#include<termios.h>
#include <iostream>
#include <QTime>

using namespace  std;
using namespace  Eigen;

void matrix_view(MatrixXd M){
    for (int i = 0; i <M.rows() ; ++i) {
        QString str;
        for (int j = 0; j <M.cols() ; ++j) {str+=QString::number(M(i,j));str+="   ";}
        qDebug()<<str;}qDebug()<<"";}

void matrix_view(VectorXd M){
    QString str;
    for (int i = 0; i <M.rows() ; ++i) {str+=QString::number(M(i));str+="   ";}
    qDebug()<<str;qDebug()<<"";}

double move2pose(double max,double t_local,double T_start ,double T_end){
    double T_move=T_end-T_start;
    double c3=(10*max)/pow(T_move,3);
    double c4=-(15*max)/pow(T_move,4);
    double c5=(6*max)/pow(T_move,5);
    double theta=0;
    if(t_local<T_start){theta=0;}
    else if (t_local<T_end){theta=c3*pow(t_local-T_start,3)+c4*pow(t_local-T_start,4)+c5*pow(t_local-T_start,5);}
    else{theta=max;}
    return theta;
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
int qra_offset[4];
int qla_offset[4];
bool qc_initial_bool;
void qc_initial(const sensor_msgs::JointState & msg){
    if (qc_initial_bool){

        for (int i = 0; i < 12; ++i) {
            qc_offset[i]=int(msg.position[i+1]);

        }

        for (int i = 12; i < 16; ++i){
            qra_offset[i-12]=int(msg.position[i+1]);
        }

        for (int i = 20; i < 24; ++i){
            qla_offset[i-20]=int(msg.position[i+1]);
        }


        qc_initial_bool=false;

        ROS_INFO("Offset_feet=%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\n",
                 qc_offset[0],qc_offset[1],qc_offset[2],qc_offset[3],qc_offset[4],
                qc_offset[5],qc_offset[6],qc_offset[7],qc_offset[8],qc_offset[9],
                qc_offset[10],qc_offset[11]);
        ROS_INFO("Offset_righthand=%d\t%d\t%d\t%d",qra_offset[0],qra_offset[1], qra_offset[2],qra_offset[3]);
        ROS_INFO("Offset_lefthand=%d\t%d\t%d\t%d",qla_offset[0],qla_offset[1], qla_offset[2],qla_offset[3]);
        ROS_INFO("Initialized!");
    }
}

VectorXd absolute_q0_r(7);VectorXd absolute_q0_l(7);
bool abs_initial_bool;


void abs_read(const sensor_msgs::JointState & msg){
    if (abs_initial_bool){

        VectorXd absolute_sensor_r(7);VectorXd absolute_sensor_l(7);
        for (int i = 0; i < 7; ++i) {
            absolute_sensor_r(i)=msg.position[i+13];            absolute_q0_r(i)=0;
            absolute_sensor_l(i)=msg.position[i+13+8];            absolute_q0_l(i)=0;
        }
        //right hand
        absolute_q0_r(0)=double(absolute_sensor_r(0)+863)/8192*2*M_PI;
        absolute_q0_r(1)=double(absolute_sensor_r(1)-(-889+2275))/8192*2*M_PI;
        absolute_q0_r(2)=-double(absolute_sensor_r(2)-2881)/8192*2*M_PI;
        absolute_q0_r(3)=double(absolute_sensor_r(3)-2058)/8192*2*M_PI;
        //    //left hand
        absolute_q0_l(0)=double(absolute_sensor_l(0)-705)/8192*2*M_PI;
        absolute_q0_l(1)=double(absolute_sensor_l(1)-(3642-2275))/8192*2*M_PI;
        absolute_q0_l(2)=-double(absolute_sensor_l(2)+344)/8192*2*M_PI;
        absolute_q0_l(3)=-double(absolute_sensor_l(3)-47)/8192*2*M_PI;
        qDebug()<<"absolutes:";
        matrix_view(absolute_sensor_r);matrix_view(absolute_sensor_l);
        qDebug()<<"positions:";
        VectorXd absolute_q0_r_deg(7);
        absolute_q0_r_deg=180/M_PI*absolute_q0_r;
        matrix_view(absolute_q0_r_deg);
        VectorXd absolute_q0_l_deg(7);
        absolute_q0_l_deg=180/M_PI*absolute_q0_l;
        matrix_view(absolute_q0_l_deg);

        abs_initial_bool=false;
    }



}

int main(int argc, char **argv)
{

    abs_initial_bool=true;
    qc_initial_bool=true;

    ros::init(argc, argv, "mynode");

    ros::NodeHandle nh;

    ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",1000);
    ros::Subscriber abs_sensor = nh.subscribe("/surena/abs_joint_state", 1000, abs_read);
    ros::Subscriber qcinit = nh.subscribe("/surena/inc_joint_state", 1000, qc_initial);

    std_msgs::Int32MultiArray msg;
    std_msgs::MultiArrayDimension msg_dim;

    msg_dim.label = "joint_position";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);


    VectorXd q_ra(7) ;VectorXd q_la(7);
q_ra.fill(0);
q_la.fill(0);

    int q_motor_r[8];int q_motor_l[8];
    for (int var = 0; var < 8; ++var) {
        q_motor_r[var]=0;q_motor_l[var]=0;
    }


    ros::Rate loop_rate(200);
    int count = 0;
    double time=0.0;

    VectorXd qr_home(7);
    VectorXd ql_home(7);

    qr_home<<10*M_PI/180,-10*M_PI/180,0,-20*M_PI/180, 0,0,0;
    ql_home<<10*M_PI/180, 10*M_PI/180,0,-20*M_PI/180, 0,0,0;

bool initializing=true;
    while (ros::ok())
    {

        if (abs_initial_bool) {
            ROS_INFO_ONCE("abs is initializing!");
            ros::spinOnce();
            continue;
        }

        if (qc_initial_bool) {
            ROS_INFO_ONCE("qc is initializing!");
            ros::spinOnce();
            continue;
        }

        if(initializing){
        ROS_INFO("press any key to start!");
        getch();
        initializing=false;
    }

        time=double(count)*.005;
        for (int i = 0; i < 4; ++i) {
            q_ra(i)=absolute_q0_r(i)+move2pose(qr_home(i)-absolute_q0_r(i),time,0,3);
            q_la(i)=absolute_q0_l(i)+move2pose(ql_home(i)-absolute_q0_l(i),time,0,3);
        }
        q_motor_r[0]=-int(10*(q_ra(0)-absolute_q0_r(0))*180/M_PI*120/60)+qra_offset[0];
        q_motor_r[1]= int(10*(q_ra(1)-absolute_q0_r(1))*180/M_PI*120/60)+qra_offset[1];
        q_motor_r[2]= -int(7*(q_ra(2)-absolute_q0_r(2))*180/M_PI*100/60)+qra_offset[2];
        q_motor_r[3]=  int(7*(q_ra(3)-absolute_q0_r(3))*180/M_PI*100/60)+qra_offset[3];

        q_motor_r[4]=0;
        q_motor_r[5]=0;
        q_motor_r[6]=0;
        q_motor_r[7]=9;

        q_motor_l[0]=int(10*(q_la(0)-absolute_q0_l(0))*180/M_PI*120/60)+qla_offset[0];
        q_motor_l[1]=int(10*(q_la(1)-absolute_q0_l(1))*180/M_PI*120/60)+qla_offset[1];
        q_motor_l[2]=-int(7*(q_la(2)-absolute_q0_l(2))*180/M_PI*100/60)+qla_offset[2];
        q_motor_l[3]=-int(7*(q_la(3)-absolute_q0_l(3))*180/M_PI*100/60)+qla_offset[3];

        q_motor_l[4]=0;
        q_motor_l[5]=0;
        q_motor_l[6]=0;
        q_motor_l[7]=9;

        msg.data.clear();
        for(int  i = 0;i < 12;i++)
        {
            msg.data.push_back(qc_offset[i]);
        }
        //right hand epose
        msg.data.push_back(q_motor_r[0]);//12 -y  a,z
        msg.data.push_back(q_motor_r[1]);//13 +x
        msg.data.push_back(q_motor_r[2]);//14 -z
        msg.data.push_back(q_motor_r[3]);//15 +y
        //right hand dynamixel + fingers
        msg.data.push_back(q_motor_r[4]);//16
        msg.data.push_back(q_motor_r[5]);//17
        msg.data.push_back(q_motor_r[6]);//18
        msg.data.push_back(q_motor_r[7]);//19
        //left hand epose
        msg.data.push_back(q_motor_l[0]);//20 +y
        msg.data.push_back(q_motor_l[1]);//21 +x
        msg.data.push_back(q_motor_l[2]);//22 -z
        msg.data.push_back(q_motor_l[3]);//23 -y
        //left hand dynamixel + fingers
        msg.data.push_back(q_motor_l[4]);//24
        msg.data.push_back(q_motor_l[5]);//25
        msg.data.push_back(q_motor_l[6]);//26
        msg.data.push_back(q_motor_l[7]);//27
        ++count;
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        if(time>3){break;}


    }



    return 0;
}
