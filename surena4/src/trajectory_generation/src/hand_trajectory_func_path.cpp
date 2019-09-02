
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
#include"trajectory_generation/handmove.h"
#include "trajectory_generation/handmoveRequest.h"
#include "trajectory_generation/handmoveResponse.h"


using namespace  std;
using namespace  Eigen;

bool simulation=true;

bool move_activate=false;

int qc_offset[12];
int qra_offset[4];
int qla_offset[4];
bool qc_initial_bool;

VectorXd absolute_q0_r(7);VectorXd absolute_q0_l(7);
bool abs_initial_bool;

ros::Publisher pub1  ;ros::Publisher pub2  ;ros::Publisher pub3  ;ros::Publisher pub4  ;
ros::Publisher pub5  ;ros::Publisher pub6  ;ros::Publisher pub7  ;ros::Publisher pub8  ;
ros::Publisher pub9  ;ros::Publisher pub10 ;ros::Publisher pub11 ;ros::Publisher pub12 ;
ros::Publisher pub13 ;ros::Publisher pub14 ;ros::Publisher pub15 ;ros::Publisher pub16 ;
ros::Publisher pub17 ;ros::Publisher pub18 ;ros::Publisher pub19 ;ros::Publisher pub20 ;
ros::Publisher pub21 ;ros::Publisher pub22 ;ros::Publisher pub23 ;ros::Publisher pub24 ;
ros::Publisher pub25 ;ros::Publisher pub26 ;ros::Publisher pub27 ;ros::Publisher pub28 ;
ros::Publisher pub29 ;ros::Publisher pub30 ;ros::Publisher pub31 ;
right_hand hand_r;
left_hand hand_l;
// waist
double Waist2ArmZ=0.2694;
double Waist2RArmY=-0.239;
double Waist2LArmY=0.239;
//double WaistYaw=0; double WaistPitch=0;
//MatrixXd R_shoulder(3,3);
//VectorXd r_left_shoulder(3);
//VectorXd r_right_shoulder(3);
//    R_shoulder<<cos(WaistYaw)*cos(WaistPitch), -sin(WaistYaw), cos(WaistYaw)*sin(WaistPitch),
//            cos(WaistPitch)*sin(WaistYaw),  cos(WaistYaw), sin(WaistYaw)*sin(WaistPitch),
//            -sin(WaistPitch),              0,               cos(WaistPitch);
//    r_left_shoulder<< Waist2ArmZ*cos(WaistYaw)*sin(WaistPitch) - Waist2LArmY*sin(WaistYaw),
//                      Waist2LArmY*cos(WaistYaw) + Waist2ArmZ*sin(WaistYaw)*sin(WaistPitch)-Waist2LArmY,
//                                                               Waist2ArmZ*cos(WaistPitch)-Waist2ArmZ;
//    r_right_shoulder<< Waist2ArmZ*cos(WaistYaw)*sin(WaistPitch) - Waist2RArmY*sin(WaistYaw),
//                       Waist2RArmY*cos(WaistYaw) + Waist2ArmZ*sin(WaistYaw)*sin(WaistPitch)-Waist2RArmY,
//                                                                Waist2ArmZ*cos(WaistPitch)-Waist2ArmZ;
 std::string scenario_r="n";
 std::string scenario_l="n";
 bool initializing=true;
bool handMove(trajectory_generation::handmoveRequest &req, trajectory_generation::handmoveResponse &res){

    qDebug()<<"req";
    if(!move_activate){
        scenario_l=req.scenario_l;
        scenario_r=req.scenario_r;

    //qDebug()<<"scenario_l="<<scenario_l.c_str()<<"scenario_r="<<scenario_r.c_str();
    initializing=true;

    move_activate=true;
    res.result=1;
    return true;
    }
        else{
        return false;
        }
}

void numplot(double num,double min,double max){
    //⬛

    QString str;
    int l=100;
    int n=int((num-min)/(max-min)*l);
    if (num<min){n=0;}
    if (num>max){n=100;}
    str+=QString::number(min);
    str+="|";
    if (n<=l/2){
        for (int i = 0; i < n; ++i) {
            str+=" ";
        }
        for (int i = 0; i < l/2-n; ++i) {
            str+="|";

        }
        str+="|";
        for (int i = 0; i < l/2; ++i) {
            str+=" ";
        }
    }
    else {
        for (int i = 0; i < l/2; ++i) {
            str+=" ";
        }
        for (int i = 0; i < n-l/2; ++i) {
            str+="|";

        }
        str+="|";
        for (int i = 0; i < l-n; ++i) {
            str+=" ";
        }

    }

    str+="|";
    str+=QString::number(max);
    str+="=>";str+=QString::number(num);
    qDebug()<<str;
    qDebug()<<"";


}

void matrix_view(MatrixXd M){

    for (int i = 0; i <M.rows() ; ++i) {
        QString str;
        for (int j = 0; j <M.cols() ; ++j) {
            str+=QString::number(M(i,j));
            str+="   ";
        }
        qDebug()<<str;
    }
    qDebug()<<"";
}

void matrix_view(VectorXd M){
    QString str;
    for (int i = 0; i <M.rows() ; ++i) {str+=QString::number(M(i));str+="   ";}
    qDebug()<<str;
    qDebug()<<"";
}

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

void  SendGazebo(vector<double> q){

    std_msgs::Float64 data;
    data.data=q[1];
    pub1.publish(data);
    data.data=q[2];
    pub2.publish(data);
    data.data=q[3];
    pub3.publish(data);
    data.data=q[4];
    pub4.publish(data);
    data.data=q[5];
    pub5.publish(data);
    data.data=q[6];
    pub6.publish(data);
    data.data=q[7];
    pub7.publish(data);
    data.data=q[8];
    pub8.publish(data);
    data.data=q[9];
    pub9.publish(data);
    data.data=q[10];
    pub10.publish(data);
    data.data=q[11];
    pub11.publish(data);
    data.data=q[12];
    pub12.publish(data);
    data.data=q[13];
    pub13.publish(data);
    data.data=q[14];
    pub14.publish(data);
    data.data=q[15];
    pub15.publish(data);
    data.data=q[16];
    pub16.publish(data);
    data.data=q[17];
    pub17.publish(data);
    data.data=q[18];
    pub18.publish(data);
    data.data=q[19];
    pub19.publish(data);
    data.data=q[20];
    pub20.publish(data);
    data.data=q[21];
    pub21.publish(data);
    data.data=q[22];
    pub22.publish(data);
    data.data=q[23];
    pub23.publish(data);
    data.data=q[24];
    pub24.publish(data);
    data.data=q[25];
    pub25.publish(data);
    data.data=q[26];
    pub26.publish(data);
    data.data=q[27];
    pub27.publish(data);
    data.data=q[28];
    pub28.publish(data);
    data.data=q[29];
    pub29.publish(data);
    data.data=q[30];
    pub30.publish(data);
    data.data=q[31];
    pub31.publish(data);

}

MatrixXd rightshoulder2waist(double WaistYaw, double WaistPitch){
    MatrixXd T(4,4);
    MatrixXd R(3,3);
    VectorXd P(3);
        R<<cos(WaistYaw)*cos(WaistPitch), -sin(WaistYaw), cos(WaistYaw)*sin(WaistPitch),
         cos(WaistPitch)*sin(WaistYaw),  cos(WaistYaw), sin(WaistYaw)*sin(WaistPitch),
                      -sin(WaistPitch),              0,               cos(WaistPitch);

               P<< Waist2ArmZ*cos(WaistYaw)*sin(WaistPitch) - Waist2RArmY*sin(WaistYaw),
                Waist2RArmY*cos(WaistYaw) + Waist2ArmZ*sin(WaistYaw)*sin(WaistPitch)-Waist2RArmY,
                Waist2ArmZ*cos(WaistPitch)-Waist2ArmZ;

    T<<R.transpose(),-R.transpose()*P,0,0,0,1;

    return T;
}

MatrixXd leftshoulder2waist(double WaistYaw, double WaistPitch){
    MatrixXd T(4,4);
    MatrixXd R(3,3);
    VectorXd P(3);
    R<<cos(WaistYaw)*cos(WaistPitch), -sin(WaistYaw), cos(WaistYaw)*sin(WaistPitch),
     cos(WaistPitch)*sin(WaistYaw),  cos(WaistYaw), sin(WaistYaw)*sin(WaistPitch),
                  -sin(WaistPitch),              0,               cos(WaistPitch);

           P<< Waist2ArmZ*cos(WaistYaw)*sin(WaistPitch) - Waist2LArmY*sin(WaistYaw),
            Waist2LArmY*cos(WaistYaw) + Waist2ArmZ*sin(WaistYaw)*sin(WaistPitch)-Waist2LArmY,
            Waist2ArmZ*cos(WaistPitch)-Waist2ArmZ;                                                                1;
T<<R.transpose(),-R.transpose()*P,0,0,0,1;
    return T;
}

int main(int argc, char **argv)
{
//    scenario_l="a";
//    scenario_r="b";
//    move_activate=true;
QByteArray joint_data;


    abs_initial_bool=!simulation;
    qc_initial_bool=!simulation;


ros::init(argc, argv, "handnode");
    ros::NodeHandle nh;



    //nh.getParam("scenario_r", scenario_r);nh.getParam("scenario_l", scenario_l);

   // ROS_INFO("scenario_r: %s",scenario_r.c_str());ROS_INFO("scenario_l: %s",scenario_l.c_str());

    ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",1000);
    ros::Subscriber abs_sensor = nh.subscribe("/surena/abs_joint_state", 1000, abs_read);
    ros::Subscriber qcinit = nh.subscribe("/surena/inc_joint_state", 1000, qc_initial);
    ros::ServiceServer HandMoveService = nh.advertiseService("HandMove", handMove);
    std_msgs::Int32MultiArray msg;
    std_msgs::MultiArrayDimension msg_dim;

    msg_dim.label = "joint_position";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);
    double t_r_offset=0;double t_l_offset=0;
    if(simulation){
        pub1  = nh.advertise<std_msgs::Float64>("rrbot/joint1_position_controller/command",100);
        pub2  = nh.advertise<std_msgs::Float64>("rrbot/joint2_position_controller/command",100);
        pub3  = nh.advertise<std_msgs::Float64>("rrbot/joint3_position_controller/command",100);
        pub4  = nh.advertise<std_msgs::Float64>("rrbot/joint4_position_controller/command",100);
        pub5  = nh.advertise<std_msgs::Float64>("rrbot/joint5_position_controller/command",100);
        pub6  = nh.advertise<std_msgs::Float64>("rrbot/joint6_position_controller/command",100);
        pub7  = nh.advertise<std_msgs::Float64>("rrbot/joint7_position_controller/command",100);
        pub8  = nh.advertise<std_msgs::Float64>("rrbot/joint8_position_controller/command",100);
        pub9  = nh.advertise<std_msgs::Float64>("rrbot/joint9_position_controller/command",100);
        pub10 = nh.advertise<std_msgs::Float64>("rrbot/joint10_position_controller/command",100);
        pub11 = nh.advertise<std_msgs::Float64>("rrbot/joint11_position_controller/command",100);
        pub12 = nh.advertise<std_msgs::Float64>("rrbot/joint12_position_controller/command",100);
        pub13 = nh.advertise<std_msgs::Float64>("rrbot/joint13_position_controller/command",100);
        pub14 = nh.advertise<std_msgs::Float64>("rrbot/joint14_position_controller/command",100);
        pub15 = nh.advertise<std_msgs::Float64>("rrbot/joint15_position_controller/command",100);
        pub16 = nh.advertise<std_msgs::Float64>("rrbot/joint16_position_controller/command",100);
        pub17 = nh.advertise<std_msgs::Float64>("rrbot/joint17_position_controller/command",100);
        pub18 = nh.advertise<std_msgs::Float64>("rrbot/joint18_position_controller/command",100);
        pub19 = nh.advertise<std_msgs::Float64>("rrbot/joint19_position_controller/command",100);
        pub20 = nh.advertise<std_msgs::Float64>("rrbot/joint20_position_controller/command",100);
        pub21 = nh.advertise<std_msgs::Float64>("rrbot/joint21_position_controller/command",100);
        pub22 = nh.advertise<std_msgs::Float64>("rrbot/joint22_position_controller/command",100);
        pub23 = nh.advertise<std_msgs::Float64>("rrbot/joint23_position_controller/command",100);
        pub24 = nh.advertise<std_msgs::Float64>("rrbot/joint24_position_controller/command",100);
        pub25 = nh.advertise<std_msgs::Float64>("rrbot/joint25_position_controller/command",100);
        pub26 = nh.advertise<std_msgs::Float64>("rrbot/joint26_position_controller/command",100);
        pub27 = nh.advertise<std_msgs::Float64>("rrbot/joint27_position_controller/command",100);
        pub28 = nh.advertise<std_msgs::Float64>("rrbot/joint28_position_controller/command",100);
        pub29 = nh.advertise<std_msgs::Float64>("rrbot/joint29_position_controller/command",100);
        pub30 = nh.advertise<std_msgs::Float64>("rrbot/joint30_position_controller/command",100);
        pub31 = nh.advertise<std_msgs::Float64>("rrbot/joint31_position_controller/command",100);
    }

    right_hand hand_funcs;

    VectorXd r_target_r(3);   VectorXd r_target_l(3);
    VectorXd r_middle_r(3);   VectorXd r_middle_l(3);
    MatrixXd R_target_r(3,3); MatrixXd R_target_l(3,3);
    VectorXd r_target_rt[10];   VectorXd r_target_lt[10];
    MatrixXd R_target_rt[10]; MatrixXd R_target_lt[10];
    for (int i = 0; i < 10; ++i) {
        r_target_rt[i].resize(3,1);
        r_target_lt[i].resize(3,1);
        R_target_rt[i].resize(3,3);
        R_target_lt[i].resize(3,3);
    }
    VectorXd r_right_palm(3);VectorXd r_left_palm(3);
    int fingers_mode_r;       int fingers_mode_l;
    VectorXd q0_r(7);         VectorXd q0_l(7);




    double d0_r;              double d0_l;
    double d_r ;              double d_l ;
    double d_des_r;           double d_des_l;
    double theta_r;           double theta_l;
    double theta_target_r;    double theta_target_l;
    double sai_r;             double sai_l;
    double sai_target_r;      double sai_target_l;
    double phi_r;             double phi_l;
    double phi_target_r;      double phi_target_l;

    QVector<double> qr1;      QVector<double> ql1;
    QVector<double> qr2;      QVector<double> ql2;
    QVector<double> qr3;      QVector<double> ql3;
    QVector<double> qr4;      QVector<double> ql4;
    QVector<double> qr5;      QVector<double> ql5;
    QVector<double> qr6;      QVector<double> ql6;
    QVector<double> qr7;      QVector<double> ql7;

    MinimumJerkInterpolation coef_generator;
    MatrixXd X_coef_r;        MatrixXd X_coef_l;
    MatrixXd Y_coef_r;        MatrixXd Y_coef_l;
    MatrixXd Z_coef_r;        MatrixXd Z_coef_l;

    MatrixXd t_r(1,3);        MatrixXd t_l(1,3);
    MatrixXd P_x_r(1,3);      MatrixXd P_x_l(1,3);
    MatrixXd V_x_r(1,3);      MatrixXd V_x_l(1,3);
    MatrixXd A_x_r(1,3);      MatrixXd A_x_l(1,3);

    MatrixXd P_y_r(1,3);      MatrixXd P_y_l(1,3);
    MatrixXd V_y_r(1,3);      MatrixXd V_y_l(1,3);
    MatrixXd A_y_r(1,3);      MatrixXd A_y_l(1,3);

    MatrixXd P_z_r(1,3);      MatrixXd P_z_l(1,3);
    MatrixXd V_z_r(1,3);      MatrixXd V_z_l(1,3);
    MatrixXd A_z_r(1,3);      MatrixXd A_z_l(1,3);

    int fingers_r=9;  int fingers_l=9;
    VectorXd q_ra;VectorXd q_la;


    int q_motor_r[8];int q_motor_l[8];
    for (int var = 0; var < 8; ++var) {
        q_motor_r[var]=0;q_motor_l[var]=0;
    }

    vector<double> q(31);

    ros::Rate loop_rate(200);
    int count = 0;
    double time=0.0;
    double time_r,time_l;

    int gest_r=0;int gest_l=0;
    int gest_count;
    VectorXd qr_end(7);VectorXd ql_end(7);
    double WaistYaw=0;
    double WaistPitch=0;
    QTime chronometer;
qDebug()<<"start!";
    while (ros::ok())
    {
if(move_activate){
    //    chronometer.start();
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
            qDebug()<<"scenario_l="<<scenario_l.c_str()<<"scenario_r="<<scenario_r.c_str();

            if(simulation){
                q0_r<<10*M_PI/180,
                        -10*M_PI/180,
                        0,
                        -20*M_PI/180,
                        0,
                        0,
                        0;

                q0_l<<10*M_PI/180,
                        10*M_PI/180,
                        0,
                        -20*M_PI/180,
                        0,
                        0,
                        0;
            }

            else{
                if (scenario_r=="h"){
                    q0_r=absolute_q0_r;
                }
                else{
                    q0_r<<10*M_PI/180,
                            -10*M_PI/180,
                            0,
                            -20*M_PI/180,
                            0,
                            0,
                            0;
                }

                if (scenario_l=="h"){
                    q0_l=absolute_q0_l;
                }
                else{
                    q0_l<<10*M_PI/180,
                            10*M_PI/180,
                            0,
                            -20*M_PI/180,
                            0,
                            0,
                            0;
                }



                qDebug("q0 init ok");

            }
            ROS_INFO("q0_r= %f, %f, %f, %f, %f, %f, %f",q0_r(0),q0_r(1),q0_r(2),q0_r(3),q0_r(4),q0_r(5),q0_r(6));
            ROS_INFO("q0_l= %f, %f, %f, %f, %f, %f, %f",q0_l(0),q0_l(1),q0_l(2),q0_l(3),q0_l(4),q0_l(5),q0_l(6));

            //self recognition
            if(scenario_r=="a"){

                r_target_r<<.4,
                        0.07,
                        -0.15;
                R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,90*M_PI/180,3)*hand_funcs.rot(2,-40*M_PI/180,3);
                r_middle_r<<.3,
                        -.1,
                        -.3;
                fingers_mode_r=10;
            }
            if(scenario_l=="a"){

                r_target_l<<.4,
                        -0.07,
                        -0.15;
                R_target_l=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,-90*M_PI/180,3)*hand_funcs.rot(2,-40*M_PI/180,3);
                r_middle_l<<.3,
                        .1,
                        -.3;
                fingers_mode_l=10;
            }
            //pointing
            if(scenario_r=="b"){

                r_target_r<<.05,
                        -0.56,
                        -0;
                R_target_r=hand_funcs.rot(1,-89*M_PI/180,3)*hand_funcs.rot(2,-10*M_PI/180,3);
                r_middle_r<<.1,
                        -.35,
                        -.3;
                fingers_mode_r=11;
            }
            if(scenario_l=="b"){

                r_target_l<<.05,
                        0.56,
                        -0;
                R_target_l=hand_funcs.rot(1,89*M_PI/180,3)*hand_funcs.rot(2,-10*M_PI/180,3);
                r_middle_l<<.1,
                        .35,
                        -.3;
                fingers_mode_l=11;
            }
            // mini touch
            if(scenario_r=="c"){

                r_target_r<<.4,
                        -0.3,
                        -0.2;
                R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,90*M_PI/180,3)*hand_funcs.rot(2,20*M_PI/180,3);
                r_middle_r<<.2,
                        -.25,
                        -.35;
                fingers_mode_r=10;
            }
            if(scenario_l=="c"){

                r_target_l<<.4,
                        0.3,
                        -0.2;
                R_target_l=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,-90*M_PI/180,3)*hand_funcs.rot(2,20*M_PI/180,3);
                r_middle_l<<.2,
                        .25,
                        -.35;
                fingers_mode_l=10;
            }
            // looking at horizon
            if(scenario_r=="d"){

                r_target_r<<.3,
                        0.09,
                        0.24;
                R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,90*M_PI/180,3)*hand_funcs.rot(3,90*M_PI/180,3)*hand_funcs.rot(1,-28*M_PI/180,3);
                r_middle_r<<.4,
                        -.2,
                       - 0.05;
                fingers_mode_r=10;

            }
            if(scenario_l=="d"){

                r_target_l<<.3,
                        -0.09,
                        0.24;
                R_target_l=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,-90*M_PI/180,3)*hand_funcs.rot(3,-90*M_PI/180,3)*hand_funcs.rot(1,28*M_PI/180,3);
                r_middle_l<<.4,
                        .2,
                        -0.05;
                fingers_mode_l=10;

            }
            // respct
            if(scenario_r=="e"){

                r_target_r<<.28,
                        0.1,
                        -0.35;
                R_target_r=hand_funcs.rot(2,-70*M_PI/180,3)*hand_funcs.rot(1,50*M_PI/180,3);
                r_middle_r<<.35,
                        -0.1,
                        -0.3;
                fingers_mode_r=10;

            }
            if(scenario_l=="e"){

                r_target_l<<.28,
                        -0.1,
                        -0.35;
                R_target_l=hand_funcs.rot(2,-70*M_PI/180,3)*hand_funcs.rot(1,-50*M_PI/180,3);
                r_middle_l<<.35,
                        0.1,
                        -0.3;
                fingers_mode_l=10;

            }

            // hand-shake
            if(scenario_r=="f"){

                r_target_r<<.4,
                        -0.05,
                        -0.4;
                R_target_r=hand_funcs.rot(2,-65*M_PI/180,3);
                r_middle_r<<.3,
                        -0.1,
                        -0.4;
                fingers_mode_r=18;


            }
            if(scenario_l=="f"){

                r_target_l<<.4,
                        0.05,
                        -0.4;
                R_target_l=hand_funcs.rot(2,-65*M_PI/180,3);
                r_middle_l<<.3,
                        0.1,
                        -0.4;
                fingers_mode_l=18;

            }
            // waving
            if(scenario_r=="g"){

                r_target_r<<.35,
                        -0.1,
                        0.3;
                R_target_r=hand_funcs.rot(2,-180*M_PI/180,3)*hand_funcs.rot(3,90*M_PI/180,3);
                r_middle_r<<.4,
                        -0.2,
                        -0.2;
                fingers_mode_r=10;

            }
            if(scenario_l=="g"){

                r_target_l<<.35,
                        0.1,
                        0.3;
                R_target_l=hand_funcs.rot(2,-180*M_PI/180,3)*hand_funcs.rot(3,-90*M_PI/180,3);
                r_middle_l<<.4,
                        0.2,
                        -0.2;
                fingers_mode_l=10;

            }
            // gripping
            if(scenario_r=="j"){
                r_target_r<<.45,
                        0.,
                        -0.25;
                r_middle_r<<.3,
                        -.0,
                        -.3;

                R_target_r=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,-10*M_PI/180+atan2(r_target_r(1),r_target_r(0)),3);
                fingers_mode_r=4;
            }
            if(scenario_l=="j"){
                r_target_l<<.45,
                        -0.,
                        -0.25;
                r_middle_l<<.3,
                        .0,
                        -.3;

                R_target_l=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,-(-10*M_PI/180+atan2(r_target_r(1),r_target_r(0))),3);
                fingers_mode_l=4;
            }



            if(scenario_r=="t"||scenario_l=="t"){

                r_target_rt[0]<<.2,
                        -0.1,
                        -0.4;
                R_target_rt[0]=hand_funcs.rot(2,-70*M_PI/180,3)*hand_funcs.rot(1,-25*M_PI/180,3)*hand_funcs.rot(3,-25*M_PI/180,3);


                r_target_rt[1]<<.35,
                        -0.1,
                        -0.35;
                R_target_rt[1]=hand_funcs.rot(2,-70*M_PI/180,3)*hand_funcs.rot(3,-70*M_PI/180,3)*hand_funcs.rot(2,-20*M_PI/180,3);

                r_target_rt[2]<<.4,
                        -0.1,
                        -0.15;
                R_target_rt[2]=hand_funcs.rot(2,-100*M_PI/180,3)*hand_funcs.rot(3,-40*M_PI/180,3)*hand_funcs.rot(2,-10*M_PI/180,3);

                r_target_rt[3]<<.2,
                        -0.1,
                        -0.4;
                R_target_rt[3]=hand_funcs.rot(2,-70*M_PI/180,3)*hand_funcs.rot(1,-25*M_PI/180,3)*hand_funcs.rot(3,-25*M_PI/180,3);

                r_target_rt[4]<<.1,
                        -0.25,
                        -0.45;
                R_target_rt[4]=hand_funcs.rot(3,-70*M_PI/180,3)*hand_funcs.rot(2,-50*M_PI/180,3);

                r_target_rt[5]<<.25,
                        -0.25,
                        -0.35;
                R_target_rt[5]=hand_funcs.rot(2,-80*M_PI/180,3)*hand_funcs.rot(1,-50*M_PI/180,3);
                r_target_rt[6]<<.25,
                        -0.1,
                        -0.45;
                R_target_rt[6]=hand_funcs.rot(2,-45*M_PI/180,3)*hand_funcs.rot(3,-30*M_PI/180,3);
                r_target_rt[7]<<.3,
                          0.1,
                        -0.35;
                R_target_rt[7]=hand_funcs.rot(2,-70*M_PI/180,3)*hand_funcs.rot(1,20*M_PI/180,3);
                r_target_rt[8]<<.25,
                        -0.2,
                        -0.45;
                R_target_rt[8]=hand_funcs.rot(2,-30*M_PI/180,3)*hand_funcs.rot(3,-15*M_PI/180,3);



                r_target_lt[0]<<.2,
                        0.1,
                        -0.4;
                R_target_lt[0]=hand_funcs.rot(2,-70*M_PI/180,3)*hand_funcs.rot(1,25*M_PI/180,3)*hand_funcs.rot(3,25*M_PI/180,3);


                r_target_lt[1]<<.35,
                        0.1,
                        -0.35;
                R_target_lt[1]=hand_funcs.rot(2,-70*M_PI/180,3)*hand_funcs.rot(3,70*M_PI/180,3)*hand_funcs.rot(2,-20*M_PI/180,3);

                r_target_lt[2]<<.4,
                        0.1,
                        -0.15;
                R_target_lt[2]=hand_funcs.rot(2,-100*M_PI/180,3)*hand_funcs.rot(3,40*M_PI/180,3)*hand_funcs.rot(2,-10*M_PI/180,3);

                r_target_lt[3]<<.2,
                        0.1,
                        -0.4;
                R_target_lt[3]=hand_funcs.rot(2,-70*M_PI/180,3)*hand_funcs.rot(1,25*M_PI/180,3)*hand_funcs.rot(3,25*M_PI/180,3);

                r_target_lt[4]<<.1,
                        0.25,
                        -0.45;
                R_target_lt[4]=hand_funcs.rot(3,70*M_PI/180,3)*hand_funcs.rot(2,-50*M_PI/180,3);

                r_target_lt[5]<<.25,
                        0.25,
                        -0.35;
                R_target_lt[5]=hand_funcs.rot(2,-80*M_PI/180,3)*hand_funcs.rot(1,50*M_PI/180,3);

                r_target_lt[6]<<.25,
                        0.1,
                        -0.45;
                R_target_lt[6]=hand_funcs.rot(2,-45*M_PI/180,3)*hand_funcs.rot(3,30*M_PI/180,3);
                r_target_lt[7]<<.3,
                         -0.1,
                        -0.35;
                R_target_lt[7]=hand_funcs.rot(2,-70*M_PI/180,3)*hand_funcs.rot(1,-20*M_PI/180,3);
                r_target_lt[8]<<.25,
                        0.2,
                        -0.45;
                R_target_lt[8]=hand_funcs.rot(2,-30*M_PI/180,3)*hand_funcs.rot(3,15*M_PI/180,3);

            }
            //synchronized movement
            if(scenario_r=="s"){
                r_target_r<<.35,
                       - 0.05,
                        -0.3;

                r_middle_r<<.2,
                        -.10,
                        -.45;
                R_target_r=hand_funcs.rot(2,-90*M_PI/180,3);
                fingers_mode_r=11;}
            if(scenario_r=="o"){
                r_target_r<<.4,
                       - 0.1,
                        -0.25;

                r_middle_r<<.2,
                        -.10,
                        -.45;
                R_target_r=hand_funcs.rot(2,-90*M_PI/180,3);
                fingers_mode_r=11;}

                if(scenario_r=="ws"){
                    r_target_r<<.4,
                           - 0.1,
                            -0.15;

                    r_middle_r<<.2,
                            -.10,
                            -.45;
                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3);

                fingers_mode_r=11;}
                if(scenario_r=="wu"){
                    r_target_r<<.4,
                           - 0.1,
                            -0.15;

                    r_middle_r<<.2,
                            -.10,
                            -.45;
                    R_target_r=hand_funcs.rot(2,-90*M_PI/180,3);

                fingers_mode_r=11;}
               if(scenario_l=="s"){
                r_target_l<<.35,
                        0.05,
                        -0.3;
                r_middle_l<<.2,
                        .1,
                        -.45;
                R_target_l=hand_funcs.rot(2,-90*M_PI/180,3);
                fingers_mode_l=11;
            }
               if(scenario_l=="o"){
                r_target_l<<.4,
                        0.1,
                        -0.25;
                r_middle_l<<.2,
                        .1,
                        -.45;
                R_target_l=hand_funcs.rot(2,-90*M_PI/180,3);
                fingers_mode_l=11;
            }

               if(scenario_l=="ws"){
                r_target_l<<.4,
                        0.1,
                        -0.15;
                r_middle_l<<.2,
                        .1,
                        -.45;
                R_target_l=hand_funcs.rot(2,-90*M_PI/180,3);
                fingers_mode_l=11;
            }

               if(scenario_l=="wu"){
                r_target_l<<.4,
                        0.1,
                        -0.15;
                r_middle_l<<.2,
                        .1,
                        -.45;
                R_target_l=hand_funcs.rot(2,-90*M_PI/180,3);
                fingers_mode_l=11;
            }



            // temp
            if(scenario_r=="z"){

                r_target_r<<.25,
                        -0.25,
                        -0.35;
                R_target_r=hand_funcs.rot(2,-80*M_PI/180,3)*hand_funcs.rot(1,-50*M_PI/180,3);
                r_middle_r<<.25,
                        -0.25,
                        -0.35;
            }



            double v0_r=0;
            double v_target_r =.4;
            right_hand hand0_r(q0_r,r_target_r,R_target_r,0,0);
            left_hand hand0_l(q0_l,r_target_l,R_target_l,0,0);
            //  ROS_INFO("r0=%f,%f,%f",hand0.r_right_palm(0),hand0.r_right_palm(1),hand0.r_right_palm(2));
            r_right_palm=hand0_r.r_right_palm;
            r_left_palm=hand0_l.r_left_palm;
            d0_r=hand0_r.dist;                        d0_l=hand0_l.dist;
            d_r=d0_r;                                 d_l=d0_l;
            d_des_r=hand0_r.d_des;                    d_des_l=hand0_l.d_des;
            theta_r=hand0_r.theta;                    theta_l=hand0_l.theta;
            theta_target_r=hand0_r.theta_target;      theta_target_l=hand0_l.theta_target;
            sai_r=hand0_r.sai;                        sai_l=hand0_l.sai;
            sai_target_r=hand0_r.sai_target;          sai_target_l=hand0_l.sai_target;
            phi_r=hand0_r.phi;                        phi_l=hand0_l.phi;
            phi_target_r=hand0_r.phi_target;          phi_target_l=hand0_l.phi_target;
            hand0_r.HO_FK_right_palm(q0_r);           hand0_l.HO_FK_left_palm(q0_l);

            q_ra=q0_r;q_la=q0_l;


            qr1.clear(); ql1.clear();
            qr2.clear(); ql2.clear();
            qr3.clear(); ql3.clear();
            qr4.clear(); ql4.clear();
            qr5.clear(); ql5.clear();
            qr6.clear(); ql6.clear();
            qr7.clear(); ql7.clear();


            qr1.append(q_ra(0));     ql1.append(q_la(0));
            qr2.append(q_ra(1));     ql2.append(q_la(1));
            qr3.append(q_ra(2));     ql3.append(q_la(2));
            qr4.append(q_ra(3));     ql4.append(q_la(3));
            qr5.append(q_ra(4));     ql5.append(q_la(4));
            qr6.append(q_ra(5));     ql6.append(q_la(5));
            qr7.append(q_ra(6));     ql7.append(q_la(6));

            if (simulation)
            {
                vector<double> q_init(31);
                for (int i = 0; i < 31; ++i) {
                    q_init[i]=0;
                }
                q_init[15]=qr1[0];   q_init[15+7]=ql1[0];
                q_init[16]=qr2[0];   q_init[16+7]=ql2[0];
                q_init[17]=qr3[0];   q_init[17+7]=ql3[0];
                q_init[18]=qr4[0];   q_init[18+7]=ql4[0];
                q_init[19]=qr5[0];   q_init[19+7]=ql5[0];
                q_init[20]=qr6[0];   q_init[20+7]=ql6[0];
                q_init[21]=qr7[0];   q_init[21+7]=ql7[0];

                SendGazebo(q_init);
            }



            //****path generation

            if(scenario_r=="t"||scenario_l=="t"){
                t_r<<0,0.5,1.5;t_l<<0,0.5,1.5;
                r_middle_r=hand0_r.r_right_palm;
                r_middle_l=hand0_l.r_left_palm;
                gest_r=0;gest_l=gest_r;//=rand()%8;

                gest_count=1;
                r_target_r=r_target_rt[gest_r];
                R_target_r=R_target_rt[gest_r];

                r_target_l=r_target_lt[gest_l];
                R_target_l=R_target_lt[gest_l];
            }
            else{
                t_r<<0,2,4;t_l<<0,2,4;
            }


            MatrixXd T_right_shoulder_transpose=rightshoulder2waist(WaistYaw,WaistPitch);
            MatrixXd T_left_shoulder_transpose=leftshoulder2waist(WaistYaw,WaistPitch);

            VectorXd temp(4);
            temp<<r_target_r,1;
            r_target_r=(T_right_shoulder_transpose*temp).block(0,0,3,1);

            temp<<r_target_l,1;
            r_target_l=(T_left_shoulder_transpose*temp).block(0,0,3,1);

            P_x_r<< hand0_r.r_right_palm(0),r_middle_r(0),r_target_r(0);
            P_y_r<< hand0_r.r_right_palm(1),r_middle_r(1),r_target_r(1);
            P_z_r<< hand0_r.r_right_palm(2),r_middle_r(2),r_target_r(2);
            P_x_l<< hand0_l.r_left_palm(0),r_middle_l(0),r_target_l(0);
            P_y_l<< hand0_l.r_left_palm(1),r_middle_l(1),r_target_l(1);
            P_z_l<< hand0_l.r_left_palm(2),r_middle_l(2),r_target_l(2);

            R_target_r=T_right_shoulder_transpose.block(0,0,3,3)*R_target_r;
            R_target_l=T_left_shoulder_transpose.block(0,0,3,3)*R_target_l;

            V_x_r<<0,INFINITY,0;     V_x_l<<0,INFINITY,0;
            V_y_r<<0,INFINITY,0;     V_y_l<<0,INFINITY,0;
            V_z_r<<0,INFINITY,0;     V_z_l<<0,INFINITY,0;
            A_x_r<<0,INFINITY,0;     A_x_l<<0,INFINITY,0;
            A_y_r<<0,INFINITY,0;     A_y_l<<0,INFINITY,0;
            A_z_r<<0,INFINITY,0;     A_z_l<<0,INFINITY,0;

            X_coef_r=coef_generator.Coefficient(t_r,P_x_r,V_x_r,A_x_r);
            Y_coef_r=coef_generator.Coefficient(t_r,P_y_r,V_y_r,A_y_r);
            Z_coef_r=coef_generator.Coefficient(t_r,P_z_r,V_z_r,A_z_r);

            X_coef_l=coef_generator.Coefficient(t_l,P_x_l,V_x_l,A_x_l);
            Y_coef_l=coef_generator.Coefficient(t_l,P_y_l,V_y_l,A_y_l);
            Z_coef_l=coef_generator.Coefficient(t_l,P_z_l,V_z_l,A_z_l);
            //   ROS_INFO("%d,%d",X_coef.rows(),X_coef.cols());
            //ROS_INFO("\ncoeff_x=%f\t%f\t%f\t%f\t%f\t%f",X_coef(0)
            //ROS_INFO("\n
            //ROS_INFO("\n

            //home



            if(scenario_r=="h"){
                qr_end=q0_r;
                fingers_mode_r=9;
            }
            if(scenario_l=="h"){
                ql_end=q0_l;
                fingers_mode_l=9;
            }
//            ROS_INFO("press any key to start!");
//            getch();


            count=0;



            initializing=false;
        }


        else {


            time=double(count)*.005;
            time_r=time; time_l=time;

            if((scenario_r=="g"||scenario_r=="f")&&time_r>=t_r(2)*2-.005){scenario_r="h";;t_r_offset=2*t_r(2);fingers_mode_r=9;}
            if((scenario_l=="g"||scenario_l=="f")&&time_l>=t_l(2)*2-.005){scenario_l="h";;t_l_offset=2*t_l(2);fingers_mode_l=9;}



            if((scenario_r=="t"||scenario_l=="t")){time_r=fmod(time,t_r(2)); time_l= time_r;

                if( time_r==0){
                    r_middle_r=r_right_palm;
                    r_middle_l=r_left_palm;

//                    int temp1=rand()%8;
//                    int temp2=rand()%8;
                   ++gest_count;

//                    while(temp1==gest_r){temp1=rand()%8;}
//                    gest_r=temp1;
//                    while(temp2==gest_l){temp2=rand()%8;}
//                    gest_l=temp2;
                    gest_r++;gest_l=gest_r;
                    if (gest_r>=8){
                        gest_r=0;
                        gest_l=gest_r;
                    }

                    qDebug()<<"gest_count:"<<gest_count;
                    if (gest_count==10){scenario_r="h"; scenario_l="h";
                        t_r_offset=gest_count*t_r(2);
                        t_l_offset=gest_count*t_l(2);
                    }
                    gest_r=gest_l;
                    r_target_r=r_target_rt[gest_r];
                    R_target_r=R_target_rt[gest_r];

                    r_target_l=r_target_lt[gest_l];
                    R_target_l=R_target_lt[gest_l];

                    P_x_r<< r_middle_r(0),r_middle_r(0),r_target_r(0);
                    P_y_r<< r_middle_r(1),r_middle_r(1),r_target_r(1);
                    P_z_r<< r_middle_r(2),r_middle_r(2),r_target_r(2);
                    P_x_l<< r_middle_l(0),r_middle_l(0),r_target_l(0);
                    P_y_l<< r_middle_l(1),r_middle_l(1),r_target_l(1);
                    P_z_l<< r_middle_l(2),r_middle_l(2),r_target_l(2);

                    V_x_r<<0,INFINITY,0;     V_x_l<<0,INFINITY,0;
                    V_y_r<<0,INFINITY,0;     V_y_l<<0,INFINITY,0;
                    V_z_r<<0,INFINITY,0;     V_z_l<<0,INFINITY,0;
                    A_x_r<<0,INFINITY,0;     A_x_l<<0,INFINITY,0;
                    A_y_r<<0,INFINITY,0;     A_y_l<<0,INFINITY,0;
                    A_z_r<<0,INFINITY,0;     A_z_l<<0,INFINITY,0;

                    X_coef_r=coef_generator.Coefficient(t_r,P_x_r,V_x_r,A_x_r);
                    Y_coef_r=coef_generator.Coefficient(t_r,P_y_r,V_y_r,A_y_r);
                    Z_coef_r=coef_generator.Coefficient(t_r,P_z_r,V_z_r,A_z_r);

                    X_coef_l=coef_generator.Coefficient(t_l,P_x_l,V_x_l,A_x_l);
                    Y_coef_l=coef_generator.Coefficient(t_l,P_y_l,V_y_l,A_y_l);
                    Z_coef_l=coef_generator.Coefficient(t_l,P_z_l,V_z_l,A_z_l);
                }
            }

            if(scenario_r=="h"){time_r=time+(t_r(2)-t_r_offset);}
            if(scenario_l=="h"){time_l=time+(t_l(2)-t_l_offset);}

            if(scenario_r=="n"){time_r=t_r(2)*3;}
            if(scenario_l=="n"){time_l=t_l(2)*3;}

            VectorXd P_r(3); VectorXd V_r(3);
            VectorXd P_l(3); VectorXd V_l(3);
            if(time_r<t_r(1)){
                P_r<<   coef_generator.GetAccVelPos(X_coef_r.row(0),time_r,0,5)(0,0),
                        coef_generator.GetAccVelPos(Y_coef_r.row(0),time_r,0,5)(0,0),
                        coef_generator.GetAccVelPos(Z_coef_r.row(0),time_r,0,5)(0,0);
                V_r<<   coef_generator.GetAccVelPos(X_coef_r.row(0),time_r,0,5)(0,1),
                        coef_generator.GetAccVelPos(Y_coef_r.row(0),time_r,0,5)(0,1),
                        coef_generator.GetAccVelPos(Z_coef_r.row(0),time_r,0,5)(0,1);




                hand_r.update_right_hand(q_ra,V_r,r_target_r,R_target_r);
                r_right_palm=hand_r.r_right_palm;
                hand_r.doQP(q_ra);
                q_ra=hand_r.q_next;
                d_r=hand_r.dist;
                theta_r=hand_r.theta;
                sai_r=hand_r.sai;
                phi_r=hand_r.phi;
            }

            else if (time_r<t_r(2)){
                P_r<<   coef_generator.GetAccVelPos(X_coef_r.row(1),time_r,t_r(1),5)(0,0),
                        coef_generator.GetAccVelPos(Y_coef_r.row(1),time_r,t_r(1),5)(0,0),
                        coef_generator.GetAccVelPos(Z_coef_r.row(1),time_r,t_r(1),5)(0,0);
                V_r<<   coef_generator.GetAccVelPos(X_coef_r.row(1),time_r,t_r(1),5)(0,1),
                        coef_generator.GetAccVelPos(Y_coef_r.row(1),time_r,t_r(1),5)(0,1),
                        coef_generator.GetAccVelPos(Z_coef_r.row(1),time_r,t_r(1),5)(0,1);


                hand_r.update_right_hand(q_ra,V_r,r_target_r,R_target_r);
                r_right_palm=hand_r.r_right_palm;

                hand_r.doQP(q_ra);
                q_ra=hand_r.q_next;
                d_r=hand_r.dist;
                theta_r=hand_r.theta;
                sai_r=hand_r.sai;
                phi_r=hand_r.phi;
                fingers_r=fingers_mode_r;
                qr_end=q_ra;
                // ROS_INFO("right: %f\t%f\t%f\t%f\t%f\t%f\t%f\t",q_ra(0),q_ra(1),q_ra(2),q_ra(3),q_ra(4),q_ra(5),q_ra(6));
                //ROS_INFO("left:  %f\t%f\t%f\t%f\t%f\t%f\t%f\t",q_la(0),q_la(1),q_la(2),q_la(3),q_la(4),q_la(5),q_la(6));

            }
            else{
                if(scenario_r=="s"){
                    double t=time_r-t_r(2);
                    double L=.1;
                    double T=4;
                    if (t<=T) {
                        double V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
//double P=10*L/pow(T,3)*pow(t,3)-15*L/pow(T,4)*pow(t,4)+6*L/pow(T,5)*pow(t,5);
//cout<<P<<endl;
                                                V_r<< 0,-V,0;
                        if (t<=T/4) {
                            T=T/4;
                            V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                            V_r<< 0,0,V;
                           }
                        else if (t<=T/2) {
                            t-=T/4;
                            T=T/4;

                            V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                            V_r<< 0,-V,0;
                           }
                        else if (t<=3*T/4) {
                            t-=T/2;
                            T=T/4;
                            V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                            V_r<< 0,0,-V;
                           }
                        else if (t<=T) {
                            t-=3*T/4;
                            T=T/4;
                            V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                            V_r<< 0,V,0;
                           }

                        hand_r.update_right_hand(q_ra,V_r,r_right_palm,R_target_r);

                        hand_r.doQP(q_ra);
                        q_ra=hand_r.q_next;
                        r_right_palm=hand_r.r_right_palm;
                        d_r=hand_r.dist;
                        theta_r=hand_r.theta;
                        sai_r=hand_r.sai;
                        phi_r=hand_r.phi;
                        fingers_r=fingers_mode_r;
                        qr_end=q_ra;}
                    else{t_r_offset=T+t_r(2);scenario_r="h";
                    }
                }

                else if(scenario_r=="o"){
                    double t=time_r-t_r(2);

                    double L=.07*2*M_PI;
                    double T=4;
                   if(t<T){
                    double V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                    double P=10*L/pow(T,3)*pow(t,3)-15*L/pow(T,4)*pow(t,4)+6*L/pow(T,5)*pow(t,5);
      V_r<<0,-V*sin(P/L*2*M_PI),V*cos(P/L*2*M_PI);


 hand_r.update_right_hand(q_ra,V_r,r_right_palm,R_target_r);
                        hand_r.doQP(q_ra);
                        q_ra=hand_r.q_next;
                        r_right_palm=hand_r.r_right_palm;
                        d_r=hand_r.dist;
                        theta_r=hand_r.theta;
                        sai_r=hand_r.sai;
                        phi_r=hand_r.phi;
                        fingers_r=fingers_mode_r;
                        qr_end=q_ra;}
                    else{t_r_offset=T+t_r(2);scenario_r="h";
                    }


}


                else if(scenario_r=="ws"){
                    double t=time_r-t_r(2);

                    double L=.1*3/2*M_PI;
                    double T=4;

                    if(t<T){
                    double V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                    double P=10*L/pow(T,3)*pow(t,3)-15*L/pow(T,4)*pow(t,4)+6*L/pow(T,5)*pow(t,5);
                    if(t<T/2){V_r<<0,-V*sin(3*P/L*M_PI),V*cos(3*P/L*M_PI);}
                    else{V_r<<0,V*cos(-3*(P-L/2)/L*M_PI),V*sin(-3*(P-L/2)/L*M_PI);}



                        hand_r.update_right_hand(q_ra,V_r,r_right_palm,R_target_r);
                        r_right_palm=hand_r.r_right_palm;
                        hand_r.doQP(q_ra);
                        q_ra=hand_r.q_next;
                        d_r=hand_r.dist;
                        theta_r=hand_r.theta;
                        sai_r=hand_r.sai;
                        phi_r=hand_r.phi;
                        fingers_r=fingers_mode_r;
                        qr_end=q_ra;


}

                    else{t_r_offset=T+t_r(2);scenario_r="h";
                    }
                }


                else if(scenario_r=="wu"){
                    double t=time_r-t_r(2);

                    double L=.1*M_PI/2+.3;
                    double T=4;

                    if(t<T){
                    double V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                    double P=10*L/pow(T,3)*pow(t,3)-15*L/pow(T,4)*pow(t,4)+6*L/pow(T,5)*pow(t,5);


                    if(P<L/3){V_r<<0,0,-V;}
                    else if(P<2*L/3){V_r<<0,V*sin(-3*M_PI/L*(P-L/3)-M_PI),V*cos(-3*M_PI/L*(P-L/3)-M_PI);}
                    else{V_r<<0,0,V;}



                        hand_r.update_right_hand(q_ra,V_r,r_right_palm,R_target_r);
                        r_right_palm=hand_r.r_right_palm;
                        hand_r.doQP(q_ra);
                        q_ra=hand_r.q_next;
                        d_r=hand_r.dist;
                        theta_r=hand_r.theta;
                        sai_r=hand_r.sai;
                        phi_r=hand_r.phi;
                        fingers_r=fingers_mode_r;
                        qr_end=q_ra;


}

                    else{t_r_offset=T+t_r(2);scenario_r="h";
                    }
                }

                else{


                    if (time_r<t_r(2)*2){



                        if(scenario_r=="g"){q_ra(2)=qr_end(2)+15*M_PI/180*sin((time_r-t_r(2))/2*(2*M_PI));}
                        else if(scenario_r=="f"){
                            if(time_r>t_r(2)+1){
                                q_ra(3)=qr_end(3)+5*M_PI/180*sin((time_r-(t_r(2)+1))/4*(2*M_PI));}
                        }
                        else{
                            ROS_INFO_ONCE("right reached!");
                            //if(scenario_r=="h")

                            MatrixXd t_h(1,2);
                            MatrixXd p_r(7,2); MatrixXd p_l(7,2);
                            MatrixXd zero(1,2); zero<<0,0;
                            MatrixXd r_coeff;  MatrixXd l_coeff;
                            t_h<<t_r(2),t_r(2)*2;


                            p_r<<   qr_end(0),10*M_PI/180,
                                    qr_end(1),-10*M_PI/180,
                                    qr_end(2),0,
                                    qr_end(3),-20*M_PI/180,
                                    qr_end(4),0,
                                    qr_end(5),0,
                                    qr_end(6),0;


                            for (int i = 0; i < 7; ++i) {
                                r_coeff=coef_generator.Coefficient(t_h,p_r.row(i),zero,zero);
                                q_ra(i)=coef_generator.GetAccVelPos(r_coeff.row(0),time_r,t_h(0) ,5)(0,0);

                            }

                            fingers_r=9;

                        }

                        if(scenario_r=="j"&&time_r==t_r(2)){ fingers_r=3;}
                        if(scenario_r=="j"&&time_r==(t_r(2)+.005)){
                            fingers_r=10;
                            for (int var = 0; var < 10*200; ++var) {
                                ros::spinOnce();
                                loop_rate.sleep();
                                //  qDebug()<<"right gripping/finger r:"<<q_motor_r[7]<<"\tl:"<<q_motor_l[7]<<"\tt_l:"<<time_l<<"\tt_r:"<<time_r;
                            }
                        }

                        if(scenario_r=="j"&&time_r==(t_r(2)+.01)){
                            fingers_r=9;
                            for (int var = 0; var < 2*200; ++var) {
                                ros::spinOnce();
                                loop_rate.sleep();
                                //  qDebug()<<"right releasing/finger r:"<<q_motor_r[7]<<"\tl:"<<q_motor_l[7]<<"\tt_l:"<<time_l<<"\tt_r:"<<time_r;
                            }
                        }


                    }
                }
            }//


            if(time_l<t_l(1)){

                P_l<<   coef_generator.GetAccVelPos(X_coef_l.row(0),time_l,0,5)(0,0),
                        coef_generator.GetAccVelPos(Y_coef_l.row(0),time_l,0,5)(0,0),
                        coef_generator.GetAccVelPos(Z_coef_l.row(0),time_l,0,5)(0,0);
                V_l<<   coef_generator.GetAccVelPos(X_coef_l.row(0),time_l,0,5)(0,1),
                        coef_generator.GetAccVelPos(Y_coef_l.row(0),time_l,0,5)(0,1),
                        coef_generator.GetAccVelPos(Z_coef_l.row(0),time_l,0,5)(0,1);


                hand_l.update_left_hand(q_la,V_l,r_target_l,R_target_l);

                r_left_palm=hand_l.r_left_palm;
                //chronometer.restart();
                hand_l.doQP(q_la);

                q_la=hand_l.q_next;
                d_l=hand_l.dist;
                theta_l=hand_l.theta;
                sai_l=hand_l.sai;
                phi_l=hand_l.phi;

            }

            else if (time_l<t_l(2)){

                P_l<<   coef_generator.GetAccVelPos(X_coef_l.row(1),time_l,t_l(1),5)(0,0),
                        coef_generator.GetAccVelPos(Y_coef_l.row(1),time_l,t_l(1),5)(0,0),
                        coef_generator.GetAccVelPos(Z_coef_l.row(1),time_l,t_l(1),5)(0,0);
                V_l<<   coef_generator.GetAccVelPos(X_coef_l.row(1),time_l,t_l(1) ,5)(0,1),
                        coef_generator.GetAccVelPos(Y_coef_l.row(1),time_l,t_l(1),5)(0,1),
                        coef_generator.GetAccVelPos(Z_coef_l.row(1),time_l,t_l(1),5)(0,1);
                hand_l.update_left_hand(q_la,V_l,r_target_l,R_target_l);
                r_left_palm=hand_l.r_left_palm;
                hand_l.doQP(q_la);
                q_la=hand_l.q_next;
                d_l=hand_l.dist;
                theta_l=hand_l.theta;
                sai_l=hand_l.sai;
                phi_l=hand_l.phi;
                fingers_l=fingers_mode_l;
                ql_end=q_la;

            }


            else{
                if(scenario_l=="s"){

                    double t=time_l-t_l(2);
                    double L=.1;
                    double T=4;
                   if(t<T){
                       double V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
//double P=10*L/pow(T,3)*pow(t,3)-15*L/pow(T,4)*pow(t,4)+6*L/pow(T,5)*pow(t,5);
//cout<<P<<endl;
                       V_l<< 0,V,0;
                       if (t<=T/4) {
                           T=T/4;
                           V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                           V_l<< 0,0,V;
                          }
                       else if (t<=T/2) {
                           t-=T/4;
                           T=T/4;

                           V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                           V_l<< 0,V,0;
                          }
                       else if (t<=3*T/4) {
                           t-=T/2;
                           T=T/4;
                           V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                           V_l<< 0,0,-V;
                          }
                       else if (t<=T) {
                           t-=3*T/4;
                           T=T/4;
                           V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                           V_l<< 0,-V,0;
                          }


                        hand_l.update_left_hand(q_la,V_l,r_left_palm,R_target_l);
                        r_left_palm=hand_l.r_left_palm;
                        hand_l.doQP(q_la);
                        q_la=hand_l.q_next;
                        d_l=hand_l.dist;
                        theta_l=hand_l.theta;
                        sai_l=hand_l.sai;
                        phi_l=hand_l.phi;
                        fingers_l=fingers_mode_l;
                        ql_end=q_la;


}

                    else{t_l_offset=T+t_l(2);scenario_l="h";
                    }
                }
                else if(scenario_l=="o"){
                    double t=time_l-t_l(2);

                    double L=.07*2*M_PI;
                    double T=4;
                   if(t<T){
                    double V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                    double P=10*L/pow(T,3)*pow(t,3)-15*L/pow(T,4)*pow(t,4)+6*L/pow(T,5)*pow(t,5);
      V_l<<0,V*sin(P/L*2*M_PI),V*cos(P/L*2*M_PI);

                        hand_l.update_left_hand(q_la,V_l,r_left_palm,R_target_l);
                        r_left_palm=hand_l.r_left_palm;
                        hand_l.doQP(q_la);
                        q_la=hand_l.q_next;
                        d_l=hand_l.dist;
                        theta_l=hand_l.theta;
                        sai_l=hand_l.sai;
                        phi_l=hand_l.phi;
                        fingers_l=fingers_mode_l;
                        ql_end=q_la;


}

                    else{t_l_offset=T+t_l(2);scenario_l="h";
                    }
                }

                else if(scenario_l=="ws"){
                    double t=time_l-t_l(2);

                    double L=.1*3/2*M_PI;
                    double T=4;

                    if(t<T){
                    double V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                    double P=10*L/pow(T,3)*pow(t,3)-15*L/pow(T,4)*pow(t,4)+6*L/pow(T,5)*pow(t,5);
                    if(t<T/2){V_l<<0,-V*sin(3*P/L*M_PI),V*cos(3*P/L*M_PI);}
                    else{V_l<<0,V*cos(-3*(P-L/2)/L*M_PI),V*sin(-3*(P-L/2)/L*M_PI);}



                        hand_l.update_left_hand(q_la,V_l,r_left_palm,R_target_l);
                        r_left_palm=hand_l.r_left_palm;
                        hand_l.doQP(q_la);
                        q_la=hand_l.q_next;
                        d_l=hand_l.dist;
                        theta_l=hand_l.theta;
                        sai_l=hand_l.sai;
                        phi_l=hand_l.phi;
                        fingers_l=fingers_mode_l;
                        ql_end=q_la;


}

                    else{t_l_offset=T+t_l(2);scenario_l="h";
                    }
                }


                else if(scenario_l=="wu"){
                    double t=time_l-t_l(2);

                    double L=.1/2*M_PI+.3;
                    double T=4;

                    if(t<T){
                    double V=30*L/pow(T,3)*pow(t,2)-60*L/pow(T,4)*pow(t,3)+30*L/pow(T,5)*pow(t,4);
                    double P=10*L/pow(T,3)*pow(t,3)-15*L/pow(T,4)*pow(t,4)+6*L/pow(T,5)*pow(t,5);
                    if(P<L/3){V_l<<0,0,-V;}
                    else if(P<2*L/3){V_l<<0,V*sin(-3*M_PI/L*(P-L/3)-M_PI),V*cos(-3*M_PI/L*(P-L/3)-M_PI);}
                    else{V_l<<0,0,V;}

                        hand_l.update_left_hand(q_la,V_l,r_left_palm,R_target_l);
                        r_left_palm=hand_l.r_left_palm;
                        hand_l.doQP(q_la);
                        q_la=hand_l.q_next;
                        d_l=hand_l.dist;
                        theta_l=hand_l.theta;
                        sai_l=hand_l.sai;
                        phi_l=hand_l.phi;
                        fingers_l=fingers_mode_l;
                        ql_end=q_la;


}

                    else{t_l_offset=T+t_l(2);scenario_l="h";
                    }
                }








                else{
                    if (time_l<t_l(2)*2){







                        if(scenario_l=="g"){q_la(2)=ql_end(2)+15*M_PI/180*sin((time_l-t_l(2))/2*(2*M_PI));}
                        else if(scenario_l=="f"){
                            if (time_l>t_l(2)+1)
                            {q_la(3)=ql_end(3)+5*M_PI/180*sin((time_l-(t_l(2)+1))/2*(2*M_PI));}
                        }
                        else{
                            ROS_INFO_ONCE("left reached!");
                            //  if(scenario_l=="h")


                            MatrixXd t_h(1,2);
                            MatrixXd p_l(7,2);
                            MatrixXd zero(1,2); zero<<0,0;
                            MatrixXd l_coeff;
                            t_h<<t_l(2),t_l(2)*2;

                            p_l<<   ql_end(0),10*M_PI/180,
                                    ql_end(1),10*M_PI/180,
                                    ql_end(2),0,
                                    ql_end(3),-20*M_PI/180,
                                    ql_end(4),0,
                                    ql_end(5),0,
                                    ql_end(6),0;

                            for (int i = 0; i < 7; ++i) {
                                l_coeff=coef_generator.Coefficient(t_h,p_l.row(i),zero,zero);
                                q_la(i)=coef_generator.GetAccVelPos(l_coeff.row(0),time_l,t_h(0) ,5)(0,0);
                            }
                            fingers_l=9;

                        }

                        if(scenario_l=="j"&&time_l==t_l(2)){ fingers_l=3;}


                        if(scenario_l=="j"&&time_l==(t_l(2)+.005)){
                            if(scenario_l=="j"){ fingers_l=10;}
                            for (int var = 0; var < 10*200; ++var) {
                                ros::spinOnce();
                                loop_rate.sleep();
                                // qDebug()<<"left gripping/finger r:"<<q_motor_r[7]<<"\tl:"<<q_motor_l[7]<<"\tt_l:"<<time_l<<"\tt_r:"<<time_r;
                            }

                        }

                        if((scenario_l=="j"&&time_l==(t_l(2)+.01))){
                            if(scenario_l=="j"){ fingers_l=9;}

                            for (int var = 0; var < 2*200; ++var) {
                                ros::spinOnce();
                                loop_rate.sleep();
                                // qDebug()<<"left releasing/finger r:"<<q_motor_r[7]<<"\tl:"<<q_motor_l[7]<<"\tt_l:"<<time_l<<"\tt_r:"<<time_r;
                            }

                        }
                    }
                }

            }



        if(scenario_l!="g"&&scenario_l!="f"&&scenario_r!="g"&&scenario_r!="f"){
            if(time_r==(t_r(2)-.005)||time_l==(t_l(2)-.005)){
                for (int var = 0; var < 2*200; ++var) {
                    ros::spinOnce();
                    loop_rate.sleep();
                }
            }
        }
          if(time_r>2*t_r(2)&&time_l>2*t_l(2)) {
              move_activate=false;
              qDebug()<<"done!";
          }


        //    matrix_view(P_r); matrix_view(P_l);
    }
    qr1.append(q_ra(0));    ql1.append(q_la(0));
    qr2.append(q_ra(1));    ql2.append(q_la(1));
    qr3.append(q_ra(2));    ql3.append(q_la(2));
    qr4.append(q_ra(3));    ql4.append(q_la(3));
    qr5.append(q_ra(4));    ql5.append(q_la(4));
    qr6.append(q_ra(5));    ql6.append(q_la(5));
    qr7.append(q_ra(6));    ql7.append(q_la(6));




    //   ROS_INFO("q1=%f\tq2=%f\tq3=%f\tq4=%f\tq5=%f\tq6=%f\tq7=%f\t",180/M_PI*q[15],180/M_PI*q[16],180/M_PI*q[17],180/M_PI*q[18],180/M_PI*q[19],180/M_PI*q[20],180/M_PI*q[21]);
    //            ROS_INFO("t=%f\nx=%f\ty=%f\tz=%f\t\nX_des=%f\tY_des=%f\tZ_des=%f",time,hand.r_right_palm(0),hand.r_right_palm(1),hand.r_right_palm(2),
    //                  P(0),P(1),P(2));
    //        ROS_INFO("psi=%f(%f)\ttheta=%f(%f)\tphi=%f(%f)\t",hand.sai,hand.sai_target,hand.theta,hand.theta_target,hand.phi,hand.phi_target);




    for (int i = 0; i < 31; ++i) {
        q[i]=0;
    }

    //         q[15]=-M_PI/2;
    //         q[16]=-10*M_PI/180;
    //         q[17]=-10*M_PI/180;

    q[15]=qr1[count];   q[15+7]=ql1[count];
    q[16]=qr2[count];   q[16+7]=ql2[count];
    q[17]=qr3[count];   q[17+7]=ql3[count];
    q[18]=qr4[count];   q[18+7]=ql4[count];
    q[19]=qr5[count];   q[19+7]=ql5[count];
    q[20]=qr6[count];   q[20+7]=ql6[count];
    q[21]=qr7[count];   q[21+7]=ql7[count];
q[14]=move2pose(WaistPitch,time,0,t_r(2))-move2pose(WaistPitch,time,t_r(2),2*t_r(2));
q[13]=move2pose(WaistYaw,time,0,t_r(2))-move2pose(WaistYaw,time,t_r(2),2*t_r(2));

    //     ROS_INFO("%f,%f,%f,%f,%f,%f,%f",q[15],q[16],q[17],q[18],q[19],q[20],q[21]);
    //   ROS_INFO("T=%f upper:%f %f lower:%f %f qdot=%f q=%f %f",hand.T,(hand.toRad(10)+q_ra(1))/hand.T,hand.qdot_max,(hand.toRad(-90)+q_ra(1))/hand.T,-hand.qdot_max,hand.qdot(2),180/M_PI*q[16],180/M_PI*q_ra(1));
    if(simulation){SendGazebo(q);}





    q_motor_r[0]=-int(10*(qr1[count]-q0_r(0))*180/M_PI*120/60)+qra_offset[0];
    q_motor_r[1]=int(10*(qr2[count]-q0_r(1))*180/M_PI*120/60)+qra_offset[1];
    q_motor_r[2]=-int(7*(qr3[count]-q0_r(2))*180/M_PI*100/60)+qra_offset[2];
    q_motor_r[3]=int(7*(qr4[count]-q0_r(3))*180/M_PI*100/60)+qra_offset[3];

    q_motor_r[4]=int((qr5[count]-q0_r(4))*(2048)/M_PI);
    q_motor_r[5]=int((qr6[count]-q0_r(5))*(4000-2050)/(23*M_PI/180));
    q_motor_r[6]=int((qr7[count]-q0_r(6))*(4000-2050)/(23*M_PI/180));

    if(q_motor_r[7]!=fingers_r&&q_motor_r[7]!=2){
        q_motor_r[7]=2;
    }
    else{
        q_motor_r[7]=fingers_r;
    }

    q_motor_l[0]=int(10*(ql1[count]-q0_l(0))*180/M_PI*120/60)+qla_offset[0];
    q_motor_l[1]=int(10*(ql2[count]-q0_l(1))*180/M_PI*120/60)+qla_offset[1];
    q_motor_l[2]=-int(7*(ql3[count]-q0_l(2))*180/M_PI*100/60)+qla_offset[2];
    q_motor_l[3]=-int(7*(ql4[count]-q0_l(3))*180/M_PI*100/60)+qla_offset[3];

    q_motor_l[4]=int((ql5[count]-q0_l(4))*(2048)/M_PI);
    q_motor_l[5]=-int((ql6[count]-q0_l(5))*(4000-2050)/(23*M_PI/180));
    q_motor_l[6]=int((ql7[count]-q0_l(6))*(4000-2050)/(23*M_PI/180));


    if(q_motor_l[7]!=fingers_l&&q_motor_l[7]!=2){
        q_motor_l[7]=2;
    }
    else{
        q_motor_l[7]=fingers_l;
    }



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
    if(!simulation){chatter_pub.publish(msg);}


    joint_data.append(":["+QString::number(q_motor_r[0])+","
            +QString::number(q_motor_r[1])+","
            +QString::number(q_motor_r[2])+","
            +QString::number(q_motor_r[3])+","
            +QString::number(q_motor_r[4])+","
            +QString::number(q_motor_r[5])+","
            +QString::number(q_motor_r[6])+","
            +QString::number(q_motor_r[7])+","
            +QString::number(q_motor_l[0])+","
            +QString::number(q_motor_l[1])+","
            +QString::number(q_motor_l[2])+","
            +QString::number(q_motor_l[3])+","
            +QString::number(q_motor_l[4])+","
            +QString::number(q_motor_l[5])+","
            +QString::number(q_motor_l[6])+","
            +QString::number(q_motor_l[7])+"]\n");
//qDebug()<<chronometer.elapsed();
//    qDebug()<<"right end effector position:";
//    matrix_view(r_right_palm);


//    VectorXd tempo=q_ra*180/M_PI;
//     matrix_view(tempo);


//    qDebug()<<"left end effector position:";
//    matrix_view(r_left_palm);





    // qDebug()<<"finger r:"<<q_motor_r[7]<<"\tl:"<<q_motor_l[7];
}
ros::spinOnce();
loop_rate.sleep();
}

    QFile myfile("Desktop/hands_joint_data.txt");
    myfile.remove();
    myfile.open(QFile::ReadWrite);
    myfile.write(joint_data);
    myfile.close();


return 0;
}
