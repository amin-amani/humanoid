

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

bool simulation=!true;



ros::Publisher pub1  ;ros::Publisher pub2  ;ros::Publisher pub3  ;ros::Publisher pub4  ;
ros::Publisher pub5  ;ros::Publisher pub6  ;ros::Publisher pub7  ;ros::Publisher pub8  ;
ros::Publisher pub9  ;ros::Publisher pub10 ;ros::Publisher pub11 ;ros::Publisher pub12 ;
ros::Publisher pub13 ;ros::Publisher pub14 ;ros::Publisher pub15 ;ros::Publisher pub16 ;
ros::Publisher pub17 ;ros::Publisher pub18 ;ros::Publisher pub19 ;ros::Publisher pub20 ;
ros::Publisher pub21 ;ros::Publisher pub22 ;ros::Publisher pub23 ;ros::Publisher pub24 ;
ros::Publisher pub25 ;ros::Publisher pub26 ;ros::Publisher pub27 ;ros::Publisher pub28 ;
ros::Publisher pub29 ;ros::Publisher pub30 ;ros::Publisher pub31 ;


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


VectorXd right2left(VectorXd qr){
    VectorXd ql(7);
    ql(0)=qr(0);
    ql(1)=-qr(1);
    ql(2)=-qr(2);
    ql(3)=qr(3);
    ql(4)=-qr(4);
    ql(5)=-qr(5);
    ql(6)=qr(6);
}


int main(int argc, char **argv)
{

QByteArray joint_data;


    abs_initial_bool=!simulation;
    qc_initial_bool=!simulation;

    if (simulation){    ros::init(argc, argv, "rrbot");}
    else{ros::init(argc, argv, "jointdata");}

    ros::NodeHandle nh("~");


    std::string scenario_r;std::string scenario_l;
    //nh.getParam("scenario_r", scenario_r);

    ROS_INFO("scenario_r: %s",scenario_r.c_str());

    ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("qc",1000);
    ros::Subscriber abs_sensor = nh.subscribe("/surena/abs_joint_state", 1000, abs_read);
    ros::Subscriber qcinit = nh.subscribe("/surena/inc_joint_state", 1000, qc_initial);

    std_msgs::Int32MultiArray msg;
    std_msgs::MultiArrayDimension msg_dim;

    msg_dim.label = "joint_position";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);
    double t_r_offset=0;
    if(simulation){
        pub1  = nh.advertise<std_msgs::Float64>("joint1_position_controller/command",100);
        pub2  = nh.advertise<std_msgs::Float64>("joint2_position_controller/command",100);
        pub3  = nh.advertise<std_msgs::Float64>("joint3_position_controller/command",100);
        pub4  = nh.advertise<std_msgs::Float64>("joint4_position_controller/command",100);
        pub5  = nh.advertise<std_msgs::Float64>("joint5_position_controller/command",100);
        pub6  = nh.advertise<std_msgs::Float64>("joint6_position_controller/command",100);
        pub7  = nh.advertise<std_msgs::Float64>("joint7_position_controller/command",100);
        pub8  = nh.advertise<std_msgs::Float64>("joint8_position_controller/command",100);
        pub9  = nh.advertise<std_msgs::Float64>("joint9_position_controller/command",100);
        pub10 = nh.advertise<std_msgs::Float64>("joint10_position_controller/command",100);
        pub11 = nh.advertise<std_msgs::Float64>("joint11_position_controller/command",100);
        pub12 = nh.advertise<std_msgs::Float64>("joint12_position_controller/command",100);
        pub13 = nh.advertise<std_msgs::Float64>("joint13_position_controller/command",100);
        pub14 = nh.advertise<std_msgs::Float64>("joint14_position_controller/command",100);
        pub15 = nh.advertise<std_msgs::Float64>("joint15_position_controller/command",100);
        pub16 = nh.advertise<std_msgs::Float64>("joint16_position_controller/command",100);
        pub17 = nh.advertise<std_msgs::Float64>("joint17_position_controller/command",100);
        pub18 = nh.advertise<std_msgs::Float64>("joint18_position_controller/command",100);
        pub19 = nh.advertise<std_msgs::Float64>("joint19_position_controller/command",100);
        pub20 = nh.advertise<std_msgs::Float64>("joint20_position_controller/command",100);
        pub21 = nh.advertise<std_msgs::Float64>("joint21_position_controller/command",100);
        pub22 = nh.advertise<std_msgs::Float64>("joint22_position_controller/command",100);
        pub23 = nh.advertise<std_msgs::Float64>("joint23_position_controller/command",100);
        pub24 = nh.advertise<std_msgs::Float64>("joint24_position_controller/command",100);
        pub25 = nh.advertise<std_msgs::Float64>("joint25_position_controller/command",100);
        pub26 = nh.advertise<std_msgs::Float64>("joint26_position_controller/command",100);
        pub27 = nh.advertise<std_msgs::Float64>("joint27_position_controller/command",100);
        pub28 = nh.advertise<std_msgs::Float64>("joint28_position_controller/command",100);
        pub29 = nh.advertise<std_msgs::Float64>("joint29_position_controller/command",100);
        pub30 = nh.advertise<std_msgs::Float64>("joint30_position_controller/command",100);
        pub31 = nh.advertise<std_msgs::Float64>("joint31_position_controller/command",100);
    }

    right_hand hand_funcs;
    right_hand hand_r;

    VectorXd r_target_r(3);
    VectorXd r_force_r(3);
    VectorXd r_deliver_r(3);
    VectorXd r_middle_r(3);
    VectorXd r_release_r(3);
    MatrixXd R_target_r(3,3);
    VectorXd r_right_palm(3);
    int fingers_mode_r;
    VectorXd q0_r(7);

    double d0_r;
    double d_r ;
    double d_des_r;
    double theta_r;
    double theta_target_r;
    double sai_r;
    double sai_target_r;
    double phi_r;
    double phi_target_r;

    QVector<double> qr1;
    QVector<double> qr2;
    QVector<double> qr3;
    QVector<double> qr4;
    QVector<double> qr5;
    QVector<double> qr6;
    QVector<double> qr7;

    MinimumJerkInterpolation coef_generator;
    MatrixXd X_coef_r;
    MatrixXd Y_coef_r;
    MatrixXd Z_coef_r;

    MatrixXd   t_r(1,7);
    MatrixXd P_x_r(1,7);
    MatrixXd V_x_r(1,7);
    MatrixXd A_x_r(1,7);
    MatrixXd P_y_r(1,7);
    MatrixXd V_y_r(1,7);
    MatrixXd A_y_r(1,7);
    MatrixXd P_z_r(1,7);
    MatrixXd V_z_r(1,7);
    MatrixXd A_z_r(1,7);


    MatrixXd  fingers_r(1,7);
    VectorXd q_ra;
VectorXd q_la;
    int q_motor_r[8];
    int q_motor_l[8];
    for (int var = 0; var < 8; ++var) {
        q_motor_r[var]=0;
        q_motor_l[var]=0;
    }

    vector<double> q(31);

    ros::Rate loop_rate(200);
    int count = 0;
    double time=0.0;
    double time_r;
    bool initializing=true;
    VectorXd qr_end(7);
    double WaistYaw=0;
    double WaistPitch=0;
    double knee=0;
    double hip=0;
    double ankle=0;
//QTime chronometer;
//chronometer.start();

    vector<double> cntrl(13);
    QCgenerator QC;
    vector<int> qref(12);

    while (ros::ok())
    {

   // chronometer.restart();

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

            if(simulation){
                q0_r<<10*M_PI/180,
                        -10*M_PI/180,
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


                qDebug("q0 init ok");

            }
            ROS_INFO("q0_r= %f, %f, %f, %f, %f, %f, %f",q0_r(0),q0_r(1),q0_r(2),q0_r(3),q0_r(4),q0_r(5),q0_r(6));


            // gripping

                r_target_r<<.45,
                        -0.05,
                        -0.35;

                r_middle_r<<.3,
                        -.2,
                        -.35;

                r_force_r<<.45,
                        -0.02,
                        -0.35;

                r_deliver_r<<.45,
                        -0.02,
                        -0.15;

                r_release_r<<.45,
                        -0.1,
                        -0.15;

                R_target_r=hand_funcs.rot(2,-90*M_PI/180,3);//*hand_funcs.rot(1,-10*M_PI/180+atan2(r_target_r(1),r_target_r(0)),3);

                //taghire R-target baraye target va deliver va taghire jabejayi
                //R_target_r=hand_funcs.rot(2,-70*M_PI/180,3);
                //R_deliver_r=hand_funcs.rot(2,-110*M_PI/180,3);



            double v0_r=0;
            double v_target_r =.4;
            right_hand hand0_r(q0_r,r_target_r,R_target_r,0,0);
            r_right_palm=hand0_r.r_right_palm;
            d0_r=hand0_r.dist;
            d_r=d0_r;
            d_des_r=hand0_r.d_des;
            theta_r=hand0_r.theta;
            theta_target_r=hand0_r.theta_target;
            sai_r=hand0_r.sai;
            sai_target_r=hand0_r.sai_target;
            phi_r=hand0_r.phi;
            phi_target_r=hand0_r.phi_target;
            hand0_r.HO_FK_right_palm(q0_r);

            q_ra=q0_r;

            qr1.append(q_ra(0));
            qr2.append(q_ra(1));
            qr3.append(q_ra(2));
            qr4.append(q_ra(3));
            qr5.append(q_ra(4));
            qr6.append(q_ra(5));
            qr7.append(q_ra(6));

            if (simulation)
            {
                vector<double> q_init(31);
                for (int i = 0; i < 31; ++i) {
                    q_init[i]=0;
                }
                q_init[15]=qr1[0];   q_init[22]=qr1[0];
                q_init[16]=qr2[0];   q_init[23]=-qr2[0];
                q_init[17]=qr3[0];   q_init[24]=-qr3[0];
                q_init[18]=qr4[0];   q_init[25]=qr4[0];
                q_init[19]=qr5[0];   q_init[26]=-qr5[0];
                q_init[20]=qr6[0];   q_init[27]=-qr6[0];
                q_init[21]=qr7[0];   q_init[28]=qr7[0];

                SendGazebo(q_init);
            }



            //****path generation




            MatrixXd T_right_shoulder_transpose=rightshoulder2waist(0*WaistYaw,0*WaistPitch);

            VectorXd temp(4);
            temp<<r_target_r,1;
            r_target_r=(T_right_shoulder_transpose*temp).block(0,0,3,1);

            temp<<r_deliver_r,1;
            r_deliver_r=(T_right_shoulder_transpose*temp).block(0,0,3,1);

            temp<<r_release_r,1;
            r_release_r=(T_right_shoulder_transpose*temp).block(0,0,3,1);
            MatrixXd ord(1,6);
            ord<<5,5,5,5,5,5;

            t_r<<0,2,4,6,10,12,14;

            P_x_r<< hand0_r.r_right_palm(0),r_middle_r(0),r_target_r(0),r_force_r(0),r_deliver_r(0),r_deliver_r(0),r_release_r(0);
            P_y_r<< hand0_r.r_right_palm(1),r_middle_r(1),r_target_r(1),r_force_r(1),r_deliver_r(1),r_deliver_r(1),r_release_r(1);
            P_z_r<< hand0_r.r_right_palm(2),r_middle_r(2),r_target_r(2),r_force_r(2),r_deliver_r(2),r_deliver_r(2),r_release_r(2);

            R_target_r=T_right_shoulder_transpose.block(0,0,3,3)*R_target_r;

            V_x_r<<0,INFINITY,0,0,0,0,0;
            V_y_r<<0,INFINITY,0,0,0,0,0;
            V_z_r<<0,INFINITY,0,0,0,0,0;
            A_x_r<<0,INFINITY,0,0,0,0,0;
            A_y_r<<0,INFINITY,0,0,0,0,0;
            A_z_r<<0,INFINITY,0,0,0,0,0;

            MatrixXd conx(3,7); conx<<P_x_r,V_x_r,A_x_r;
            MatrixXd cony(3,7); cony<<P_y_r,V_y_r,A_y_r;
            MatrixXd conz(3,7); conz<<P_z_r,V_z_r,A_z_r;

//            X_coef_r=coef_generator.Coefficient(t_r,P_x_r,V_x_r,A_x_r);
//            Y_coef_r=coef_generator.Coefficient(t_r,P_y_r,V_y_r,A_y_r);
//            Z_coef_r=coef_generator.Coefficient(t_r,P_z_r,V_z_r,A_z_r);

            X_coef_r=coef_generator.Coefficient1(t_r,ord,conx,.1).transpose();
            Y_coef_r=coef_generator.Coefficient1(t_r,ord,cony,.1).transpose();
            Z_coef_r=coef_generator.Coefficient1(t_r,ord,conz,.1).transpose();

            fingers_r<<10,10,10,10,10,10,10;

            //ROS_INFO("theta_target=%f,sai_target=%f,phi_target=%f",theta_target,sai_target,phi_target);
            //ROS_INFO("\nr_target_r=\n%f\n%f\n%f",r_target_r(0),r_target_r(1),r_target_r(2));
            // ROS_INFO("\nR_target_r=\n%f\t%f\t%f\n%f\t%f\t%f\n%f\t%f\t%f\n",R_target_r(0,0),R_target_r(0,1),R_target_r(0,2),R_target_r(1,0),R_target_r(1,1),R_target_r(1,2),R_target_r(2,0),R_target_r(2,1),R_target_r(2,2));
            ROS_INFO("press any key to start!");
            getch();
            initializing=false;


        }


        else {


            time=double(count)*.005;
            time_r=time;



            VectorXd P_r(3); VectorXd V_r(3);

            if(time_r<=t_r(t_r.cols()-1)){
            for(int i=0;i<t_r.cols()-1;i++){

                if(time_r>=t_r(i)&&time_r<t_r(i+1)){
                    if(i==4){
                        P_r<<   coef_generator.GetAccVelPos(X_coef_r.row(i),t_r(i),0,5)(0,0),
                                coef_generator.GetAccVelPos(Y_coef_r.row(i),t_r(i),0,5)(0,0),
                                coef_generator.GetAccVelPos(Z_coef_r.row(i),t_r(i),0,5)(0,0);
                        V_r.fill(0);
                    }
                    else{
                        P_r<<   coef_generator.GetAccVelPos(X_coef_r.row(i),time_r,0,5)(0,0),
                                coef_generator.GetAccVelPos(Y_coef_r.row(i),time_r,0,5)(0,0),
                                coef_generator.GetAccVelPos(Z_coef_r.row(i),time_r,0,5)(0,0);
                        V_r<<   coef_generator.GetAccVelPos(X_coef_r.row(i),time_r,0,5)(0,1),
                                coef_generator.GetAccVelPos(Y_coef_r.row(i),time_r,0,5)(0,1),
                                coef_generator.GetAccVelPos(Z_coef_r.row(i),time_r,0,5)(0,1);
                              }

//VectorXd temp_r(3);
//switch (i) {
//case 0:
//   temp_r=r_target_r ;
//    break;
//case 1:
//   temp_r=r_target_r ;
//    break;
//case 2:
//   temp_r=r_deliver_r ;
//    break;
//case 3:
//   temp_r=r_deliver_r ;
//    break;
//default:
//    break;
//}
                   hand_r.update_right_hand(q_ra,V_r,r_target_r,R_target_r);
                   r_right_palm=hand_r.r_right_palm;
                   hand_r.doQP(q_ra);
                   q_ra=hand_r.q_next;

                   d_r=hand_r.dist;
                   theta_r=hand_r.theta;
                   sai_r=hand_r.sai;
                   phi_r=hand_r.phi;
                   qr_end=q_ra;

                  fingers_mode_r=fingers_r(i);
                }

            }
}

            else if (time_r<=t_r(t_r.cols()-1)+t_r(2)){


                            ROS_INFO_ONCE("right reached!");


                            MatrixXd t_h(1,2);
                            MatrixXd p_r(7,2); MatrixXd p_l(7,2);
                            MatrixXd zero(1,2); zero<<0,0;
                            MatrixXd r_coeff;  MatrixXd l_coeff;
                          //  t_h<<t_r(2),t_r(2)*2;

                            t_h<<t_r(t_r.cols()-1),t_r(t_r.cols()-1)+t_r(2);
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
                           fingers_mode_r=9;
                        }
            }

      double dh=move2pose(.2,time_r,t_r(1),t_r(2))-move2pose(.2,time_r,t_r(3),t_r(4));
      double l=.36+.37-dh;

      knee=acos((l*l-.37*.37-.36*.36)/2/.36/.37);
      hip=-atan(sin(knee)*.36/(cos(knee)*.36+.37));
      ankle=-knee-hip;
      WaistPitch=move2pose(.2,time_r,t_r(1),t_r(2))-move2pose(.2,time_r,t_r(3),t_r(4));
      WaistYaw=move2pose(.3,time_r,t_r(4),t_r(5)-1)-move2pose(.3,time_r,t_r(t_r.cols()-1),t_r(t_r.cols()-1)+t_r(2));

          if(time_r>t_r(t_r.cols()-1)+t_r(2)) {break;}


    qr1.append(q_ra(0));
    qr2.append(q_ra(1));
    qr3.append(q_ra(2));
    qr4.append(q_ra(3));
    qr5.append(q_ra(4));
    qr6.append(q_ra(5));
    qr7.append(q_ra(6));

    for (int i = 0; i < 31; ++i) {
        q[i]=0;
    }

    //         q[15]=-M_PI/2;
    //         q[16]=-10*M_PI/180;
    //         q[17]=-10*M_PI/180;
    q[3]=hip;
    q[4]=knee;
    q[5]=ankle;
    q[9]=hip;
    q[10]=knee;
    q[11]=ankle;
    q[15]=qr1[count];  q[22]=qr1[count];
    q[16]=qr2[count];  q[23]=-qr2[count];
    q[17]=qr3[count];  q[24]=-qr3[count];
    q[18]=qr4[count];  q[25]=qr4[count];
    q[19]=qr5[count];  q[26]=-qr5[count];
    q[20]=qr6[count];  q[27]=-qr6[count];
    q[21]=qr7[count];  q[28]=qr7[count];
    q[14]=WaistPitch;
    q[13]=WaistYaw;

    if(simulation){SendGazebo(q);}

    q_motor_r[0]=-int(10*(qr1[count]-q0_r(0))*180/M_PI*120/60)+qra_offset[0];
    q_motor_r[1]=int(10*(qr2[count]-q0_r(1))*180/M_PI*120/60)+qra_offset[1];
    q_motor_r[2]=-int(7*(qr3[count]-q0_r(2))*180/M_PI*100/60)+qra_offset[2];
    q_motor_r[3]=int(7*(qr4[count]-q0_r(3))*180/M_PI*100/60)+qra_offset[3];
    q_motor_r[4]=int((qr5[count]-q0_r(4))*(2048)/M_PI);
    q_motor_r[5]=int((qr6[count]-q0_r(5))*(4000-2050)/(23*M_PI/180));
    q_motor_r[6]=int((qr7[count]-q0_r(6))*(4000-2050)/(23*M_PI/180));

    if(q_motor_r[7]!=fingers_mode_r&&q_motor_r[7]!=2){
        q_motor_r[7]=2;

    }
    else{
        q_motor_r[7]=fingers_mode_r;
    }



    q_motor_l[0]=int(10*(qr1[count]-q0_r(0))*180/M_PI*120/60)+qla_offset[0];
    q_motor_l[1]=-int(10*(qr2[count]-q0_r(1))*180/M_PI*120/60)+qla_offset[1];
    q_motor_l[2]=int(7*(qr3[count]-q0_r(2))*180/M_PI*100/60)+qla_offset[2];
    q_motor_l[3]=-int(7*(qr4[count]-q0_r(3))*180/M_PI*100/60)+qla_offset[3];

    q_motor_l[4]=-int((qr5[count]-q0_r(4))*(2048)/M_PI);
    q_motor_l[5]=int((qr6[count]-q0_r(5))*(4000-2050)/(23*M_PI/180));
    q_motor_l[6]=int((qr7[count]-q0_r(6))*(4000-2050)/(23*M_PI/180));
    q_motor_l[7]=q_motor_r[7];



    cntrl[0]=0.0;
    cntrl[1]=0;
    cntrl[2]=0;
    cntrl[3]=hip;
    cntrl[4]=knee;
    cntrl[5]=ankle;
    cntrl[6]=0;//roll
    cntrl[7]=0;
    cntrl[8]=0;
    cntrl[9]=hip;
    cntrl[10]=knee;
    cntrl[11]=ankle;
    cntrl[12]=0;

    // qDebug()<<"Pitch: hip:"<<cntrl[3]<<"\tknee:"<<cntrl[4]<<"\tankle:"<<cntrl[5];



    qref=QC.ctrldata2qc(cntrl);




    msg.data.clear();
    for(int  i = 0;i < 12;i++)
    {
        msg.data.push_back( qref[i]+qc_offset[i]);
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

//    if(q_motor_r[7]==2){

//        for (int var = 0; var < 2; ++var) {




//        msg.data.clear();
//        for(int  i = 0;i < 12;i++)
//        {
//            msg.data.push_back(qc_offset[i]);
//        }
//        //right hand epose
//        msg.data.push_back(q_motor_r[0]);//12 -y  a,z
//        msg.data.push_back(q_motor_r[1]);//13 +x
//        msg.data.push_back(q_motor_r[2]);//14 -z
//        msg.data.push_back(q_motor_r[3]);//15 +y
//        //right hand dynamixel + fingers
//        msg.data.push_back(q_motor_r[4]);//16
//        msg.data.push_back(q_motor_r[5]);//17
//        msg.data.push_back(q_motor_r[6]);//18
//        msg.data.push_back(q_motor_r[7]);//19
//        //left hand epose
//        msg.data.push_back(q_motor_l[0]);//20 +y
//        msg.data.push_back(q_motor_l[1]);//21 +x
//        msg.data.push_back(q_motor_l[2]);//22 -z
//        msg.data.push_back(q_motor_l[3]);//23 -y
//        //left hand dynamixel + fingers
//        msg.data.push_back(q_motor_l[4]);//24
//        msg.data.push_back(q_motor_l[5]);//25
//        msg.data.push_back(q_motor_l[6]);//26
//        msg.data.push_back(q_motor_l[7]);//27

//        ros::spinOnce();
//        loop_rate.sleep();
//    qDebug()<<time_r<<"\t"<<q_motor_r[7];
//        }

//    }



    ++count;
    if(!simulation){chatter_pub.publish(msg);}
matrix_view(r_right_palm);

qDebug()<<time_r<<"\t"<<q_motor_r[7];

//    joint_data.append(":["+QString::number(q_motor_r[0])+","
//            +QString::number(q_motor_r[1])+","
//            +QString::number(q_motor_r[2])+","
//            +QString::number(q_motor_r[3])+","
//            +QString::number(q_motor_r[4])+","
//            +QString::number(q_motor_r[5])+","
//            +QString::number(q_motor_r[6])+","
//            +QString::number(q_motor_r[7])+","
//            +QString::number(q_motor_l[0])+","
//            +QString::number(q_motor_l[1])+","
//            +QString::number(q_motor_l[2])+","
//            +QString::number(q_motor_l[3])+","
//            +QString::number(q_motor_l[4])+","
//            +QString::number(q_motor_l[5])+","
//            +QString::number(q_motor_l[6])+","
//            +QString::number(q_motor_l[7])+"]\n");


    ros::spinOnce();
    loop_rate.sleep();
}

//    QFile myfile("Desktop/hands_joint_data.txt");
//    myfile.remove();
//    myfile.open(QFile::ReadWrite);
//    myfile.write(joint_data);
//    myfile.close();


return 0;
}
