
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
#include "right_hand.h"
#include <sensor_msgs/JointState.h>

#include<termios.h>
#include <iostream>

using namespace  std;
using namespace  Eigen;

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
ros::Publisher pub29 ;
ros::Publisher pub30 ;
ros::Publisher pub31 ;

bool simulation=!true;

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

        for (int i = 16; i < 20; ++i){
            qla_offset[i-16]=int(msg.position[i+1]);
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

VectorXd absolute_q0(7);
bool abs_initial_bool;


void abs_read(const sensor_msgs::JointState & msg){
    if (abs_initial_bool){

    VectorXd absolute_sensor(7);
    for (int i = 0; i < 7; ++i) {
        absolute_sensor(i)=msg.position[i+13];
        absolute_q0(i)=0;
    }
//right hand
    absolute_q0(0)=double(absolute_sensor(0)-2867)/8192*2*M_PI;
    absolute_q0(1)=double(absolute_sensor(1)-1362)/8192*2*M_PI;
    absolute_q0(2)=-double(absolute_sensor(2)-2888)/8192*2*M_PI;
    absolute_q0(3)=double(absolute_sensor(3)-2088)/8192*2*M_PI;
//    //left hand
//        absolute_q0(0)=double(absolute_sensor(0)-665)/8192*2*M_PI;
//        absolute_q0(1)=double(absolute_sensor(1)-1362)/8192*2*M_PI;
//        absolute_q0(2)=-double(absolute_sensor(2)+2715)/8192*2*M_PI;
//        absolute_q0(3)=-double(absolute_sensor(3)-83)/8192*2*M_PI;
matrix_view(absolute_sensor);
VectorXd absolute_q0_deg(7);
absolute_q0_deg=180/M_PI*absolute_q0;
matrix_view(absolute_q0_deg);
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



int main(int argc, char **argv)
{
abs_initial_bool=!simulation;
qc_initial_bool=!simulation;

    if (simulation){    ros::init(argc, argv, "rrbot");}
    else{ros::init(argc, argv, "jointdata");}

    ros::NodeHandle nh("~");


    std::string scenario;
    nh.getParam("scenario", scenario);

    ROS_INFO("scenario: %s",scenario.c_str());

    ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("qc",1000);
    ros::Subscriber abs_sensor = nh.subscribe("/surena/abs_joint_state", 1000, abs_read);
    ros::Subscriber qcinit = nh.subscribe("/surena/inc_joint_state", 1000, qc_initial);

    std_msgs::Int32MultiArray msg;
    std_msgs::MultiArrayDimension msg_dim;

    msg_dim.label = "joint_position";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);

    if(simulation){
        pub1  = nh.advertise<std_msgs::Float64>("joint1_position_controller/command",1000);
        pub2  = nh.advertise<std_msgs::Float64>("joint2_position_controller/command",1000);
        pub3  = nh.advertise<std_msgs::Float64>("joint3_position_controller/command",1000);
        pub4  = nh.advertise<std_msgs::Float64>("joint4_position_controller/command",1000);
        pub5  = nh.advertise<std_msgs::Float64>("joint5_position_controller/command",1000);
        pub6  = nh.advertise<std_msgs::Float64>("joint6_position_controller/command",1000);
        pub7  = nh.advertise<std_msgs::Float64>("joint7_position_controller/command",1000);
        pub8  = nh.advertise<std_msgs::Float64>("joint8_position_controller/command",1000);
        pub9  = nh.advertise<std_msgs::Float64>("joint9_position_controller/command",1000);
        pub10 = nh.advertise<std_msgs::Float64>("joint10_position_controller/command",1000);
        pub11 = nh.advertise<std_msgs::Float64>("joint11_position_controller/command",1000);
        pub12 = nh.advertise<std_msgs::Float64>("joint12_position_controller/command",1000);
        pub13 = nh.advertise<std_msgs::Float64>("joint13_position_controller/command",1000);
        pub14 = nh.advertise<std_msgs::Float64>("joint14_position_controller/command",1000);
        pub15 = nh.advertise<std_msgs::Float64>("joint15_position_controller/command",1000);
        pub16 = nh.advertise<std_msgs::Float64>("joint16_position_controller/command",1000);
        pub17 = nh.advertise<std_msgs::Float64>("joint17_position_controller/command",1000);
        pub18 = nh.advertise<std_msgs::Float64>("joint18_position_controller/command",1000);
        pub19 = nh.advertise<std_msgs::Float64>("joint19_position_controller/command",1000);
        pub20 = nh.advertise<std_msgs::Float64>("joint20_position_controller/command",1000);
        pub21 = nh.advertise<std_msgs::Float64>("joint21_position_controller/command",1000);
        pub22 = nh.advertise<std_msgs::Float64>("joint22_position_controller/command",1000);
        pub23 = nh.advertise<std_msgs::Float64>("joint23_position_controller/command",1000);
        pub24 = nh.advertise<std_msgs::Float64>("joint24_position_controller/command",1000);
        pub25 = nh.advertise<std_msgs::Float64>("joint25_position_controller/command",1000);
        pub26 = nh.advertise<std_msgs::Float64>("joint26_position_controller/command",1000);
        pub27 = nh.advertise<std_msgs::Float64>("joint27_position_controller/command",1000);
        pub28 = nh.advertise<std_msgs::Float64>("joint28_position_controller/command",1000);
        pub29 = nh.advertise<std_msgs::Float64>("joint29_position_controller/command",1000);
        pub30 = nh.advertise<std_msgs::Float64>("joint30_position_controller/command",1000);
        pub31 = nh.advertise<std_msgs::Float64>("joint31_position_controller/command",1000);
    }

    right_hand hand_funcs;

    VectorXd r_target(3);
    VectorXd r_middle(3);
    MatrixXd R_target(3,3);
    int fingers_mode;
    VectorXd q0(7);


    double d0;
    double d  ;
    double d_des;
    double theta;
    double theta_target;
    double sai;
    double sai_target;
    double phi;
    double phi_target;

    QVector<double> qr1;
    QVector<double> qr2;
    QVector<double> qr3;
    QVector<double> qr4;
    QVector<double> qr5;
    QVector<double> qr6;
    QVector<double> qr7;

    MinimumJerkInterpolation coef_generator;
    MatrixXd X_coef;
    MatrixXd Y_coef;
    MatrixXd Z_coef;

    MatrixXd t(1,3);
    MatrixXd P_x(1,3);
    MatrixXd V_x(1,3);
    MatrixXd A_x(1,3);

    MatrixXd P_y(1,3);
    MatrixXd V_y(1,3);
    MatrixXd A_y(1,3);

    MatrixXd P_z(1,3);
    MatrixXd V_z(1,3);
    MatrixXd A_z(1,3);


    VectorXd q_ra;



    int q_motor[8];
    for (int var = 0; var < 8; ++var) {
        q_motor[var]=0;
    }

    vector<double> q(31);

    ros::Rate loop_rate(200);
    int count = 0;
    double time=0.0;
    bool initializing=true;

    VectorXd qr_end(7);
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

            if(simulation){
                q0<<10*M_PI/180,
                        -4*M_PI/180,
                        0,
                        -20*M_PI/180,
                        0,
                        0,
                        0;

//                q0<<0,
//                        -10*M_PI/180,
//                        0,
//                        0,
//                        0,
//                        0,
//                        0;
            }
            else{



                q0=absolute_q0;
qDebug("q0 init ok");

//                q0<<10*M_PI/180,
//                        -4*M_PI/180,
//                        0,
//                        -20*M_PI/180,
//                        0,
//                        0,
//                        0;

//                q0<<0,
//                        -10*M_PI/180,
//                        0,
//                        0,
//                        0,
//                        0,
//                        0;
             }
ROS_INFO("q0= %f, %f, %f, %f, %f, %f, %f",q0(0),q0(1),q0(2),q0(3),q0(4),q0(5),q0(6));
            //self recognition
            if(scenario=="a"){

                r_target<<.4,
                        0.1,
                        -0.15;
                R_target=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,90*M_PI/180,3)*hand_funcs.rot(2,-40*M_PI/180,3);
                r_middle<<.3,
                        -.0,
                        -.3;
                fingers_mode=15;
            }

            //pointing
            if(scenario=="b"){

                r_target<<.05,
                        -0.56,
                        -0;
                R_target=hand_funcs.rot(1,-90*M_PI/180,3)*hand_funcs.rot(2,-10*M_PI/180,3);
                r_middle<<.1,
                        -.35,
                        -.3;
                fingers_mode=8;
            }

            // mini touch
            if(scenario=="c"){

                r_target<<.4,
                        -0.3,
                        -0.2;
                R_target=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(3,90*M_PI/180,3)*hand_funcs.rot(2,20*M_PI/180,3);
                r_middle<<.2,
                        -.25,
                        -.35;
                fingers_mode=7;
            }

            // looking at horizon
            if(scenario=="d"){

                r_target<<.33,
                        0.08,
                        0.24;
                R_target=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,90*M_PI/180,3)*hand_funcs.rot(3,90*M_PI/180,3)*hand_funcs.rot(1,-28*M_PI/180,3);
                r_middle<<.4,
                        -.2,
                        -0.05;
                fingers_mode=7;

            }

            // respct
            if(scenario=="e"){

                r_target<<.25,
                        0.15,
                        -0.3;
                R_target=hand_funcs.rot(2,-90*M_PI/180,3)*hand_funcs.rot(1,65*M_PI/180,3);
                r_middle<<.4,
                        -0.05,
                        -0.3;
                fingers_mode=7;

            }

            // hand-shake
            if(scenario=="f"){

                r_target<<.45,
                        0.0,
                        -0.25;
                R_target=hand_funcs.rot(2,-80*M_PI/180,3);
                r_middle<<.3,
                        -0.05,
                        -0.4;
                fingers_mode=5;

            }

            // waving
            if(scenario=="g"){

                r_target<<.35,
                        -0.1,
                        0.3;
                R_target=hand_funcs.rot(2,-180*M_PI/180,3)*hand_funcs.rot(3,90*M_PI/180,3);
                r_middle<<.4,
                        -0.2,
                        -0.2;
                fingers_mode=7;

            }

            // temp
            if(scenario=="z"){

                r_target<<.25,
                        -0.25,
                        -0.35;
                R_target=hand_funcs.rot(2,-80*M_PI/180,3)*hand_funcs.rot(1,-50*M_PI/180,3);
                r_middle<<.25,
                        -0.25,
                        -0.35;
            }



            double v0=0;
            double v_target =.4;
            right_hand hand0(q0,r_target,R_target,0,0);
          //  ROS_INFO("r0=%f,%f,%f",hand0.r_right_palm(0),hand0.r_right_palm(1),hand0.r_right_palm(2));

            d0=hand0.dist;
            d=d0;
            d_des=hand0.d_des;
            theta=hand0.theta;
            theta_target=hand0.theta_target;
            sai=hand0.sai;
            sai_target=hand0.sai_target;
            phi=hand0.phi;
            phi_target=hand0.phi_target;
            hand0.HO_FK_right_palm(q0);

            q_ra=q0;

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
                q_init[15]=qr1[0];
                q_init[16]=qr2[0];
                q_init[17]=qr3[0];
                q_init[18]=qr4[0];
                q_init[19]=qr5[0];
                q_init[20]=qr6[0];
                q_init[21]=qr7[0];

                SendGazebo(q_init);
            }



            //****path generation

            t<<0,2,4;
            P_x<< hand0.r_right_palm(0),r_middle(0),r_target(0);
            P_y<< hand0.r_right_palm(1),r_middle(1),r_target(1);
            P_z<< hand0.r_right_palm(2),r_middle(2),r_target(2);
            V_x<<0,INFINITY,0;
            V_y<<0,INFINITY,0;
            V_z<<0,INFINITY,0;
            A_x<<0,INFINITY,0;
            A_y<<0,INFINITY,0;
            A_z<<0,INFINITY,0;

            X_coef=coef_generator.Coefficient(t,P_x,V_x,A_x);
            Y_coef=coef_generator.Coefficient(t,P_y,V_y,A_y);
            Z_coef=coef_generator.Coefficient(t,P_z,V_z,A_z);
         //   ROS_INFO("%d,%d",X_coef.rows(),X_coef.cols());
            //ROS_INFO("\ncoeff_x=%f\t%f\t%f\t%f\t%f\t%f",X_coef(0)
            //ROS_INFO("\n
            //ROS_INFO("\n

            //home
            if(scenario=="h"){
                 qr_end=q0;
//                qr_end<<0,
//                        0,
//                        0,
//                        0,
//                        0,
//                        0,
//                        0;
                 fingers_mode=6;

            }









            //ROS_INFO("theta_target=%f,sai_target=%f,phi_target=%f",theta_target,sai_target,phi_target);
            //ROS_INFO("\nr_target=\n%f\n%f\n%f",r_target(0),r_target(1),r_target(2));
           // ROS_INFO("\nR_target=\n%f\t%f\t%f\n%f\t%f\t%f\n%f\t%f\t%f\n",R_target(0,0),R_target(0,1),R_target(0,2),R_target(1,0),R_target(1,1),R_target(1,2),R_target(2,0),R_target(2,1),R_target(2,2));
            ROS_INFO("press any key to start!");
            getch();
            initializing=false;
        }


        else {

int fingers=6;
            time=double(count)*.005;
            if(scenario=="h"){time+=t(2);}

            VectorXd P(3);
            VectorXd V(3);

            if(time<t(1)){
                P<<coef_generator.GetAccVelPos(X_coef.row(0),time,0,5)(0,0),
                        coef_generator.GetAccVelPos(Y_coef.row(0),time,0,5)(0,0),
                        coef_generator.GetAccVelPos(Z_coef.row(0),time,0,5)(0,0);


                V<<coef_generator.GetAccVelPos(X_coef.row(0),time,0,5)(0,1),
                        coef_generator.GetAccVelPos(Y_coef.row(0),time,0,5)(0,1),
                        coef_generator.GetAccVelPos(Z_coef.row(0),time,0,5)(0,1);
                right_hand hand(q_ra,V,r_target,R_target);
                hand.doQP(q_ra);
                q_ra=hand.q_next;
                d=hand.dist;
                theta=hand.theta;
                sai=hand.sai;
                phi=hand.phi;
            }
            else if (time<t(2)){
                P<<coef_generator.GetAccVelPos(X_coef.row(1),time,t(1),5)(0,0),
                        coef_generator.GetAccVelPos(Y_coef.row(1),time,t(1),5)(0,0),
                        coef_generator.GetAccVelPos(Z_coef.row(1),time,t(1),5)(0,0);


                V<<coef_generator.GetAccVelPos(X_coef.row(1),time,t(1) ,5)(0,1),
                        coef_generator.GetAccVelPos(Y_coef.row(1),time,t(1),5)(0,1),
                        coef_generator.GetAccVelPos(Z_coef.row(1),time,t(1),5)(0,1);
                right_hand hand(q_ra,V,r_target,R_target);
                hand.doQP(q_ra);
                q_ra=hand.q_next;
                d=hand.dist;
                theta=hand.theta;
                sai=hand.sai;
                phi=hand.phi;
                qr_end=q_ra;
                fingers=fingers_mode;
                ROS_INFO("%f\t%f\t%f\t%f\t%f\t%f\t%f\t",qr_end(0),qr_end(1),qr_end(2),qr_end(3),qr_end(4),qr_end(5),qr_end(6));


            }
            else if (time<t(2)*2){

                if(scenario=="g" && time<8){q_ra(2)=qr_end(2)+15*M_PI/180*sin((time-t(2))/2*(2*M_PI));}
                else if(scenario=="f" && time<8){q_ra(3)=qr_end(3)+5*M_PI/180*sin((time-t(2))/2*(2*M_PI));}
                else{



                    ROS_INFO_ONCE("reached!");
if(scenario=="h")
{
    MatrixXd t_r(1,2);
    MatrixXd p_r(7,2);
    MatrixXd z_r(1,2);
    MatrixXd r_coeff;
    t_r<<t(2),t(2)*2;
    z_r<<0,0;

    p_r<<qr_end(0),10*M_PI/180,
            qr_end(1),-6*M_PI/180,
            qr_end(2),0,
            qr_end(3),-20*M_PI/180,
            qr_end(4),0,
            qr_end(5),0,
            qr_end(6),0;

    for (int i = 0; i < 7; ++i) {
        r_coeff=coef_generator.Coefficient(t_r,p_r.row(i),z_r,z_r);
        q_ra(i)=coef_generator.GetAccVelPos(r_coeff.row(0),time,t_r(0) ,5)(0,0);

    }
    fingers=fingers_mode;
}
 else{break;}

                    //                    r_coeff=(coef_generator.Coefficient(t_r,p_r,z_r,z_r));
                    //                    q_ra(0)=coef_generator.GetAccVelPos(r_coeff.row(0),time,t_r(0) ,5)(0,0);

                    // ROS_INFO("  %f\t%f\t%f\t%f\t%f\t%f\t%f\t",q_ra(0),q_ra(1),q_ra(2),q_ra(3),q_ra(4),q_ra(5),q_ra(6));

                }


                //else{break;}
            }
            else{break;}

            qr1.append(q_ra(0));
            qr2.append(q_ra(1));
            qr3.append(q_ra(2));
            qr4.append(q_ra(3));
            qr5.append(q_ra(4));
            qr6.append(q_ra(5));
            qr7.append(q_ra(6));




            //   ROS_INFO("q1=%f\tq2=%f\tq3=%f\tq4=%f\tq5=%f\tq6=%f\tq7=%f\t",180/M_PI*q[15],180/M_PI*q[16],180/M_PI*q[17],180/M_PI*q[18],180/M_PI*q[19],180/M_PI*q[20],180/M_PI*q[21]);
            //            ROS_INFO("t=%f\nx=%f\ty=%f\tz=%f\t\nX_des=%f\tY_des=%f\tZ_des=%f",time,hand.r_right_palm(0),hand.r_right_palm(1),hand.r_right_palm(2),
            //                  P(0),P(1),P(2));
            //        ROS_INFO("psi=%f(%f)\ttheta=%f(%f)\tphi=%f(%f)\t",hand.sai,hand.sai_target,hand.theta,hand.theta_target,hand.phi,hand.phi_target);



            {
                for (int i = 0; i < 31; ++i) {
                    q[i]=0;
                }

                //         q[15]=-M_PI/2;
                //         q[16]=-10*M_PI/180;
                //         q[17]=-10*M_PI/180;

                    q[15]=qr1[count];
                    q[16]=qr2[count];
                    q[17]=qr3[count];
                    q[18]=qr4[count];

                    q[19]=qr5[count];
                    q[20]=qr6[count];
                    q[21]=qr7[count];


                //     ROS_INFO("%f,%f,%f,%f,%f,%f,%f",q[15],q[16],q[17],q[18],q[19],q[20],q[21]);
                //   ROS_INFO("T=%f upper:%f %f lower:%f %f qdot=%f q=%f %f",hand.T,(hand.toRad(10)+q_ra(1))/hand.T,hand.qdot_max,(hand.toRad(-90)+q_ra(1))/hand.T,-hand.qdot_max,hand.qdot(2),180/M_PI*q[16],180/M_PI*q_ra(1));
if(simulation){SendGazebo(q);}


            }


            q_motor[0]=-int(10*(qr1[count]-q0(0))*180/M_PI*120/60)+qra_offset[0];
            q_motor[1]=int(10*(qr2[count]-q0(1))*180/M_PI*120/60)+qra_offset[1];
            q_motor[2]=-int(7*(qr3[count]-q0(2))*180/M_PI*100/60)+qra_offset[2];
            q_motor[3]=int(7*(qr4[count]-q0(3))*180/M_PI*100/60)+qra_offset[3];

            q_motor[4]=int((qr5[count]-q0(4))*(2048)/M_PI);
            q_motor[5]=int((qr6[count]-q0(5))*(4000-2050)/(23*M_PI/180));
            q_motor[6]=int((qr7[count]-q0(6))*(4000-2050)/(23*M_PI/180));

            q_motor[7]=fingers;

            msg.data.clear();
            for(int  i = 0;i < 12;i++)
            {
                msg.data.push_back(qc_offset[i]);
            }
            //right hand epose
            msg.data.push_back(q_motor[0]);//12 -y  a,z
            msg.data.push_back(q_motor[1]);//13 +x
            msg.data.push_back(q_motor[2]);//14 -z
            msg.data.push_back(q_motor[3]);//15 +y
            //right hand dynamixel + fingers
            msg.data.push_back(q_motor[4]);//16
            msg.data.push_back(q_motor[5]);//17
            msg.data.push_back(q_motor[6]);//18
            msg.data.push_back(q_motor[7]);//19
            //left hand epose
            msg.data.push_back(0);//20 +y
            msg.data.push_back(0);//21 +x
            msg.data.push_back(0);//22 -z
            msg.data.push_back(0);//23 -y
            //left hand dynamixel + fingers
            msg.data.push_back(0);//24
            msg.data.push_back(0);//25
            msg.data.push_back(0);//26
            msg.data.push_back(6);//27
            ++count;
if(!simulation){chatter_pub.publish(msg);}

        }

        ros::spinOnce();
        loop_rate.sleep();


    }
    return 0;
}
