#include "ros/ros.h"
#include "std_msgs/String.h"
#include"Eigen/Dense"
#include <vector>
#include <iostream>
#include <QString>
#include <QList>
#include "Robot.h"
#include"taskspaceonline3.h"
#include <qmath.h>
#include <cstring>
#include<qdebug.h>
#include <Eigen/Geometry>
#include <cstdlib>
#include "Eigen/eiquadprog.h"
#include "Eigen/Core"
#include "Eigen/Cholesky"
#include "Eigen/LU"
#include<std_msgs/Int32MultiArray.h>
#include<std_msgs/Float32MultiArray.h>
#include<geometry_msgs/Wrench.h>
#include<math.h>
#include<sensor_msgs/Imu.h>
#include<std_msgs/Float64.h>
#include "qcgenerator.h"
#include<termios.h>
#include<gazebo_msgs/LinkStates.h>
#include<sensor_msgs/JointState.h>
#include"pidcontroller.h"


using namespace  std;
using namespace  Eigen;

bool left_first=true;//right support in first step
bool simulation=false;
double T_begin;
double Tds=.75;
double Tss=.75;
double Tc;
int N_stride=300;



double saturate(double a, double min, double max){
    if(a<min){return min;ROS_INFO("subceeding!");}
    else if(a>max){return max;ROS_INFO("exceeding!");}
    else{return a;}
}

bool FileExists(QString path) {
    QFileInfo check_file(path);
    // check if file exists and if yes: Is it really a file and no directory?
    if (check_file.exists() && check_file.isFile()) {
        return true;
    } else {
        return false;
    }
}

QList <QByteArray> GetContent(QByteArray content)
{
    QList <QByteArray> result;
    int index=0;
    while (index<content.length()) {

        index=content.indexOf(":",index);
        if (index<0)
        {
            break;
        }

        QByteArray temp=content.mid(index,content.length());
        QByteArray line=temp.split('\n')[0];
        QList <QByteArray> values=line.split(':');
        if (values.length()>1){
            result.append(values[1]);
        }

        index++;
    }
    return result;
}

MatrixXd ExtractionOfMatrix(QByteArray data)
{
    MatrixXd mat;
    QByteArray insideBrackets=data.split(']')[0];
    insideBrackets= insideBrackets.split('[')[1];
    QList <QByteArray> rows=insideBrackets.split(';');
    QList <QByteArray> columns=rows[0].split(',');

    mat.resize(rows.length(),columns.length());

    //////initial mat values
    for (int i = 0; i < rows.length(); i++) {
        QList <QByteArray> currentCols=rows[i].split(',');
        for (int j = 0; j < currentCols.length(); j++) {
            mat(i,j) = currentCols[j].toDouble();

        }
    }


    return mat;
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

ros::Publisher pub1; ros::Publisher pub2; ros::Publisher pub3; ros::Publisher pub4;
ros::Publisher pub5; ros::Publisher pub6; ros::Publisher pub7; ros::Publisher pub8;
ros::Publisher pub9; ros::Publisher pub10; ros::Publisher pub11; ros::Publisher pub12;
ros::Publisher pub13; ros::Publisher pub14; ros::Publisher pub15; ros::Publisher pub16;
ros::Publisher pub17; ros::Publisher pub18; ros::Publisher pub19; ros::Publisher pub20;
ros::Publisher pub21; ros::Publisher pub22; ros::Publisher pub23; ros::Publisher pub24;
ros::Publisher pub25; ros::Publisher pub26; ros::Publisher pub27; ros::Publisher pub28;
ros::Publisher pub29; ros::Publisher pub30; ros::Publisher pub31;

void  SendGazebo(vector<double> ctrl){

    std_msgs::Float64 data;
    data.data=ctrl[1];
    pub1.publish(data);
    data.data=ctrl[2];
    pub2.publish(data);
    data.data=ctrl[3];
    pub3.publish(data);
    data.data=ctrl[4];
    pub4.publish(data);
    data.data=ctrl[5];
    pub5.publish(data);
    data.data=ctrl[6];
    pub6.publish(data);
    data.data=ctrl[7];
    pub7.publish(data);
    data.data=ctrl[8];
    pub8.publish(data);
    data.data=ctrl[9];
    pub9.publish(data);
    data.data=ctrl[10];
    pub10.publish(data);
    data.data=ctrl[11];
    pub11.publish(data);
    data.data=ctrl[12];
    pub12.publish(data);
    data.data=0;
    pub13.publish(data);
    data.data=0;
    pub14.publish(data);
    data.data=0;
    pub15.publish(data);
    data.data=0-10*M_PI/180;
    pub16.publish(data);
    data.data=0;
    pub17.publish(data);
    data.data=0;
    pub18.publish(data);
    data.data=0;
    pub19.publish(data);
    data.data=0;
    pub20.publish(data);
    data.data=0;
    pub21.publish(data);
    data.data=0;
    pub22.publish(data);
    data.data=0+10*M_PI/180;
    pub23.publish(data);
    data.data=0;
    pub24.publish(data);
    data.data=0;
    pub25.publish(data);
    data.data=0;
    pub26.publish(data);
    data.data=0;
    pub27.publish(data);
    data.data=0;
    pub28.publish(data);



}


void  SendGazebo(QList<LinkM> links,MatrixXd RollModifieds, double PitchModifieds, double theta_r, double phi_r, double theta_l, double phi_l){
    if(links.count()<28){qDebug()<<"index err";return;}
    std_msgs::Float64 data;

    data.data=links[1].JointAngle;    pub1.publish(data);
    data.data=links[2].JointAngle+RollModifieds(0,0);    pub2.publish(data);
    data.data=links[3].JointAngle+PitchModifieds;    pub3.publish(data);
    data.data=links[4].JointAngle;    pub4.publish(data);
    // data.data=links[5].JointAngle+theta_r;    pub5.publish(data);
    data.data=saturate(links[5].JointAngle,-M_PI/5.4,M_PI/4)+theta_r;    pub5.publish(data);
    data.data=links[6].JointAngle+phi_r;    pub6.publish(data);
    data.data=links[7].JointAngle;    pub7.publish(data);
    data.data=links[8].JointAngle+RollModifieds(1,0);    pub8.publish(data);
    data.data=links[9].JointAngle+PitchModifieds;    pub9.publish(data);
    data.data=links[10].JointAngle;    pub10.publish(data);
    //data.data=links[11].JointAngle+theta_l;    pub11.publish(data);
    data.data=saturate(links[11].JointAngle,-M_PI/5.4,M_PI/4)+theta_l;    pub11.publish(data);
    data.data=links[12].JointAngle+phi_l;    pub12.publish(data);
    data.data=links[13].JointAngle;    pub13.publish(data);
    data.data=links[14].JointAngle;    pub14.publish(data);
    data.data=links[15].JointAngle;    pub15.publish(data);
    data.data=links[16].JointAngle;    pub16.publish(data);
    data.data=links[17].JointAngle;    pub17.publish(data);
    data.data=links[18].JointAngle;    pub18.publish(data);
    data.data=links[19].JointAngle;    pub19.publish(data);
    data.data=links[20].JointAngle;    pub20.publish(data);
    data.data=links[21].JointAngle;    pub21.publish(data);
    data.data=links[22].JointAngle;    pub22.publish(data);
    data.data=links[23].JointAngle;    pub23.publish(data);
    data.data=links[24].JointAngle;    pub24.publish(data);
    data.data=links[25].JointAngle;    pub25.publish(data);
    data.data=links[26].JointAngle;    pub26.publish(data);
    data.data=links[27].JointAngle;    pub27.publish(data);
    data.data=links[28].JointAngle;    pub28.publish(data);
    data.data=0;    pub29.publish(data);
    data.data=0;    pub30.publish(data);
    data.data=0;    pub31.publish(data);
}

void  SendGazebo_reverse(QList<LinkM> links,MatrixXd RollModifieds, double PitchModifieds, double theta_r, double phi_r, double theta_l, double phi_l){
    if(links.count()<28){qDebug()<<"index err";return;}
    std_msgs::Float64 data;

    data.data=-links[7].JointAngle;    pub1.publish(data);
    data.data=-links[8].JointAngle+RollModifieds(0,0);    pub2.publish(data);
    data.data=links[9].JointAngle+PitchModifieds;    pub3.publish(data);
    data.data=links[10].JointAngle;    pub4.publish(data);
    // data.data=links[5].JointAngle+theta_r;    pub5.publish(data);
    data.data=saturate(links[11].JointAngle,-M_PI/5.4,M_PI/4)+theta_r;    pub5.publish(data);
    data.data=-links[12].JointAngle+phi_r;    pub6.publish(data);
    data.data=-links[1].JointAngle;    pub7.publish(data);
    data.data=-links[2].JointAngle+RollModifieds(1,0);    pub8.publish(data);
    data.data=links[3].JointAngle+PitchModifieds;    pub9.publish(data);
    data.data=links[4].JointAngle;    pub10.publish(data);
    //data.data=links[11].JointAngle+theta_l;    pub11.publish(data);
    data.data=saturate(links[5].JointAngle,-M_PI/5.4,M_PI/4)+theta_l;    pub11.publish(data);
    data.data=-links[6].JointAngle+phi_l;    pub12.publish(data);

    data.data=links[13].JointAngle;  pub13.publish(data);
    data.data=links[14].JointAngle;  pub14.publish(data);

    data.data=links[15].JointAngle; pub22.publish(data);
    data.data=-links[16].JointAngle; pub23.publish(data);
    data.data=-links[17].JointAngle; pub24.publish(data);
    data.data=links[18].JointAngle; pub25.publish(data);
    data.data=-links[19].JointAngle; pub26.publish(data);
    data.data=-links[20].JointAngle; pub27.publish(data);
    data.data=links[21].JointAngle; pub28.publish(data);

    data.data=links[22].JointAngle; pub15.publish(data);
    data.data=-links[23].JointAngle; pub16.publish(data);
    data.data=-links[24].JointAngle; pub17.publish(data);
    data.data=links[25].JointAngle; pub18.publish(data);
    data.data=-links[26].JointAngle; pub19.publish(data);
    data.data=-links[27].JointAngle; pub20.publish(data);
    data.data=links[28].JointAngle; pub21.publish(data);

    data.data=0;    pub29.publish(data);
    data.data=0;    pub30.publish(data);
    data.data=0;    pub31.publish(data);
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

MinimumJerkInterpolation CoefGen;

//data of left foot sensor
int a,b,c,d,e,f,g,h;

//angles of ankle adaptation
double teta_motor_L,teta_motor_R;//pitch
double phi_motor_L,phi_motor_R;//roll
double PitchModified;

int qc_offset[12];
bool qc_initial_bool;

//
int bump_pushed[8];
int bump_notpushed[8];
bool bump_initialize;
void receiveFootSensor(const std_msgs::Int32MultiArray& msg)
{
    if (msg.data.size()!=8) {
        qDebug("the size of sensor data is in wrong");
        return;
    }
    if(bump_initialize){
        for (int i = 0; i < 8; ++i) {
            bump_pushed[i]=msg.data[i];
        }
        bump_initialize=false;
    }
    //ROS_INFO("I heard: [%d  %d %d %d %d  %d %d %d]", (int)msg.data[0],(int)msg.data[1],(int)msg.data[2],(int)msg.data[3],(int)msg.data[4],(int)msg.data[5],(int)msg.data[6],(int)msg.data[7]);
    int temp[8];

    bump_pushed[0]=1095;bump_pushed[1]= 849;bump_pushed[2]=3124;bump_pushed[3]=3003;
    bump_pushed[4]=3126;bump_pushed[5]=2925;bump_pushed[6]=1204;bump_pushed[7]=923;

    bump_notpushed[0]=1012;bump_notpushed[1]= 930;bump_notpushed[2]=3037;bump_notpushed[3]=3098;
    bump_notpushed[4]=3042;bump_notpushed[5]=3009;bump_notpushed[6]=1123;bump_notpushed[7]=1014;
    temp[0]=msg.data[0]-bump_notpushed[0];
    temp[1]=-1*(msg.data[1]-bump_notpushed[1]);
    temp[2]=msg.data[2]-bump_notpushed[2];
    temp[3]=-1*(msg.data[3]-bump_notpushed[3]);

    //normalizing data of sensors
    a=temp[0]*(100.0/(bump_pushed[0]-bump_notpushed[0]));
    b=temp[1]*(100.0/(bump_notpushed[1]-bump_pushed[1]));
    c=temp[2]*(100.0/(bump_pushed[2]-bump_notpushed[2]));
    d=temp[3]*(100.0/(bump_notpushed[3]-bump_pushed[3]));

    //ROS_INFO("I heard a b c d: [%d  %d %d %d]", a,b,c,d);

    temp[4]=msg.data[4]-bump_notpushed[4];
    temp[5]=-1*(msg.data[5]-bump_notpushed[5]);
    temp[6]=msg.data[6]-bump_notpushed[6];
    temp[7]=-1*(msg.data[7]-bump_notpushed[7]);

    //normalizing data of sensors
    e=temp[4]*(100.0/(bump_pushed[4]-bump_notpushed[4]));
    f=temp[5]*(100.0/(bump_notpushed[5]-bump_pushed[5]));
    g=temp[6]*(100.0/(bump_pushed[6]-bump_notpushed[6]));
    h=temp[7]*(100.0/(bump_notpushed[7]-bump_pushed[7]));

    // ROS_INFO("I heard e f g h: [%d  %d %d %d]", e,f,g,h);

    //deleting data with negative sign
    if (a<0){a=0;} if (b<0){b=0;} if (c<0){c=0;} if (d<0){d=0;}
    if (e<0){e=0;} if (f<0){f=0;} if (g<0){g=0;} if (h<0){h=0;}
}

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

double Fzl,Fzr,Mxl,Mxr;
void FT_left_feedback(const geometry_msgs::Wrench &msg){
    Fzl=msg.force.z;
    Mxl=msg.torque.y;
}

void FT_right_feedback(const geometry_msgs::Wrench &msg){
    Fzr=msg.force.z;
    Mxr=msg.torque.x;
}

MatrixXd quater2rot(double w,double x,double y, double z){
    MatrixXd R(3,3);
    R<<w*w+x*x-y*y-z*z,2*x*y-2*w*z,2*x*z+2*w*y,
            2*x*y+2*w*z,w*w-x*x+y*y-z*z,2*y*z-2*w*x,
            2*x*z-2*w*y,2*y*z+2*w*x,w*w-x*x-y*y+z*z;
    return R;
}
double roll_imu=0;
int N=100;
double roll_imu_qeue[100];
double a_y=0;double a_z=0;
void IMU_feedback(const sensor_msgs::Imu &msg){
//   double w,x,y,z;
//   w=msg.orientation.w;
//   x=msg.orientation.x;
//   y=msg.orientation.y;
//   z=msg.orientation.z;
//   MatrixXd R(3,3);
//   R=quater2rot(w,x,y,z);
  // matrix_view(R);
//   roll_imu=acos(R(2,1));
   a_y=msg.linear_acceleration.y;
   a_z=msg.linear_acceleration.z;
   roll_imu_qeue[0]=atan2(a_y,a_z)/8;
   for (int i = 1; i < N; ++i) {
       roll_imu_qeue[i]=roll_imu_qeue[i-1];
   }
   roll_imu=0;
   for (int i = 0; i < N; ++i) {
       roll_imu+=roll_imu_qeue[i]/100;
   }

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
    temp=R_left*vec_A_E;    //A=temp(2)+z_left;
    a=int(((.02-temp(2)-z_left)+abs(.02-temp(2)-z_left))*5000/2);
    temp=R_left*vec_B_F;    //B=temp(2)+z_left;
    b=int(((.02-temp(2)-z_left)+abs(.02-temp(2)-z_left))*5000/2);
    temp=R_left*vec_C_G;    //C=temp(2)+z_left;
    c=int(((.02-temp(2)-z_left)+abs(.02-temp(2)-z_left))*5000/2);
    temp=R_left*vec_D_H;    //D=temp(2)+z_left;
    d=int(((.02-temp(2)-z_left)+abs(.02-temp(2)-z_left))*5000/2);
    temp=R_right*vec_A_E;    //E=temp(2)+z_right;
    e=int(((.02-temp(2)-z_right)+abs(.02-temp(2)-z_right))*5000/2);
    temp=R_right*vec_B_F;    //F=temp(2)+z_right;
    f=int(((.02-temp(2)-z_right)+abs(.02-temp(2)-z_right))*5000/2);
    temp=R_right*vec_C_G;    //G=temp(2)+z_right;
    g=int(((.02-temp(2)-z_right)+abs(.02-temp(2)-z_right))*5000/2);
    temp=R_right*vec_D_H;    //H=temp(2)+z_right;
    h=int(((.02-temp(2)-z_right)+abs(.02-temp(2)-z_right))*5000/2);
    //ROS_INFO("z_left=%f  A=%d,B=%d,C=%d,D=%d\tz_right=%f  E=%d,F=%d,G=%d,H=%d",z_left,a,b,c,d,z_right,e,f,g,h);
    //    ROS_INFO("x_left=%f,x_right=%f,y_left=%f,y_right=%fz_left=%f,z_right=%f",x_left,x_right,y_left,y_right,z_left,z_right);
}



int main(int argc, char **argv)
{
    for (int i = 0; i < N; ++i) {
        roll_imu_qeue[i]=0;
    }


    QString address;

  //  address += "/home/milad/humanoid/surena4/src/trajectory_generation/src/OfflineWalkData/offlinepub.txt";
  //  address += "/home/milad/humanoid/surena4/src/trajectory_generation/src/OfflineWalkData/JointDataTestRadian2.txt";
     address += "/home/cast/humanoid/surena4/src/trajectory_generation/src/OfflineWalkData/JointData";
     address +="Tc1.5Td0.75";
//address += "TestRadian5.txt";

    if (!FileExists(address))
    {
        qWarning()<<"Invalid Robot data Path:"<<address;

    }
    QFile file(address);
    file.open(QFile::ReadWrite);
    QByteArray content;
    content = file.readAll();file.close();

    QList<QByteArray> Position= GetContent(content);
    int N =Position.count();
    MatrixXd positionmat[N];
    for (int i = 0; i < N; ++i) {
        positionmat[i]=ExtractionOfMatrix(Position[i]);
        //  matrix_view(positionmat[i]);

    }





    qc_initial_bool=!simulation;
    bump_initialize=false;


    vector<double> cntrl(13);
    QCgenerator QC;
    for (int i = 0; i < 12; ++i) {
        qc_offset[i]=0;
    }


    teta_motor_L=0;
    teta_motor_R=0;
    phi_motor_L=0;
    phi_motor_R=0;


    //*******************This part of code is for initialization of joints of the robot for walking**********************************
    int count = 0;
    int counter=0;
    double time=0;

    ros::init(argc, argv, "myNode");
    ros::NodeHandle nh;
    ros::Publisher  chatter_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",1000);
    ros::Publisher  contact_flag  = nh.advertise<std_msgs::Int32MultiArray>("contact_flag_timing",100);
    ros::Publisher  trajectory_data_pub  = nh.advertise<std_msgs::Float32MultiArray>("my_trajectory_data",100);
    std_msgs::Float32MultiArray trajectory_data;

    ros::Subscriber sub = nh.subscribe("/surena/bump_sensor_state", 1000, receiveFootSensor);
    ros::Subscriber ft_left = nh.subscribe("/surena/ft_l_state",1000,FT_left_feedback);
    ros::Subscriber ft_right = nh.subscribe("/surena/ft_r_state",1000,FT_right_feedback);
    ros::Subscriber imu = nh.subscribe("/surena/imu_state",1000,IMU_feedback);
    ros::Subscriber qcinit = nh.subscribe("/surena/inc_joint_state", 1000, qc_initial);
    if(simulation){//gazebo publishers
        pub1  = nh.advertise<std_msgs::Float64>("rrbot/joint1_position_controller/command",1000);
        pub2  = nh.advertise<std_msgs::Float64>("rrbot/joint2_position_controller/command",1000);
        pub3  = nh.advertise<std_msgs::Float64>("rrbot/joint3_position_controller/command",1000);
        pub4  = nh.advertise<std_msgs::Float64>("rrbot/joint4_position_controller/command",1000);
        pub5  = nh.advertise<std_msgs::Float64>("rrbot/joint5_position_controller/command",1000);
        pub6  = nh.advertise<std_msgs::Float64>("rrbot/joint6_position_controller/command",1000);
        pub7  = nh.advertise<std_msgs::Float64>("rrbot/joint7_position_controller/command",1000);
        pub8  = nh.advertise<std_msgs::Float64>("rrbot/joint8_position_controller/command",1000);
        pub9  = nh.advertise<std_msgs::Float64>("rrbot/joint9_position_controller/command",1000);
        pub10 = nh.advertise<std_msgs::Float64>("rrbot/joint10_position_controller/command",1000);
        pub11 = nh.advertise<std_msgs::Float64>("rrbot/joint11_position_controller/command",1000);
        pub12 = nh.advertise<std_msgs::Float64>("rrbot/joint12_position_controller/command",1000);
        pub13 = nh.advertise<std_msgs::Float64>("rrbot/joint13_position_controller/command",1000);
        pub14 = nh.advertise<std_msgs::Float64>("rrbot/joint14_position_controller/command",1000);
        pub15 = nh.advertise<std_msgs::Float64>("rrbot/joint15_position_controller/command",1000);
        pub16 = nh.advertise<std_msgs::Float64>("rrbot/joint16_position_controller/command",1000);
        pub17 = nh.advertise<std_msgs::Float64>("rrbot/joint17_position_controller/command",1000);
        pub18 = nh.advertise<std_msgs::Float64>("rrbot/joint18_position_controller/command",1000);
        pub19 = nh.advertise<std_msgs::Float64>("rrbot/joint19_position_controller/command",1000);
        pub20 = nh.advertise<std_msgs::Float64>("rrbot/joint20_position_controller/command",1000);
        pub21 = nh.advertise<std_msgs::Float64>("rrbot/joint21_position_controller/command",1000);
        pub22 = nh.advertise<std_msgs::Float64>("rrbot/joint22_position_controller/command",1000);
        pub23 = nh.advertise<std_msgs::Float64>("rrbot/joint23_position_controller/command",1000);
        pub24 = nh.advertise<std_msgs::Float64>("rrbot/joint24_position_controller/command",1000);
        pub25 = nh.advertise<std_msgs::Float64>("rrbot/joint25_position_controller/command",1000);
        pub26 = nh.advertise<std_msgs::Float64>("rrbot/joint26_position_controller/command",1000);
        pub27 = nh.advertise<std_msgs::Float64>("rrbot/joint27_position_controller/command",1000);
        pub28 = nh.advertise<std_msgs::Float64>("rrbot/joint28_position_controller/command",1000);
        pub29 = nh.advertise<std_msgs::Float64>("rrbot/joint29_position_controller/command",1000);
        pub30 = nh.advertise<std_msgs::Float64>("rrbot/joint30_position_controller/command",1000);
        pub31 = nh.advertise<std_msgs::Float64>("rrbot/joint31_position_controller/command",1000);
    }
    ros::Subscriber ankleStates = nh.subscribe("/gazebo/link_states", 10, ankle_states);



    ros::Rate loop_rate(200);
    std_msgs::Int32MultiArray msg;
    std_msgs::MultiArrayDimension msg_dim;

    msg_dim.label = "joint_position";
    msg_dim.size = 1;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);

    MatrixXd P;

    ROS_INFO("press any key to start!");
    getch();
    ROS_INFO("started!");
Tc=Tds+Tss;
T_begin=6+Tds/2;
    while (ros::ok())
    {

        //  for robot test musbe uncommented


        if (qc_initial_bool) {
            ROS_INFO_ONCE("qc is initializing!");
            ros::spinOnce();
            continue;
        }

        cntrl[0]=0.0;
        for (int i = 1; i <=12 ; ++i) {
           // cntrl[i]= positionmat[count](0,i-1);
            cntrl[i]=0;
        }
        //matrix_view(positionmat[count]);


       cntrl[2]+=roll_imu;
       cntrl[8]+=roll_imu;
       cntrl[6]-=roll_imu;
       cntrl[12]-=roll_imu/10;


        vector<int> qref(12);
        qref=QC.ctrldata2qc(cntrl);

        msg.data.clear();

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
        if(!simulation){chatter_pub.publish(msg);}

        if (simulation){
            SendGazebo(cntrl);
            //            if(left_first){SendGazebo(links,0*RollModified,0*PitchModified,teta_motor_R,phi_motor_R,teta_motor_L,phi_motor_L);}
            //            else{SendGazebo_reverse(links,0*RollModified,0*PitchModified,teta_motor_R,phi_motor_R,teta_motor_L,phi_motor_L);}
        }


        trajectory_data.data.clear();
        trajectory_data.data.push_back(cntrl[1]);
        trajectory_data.data.push_back(cntrl[2]);
        trajectory_data.data.push_back(cntrl[3]);
        trajectory_data.data.push_back(cntrl[4]);
        trajectory_data.data.push_back(cntrl[5]);
        trajectory_data.data.push_back(cntrl[6]);
        trajectory_data.data.push_back(cntrl[7]);
        trajectory_data.data.push_back(cntrl[8]);
        trajectory_data.data.push_back(cntrl[9]);
        trajectory_data.data.push_back(cntrl[10]);
        trajectory_data.data.push_back(cntrl[11]);
        trajectory_data.data.push_back(cntrl[12]);
        trajectory_data_pub.publish(trajectory_data);


        if(count%20==0){ //use to print once in n steps
numplot(roll_imu,-M_PI/2,M_PI/2);
          //numplot(a_y,-5,5);
        }

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
        if(counter<N_stride){
        if(count==int((2*Tc+T_begin)*200)){
            count=int(T_begin*200);
            counter++;
        }}
        if(count>=N){break;}
        time+=.005;
       // qDebug()<<time<<"\t"<<counter<<"\t"<<count;
    }
    return 0;
}


