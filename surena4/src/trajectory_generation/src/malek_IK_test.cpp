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
Robot SURENA;
MatrixXd PoseRoot(6,1);
MatrixXd PoseRFoot(6,1);
MatrixXd PoseLFoot(6,1);
QList<LinkM> links;
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



int main(int argc, char **argv)
{

    QString address="/media/cast/MalekZadeh/dataout.txt";


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

    //*******************This part of code is for initialization of joints of the robot for walking**********************************
    int count = 0;
    double time=0;

    ros::init(argc, argv, "myNode");
    ros::NodeHandle nh;


    ros::Rate loop_rate(200);




    ROS_INFO("press any key to start!");
    getch();
    ROS_INFO("started!");
QByteArray data;
    while (ros::ok())
    {



        PoseRFoot<<positionmat[count](0,0),positionmat[count](0,1),positionmat[count](0,2),0,0,0;
        PoseLFoot<<positionmat[count](0,3),positionmat[count](0,4),positionmat[count](0,5),0,0,0;
        PoseRoot<<positionmat[count](0,6),positionmat[count](0,7),positionmat[count](0,8),0,0,0;



        SURENA.doIK("LLeg_AnkleR_J6",PoseLFoot,"Body", PoseRoot);
        SURENA.doIK("RLeg_AnkleR_J6",PoseRFoot,"Body", PoseRoot);

//qDebug()<<time;
//MatrixXd temp=PoseRFoot.transpose();
//matrix_view(temp);
//temp=PoseLFoot.transpose();
//matrix_view(temp);
//temp=PoseRoot.transpose();
//matrix_view(temp);


links = SURENA.GetLinks();
data.append(QString::number(links[3].JointAngle)+","+QString::number(links[4].JointAngle)+","+QString::number(links[5].JointAngle)+","+
        QString::number(links[9].JointAngle)+","+QString::number(links[10].JointAngle)+","+QString::number(links[11].JointAngle)+","+"\n");

        ++count;
        ros::spinOnce();
        loop_rate.sleep();
        if(count>=N){break;}
        time+=.005;

}


    QFile myfile("/media/cast/MalekZadeh/data_joints.txt");
    myfile.remove();
    myfile.open(QFile::ReadWrite);
    myfile.write(data);
    myfile.close();
    return 0;
}


