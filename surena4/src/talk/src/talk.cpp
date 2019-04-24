#include <qapplication.h>
#include "ExportSO.h"
#include <iostream>
#include <QtMultimedia/QSound>
#include <qaudioformat.h>
#include "ros/ros.h"
#include <QDebug>

int main(int argc,char *argv[])
{
        QCoreApplication a(argc, argv);
    ros::init(argc, argv, "talk");
    ros::NodeHandle nh;
    void *Buff;
    // in some rwason this line cause crash entire application
    QString path="/home/cast/humanoid/surena4/src/talk/src/data/";
    // QString path="data/";
    qDebug()<<path;
     RHM_INIT(path.toStdString().data());
    //      RHM_INIT("data/");
    ros::Rate loop_rate(1);
    long vSize=RHM_Speak(L"سلام من توانایی تشخیص چهره دارم ", &Buff);
    FILE *WaveFile = fopen("/home/cast/humanoid/surena4/src/talk/src/TTSOut.wav", "wb");
    fwrite((void*)Buff, vSize, 1, WaveFile);
    fclose(WaveFile);
    // a.exec();
    QSound Sns("/home/cast/humanoid/surena4/src/talk/src/TTSOut.wav");
    Sns.play();
    while (ros::ok())
      {

    }
        return 0;
}
//int main(int argc, char **argv)
//{


//    QApplication a(argc, argv);

//    void *Buff;

//        RHM_INIT("/home/cast/humanoid/surena4/src/talk/src/data/");
//        // RHM_INIT("data/");
//        //RHM_SetSpeakerVoice(1); //to become man uncomment this
//        // u can also use ',' in any sentence to apply delay in speech.
//        long vSize=RHM_Speak(L"سلام من توانایی تشخیص چهره دارم ", &Buff);

//        FILE *WaveFile = fopen("/home/cast/humanoid/surena4/src/talk/src/TTSOut.wav", "wb");
//        fwrite((void*)Buff, vSize, 1, WaveFile);
//        fclose(WaveFile);

//       // a.exec();
//        QSound Sns("/home/cast/humanoid/surena4/src/talk/src/TTSOut.wav");


//        Sns.play();
//        return a.exec();

//}
