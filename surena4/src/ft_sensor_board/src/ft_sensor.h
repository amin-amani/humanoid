#ifndef FT_SENSOR_H
#define FT_SENSOR_H

#include <QObject>
#include <QDebug>
#include "usbhid.h"
#include <QtCore>
#include <QtGui>
#include <QtDebug>
#include <QTimer>
//#include <QtTest/QSignalSpy>
#if defined(Q_OS_ANDROID)
#include <dbt.h>
#endif
#include "libusbwin/libusb.h"



class ft_sensor : public QObject
{
    Q_OBJECT

   const double m_dDecouplingCoefficient[6][6]={
{1,0,0,0,0,0},
{0,1,0,0,0,0},
{0,0,1,0,0,0},
{0,0,0,1,0,0},
{0,0,0,0,1,0},
{0,0,0,0,0,1},

//{1033.78406318488,0,0,0,0,0},
//{0,1037.08620260516,0,0,0,0},
//{0,0,7022.47191011235,0,0,0},
//{0,0,0,92.2339051835454,0,0},
//{0,0,0,0,92.6698174404596,0},
//{0,0,0,0,0,60.819851599562},
};
   const double sensitivityFT[6][1]=  // #573 Right foot FTsensor
   {
       {9.6732*0.0001},
       {9.6424*0.0001},
       {1.4240*0.0001},
       {1.0842*0.01},
       {1.0791*0.01},
       {1.6442*0.01},
   };
const    double sensitivityFT1[6][1]= // #574 Left foot FTsensor
   {
       {9.5469*0.0001},
       {9.6402*0.0001},
       {1.4234*0.0001},
       {1.0638*0.01},
       {1.0505*0.01},
       {1.6489*0.01},
   };
   const double offsetFT[6][1]= // #573 Right foot FTsensor
   {
       {32767},
       //{32703},
       {32625},
       //{32720},
       {32655},
       //{32689},
       {32585},
       //{32639},
       {32750},
       //{32663},
       {32550},
   };
   const double offsetFT1[6][1]= // #574 Left foot FTsensor
   {
       //{32689},
       {32660},
       //{32762},
       {32775},
       //{32761},
       {32785},
       //{32691},
       {32780},
       //{32749},
       {32675},
       //{32739},
       {32700},
   };
   const double gainFT[6][1]= // #573 Right foot FTsensor
   {
       {123.3246},
       {123.3079},
       {123.3880},
       {123.4881},
       {123.5682},
       {123.3213},
   };
 const   double gainFT1[6][1]= // #574 Left foot FTsensor
   {
       {123.5760},
       {123.5726},
       {123.5427},
       {123.5660},
       {123.5992},
       {123.4928},
   };
const    double ExFT[6][1]= // #573 Right foot FTsensor
   {
       {4.971},
       {4.971},
       {4.971},
       {4.971},
       {4.971},
       {4.971},
   };
 const   double ExFT1[6][1]= // #574 Left foot FTsensor
   {
       {4.9846},
       {4.9846},
       {4.9846},
       {4.9846},
       {4.9846},
       {4.9846},
   };

public:
    explicit ft_sensor(QObject *parent = nullptr);
    USBHID hid;

    bool Init(int pid, int vid);


    bool Read(QVector<double> &data);
signals:

public slots:
};

#endif // FT_SENSOR_H
