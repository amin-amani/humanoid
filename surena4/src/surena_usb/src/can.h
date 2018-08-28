#ifndef CAN_H
#define CAN_H

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

class Can : public QObject
{
    Q_OBJECT
public:
    // handle for our USB device
    USBHID hid;

 bool Init(int pid, int vid);

    explicit Can(QObject *parent = 0);

    bool WriteMessage(uint16_t canID, unsigned char devID, QByteArray data);
    bool ReadMessage(uint8_t devID, QByteArray &buffer);

    bool SetPosition(QList<int> positionList);
    QByteArray Int2QbyteArray(int value);
    //  bool ReadMessage(uint8_t devID, QByteArray &buffer);
    void WaitMs(int ms);
signals:
void Dummy();
public slots:
};

#endif // CAN_H
