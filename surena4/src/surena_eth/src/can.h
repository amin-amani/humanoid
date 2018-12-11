#ifndef CAN_H
#define CAN_H

#include <QObject>
#include <QDebug>
#include <QtCore>
#include <QtGui>
#include <QtDebug>
#include <QTimer>

#include "tcphandler.h"

class Can : public QObject
{
    Q_OBJECT
public:
    // handle for our Ethernet device
    TcpHandler tcp;

 //bool Init(int pid, int vid, HANDLE handle);
    bool Init();
    void WaitMs (int timeout);


    explicit Can(QObject *parent = 0);

    void WriteMessage(uint16_t canID, unsigned char devID, QByteArray data);
    void SetPosition(QList<int> positionList);
    QByteArray Int2QbyteArray(int value);
    void ReadMessage(uint8_t devID, QByteArray &response);

    QByteArray CanReceiveData();

signals:
    void Dummy();
public slots:
};

#endif // CAN_H
