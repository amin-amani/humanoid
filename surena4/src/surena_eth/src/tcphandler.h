#ifndef TCPHANDLER_H
#define TCPHANDLER_H

#include <QObject>
#include <QDebug>
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QUdpSocket>
#include <QtNetwork/QtNetwork>
#include <QTimer>
#include <QEventLoop>


class TcpHandler : public QObject
{
    Q_OBJECT
    QUdpSocket *_socket;
    QHostAddress addr_ip4; // ("192.168.1.10");
private:
    QByteArray incommingData;
    QByteArray buffer;

public:
    explicit TcpHandler(QObject *parent = 0);
    int WriteData(QByteArray);
    bool IsConnected;
    void Disconnect();
    QByteArray SendCommand(QByteArray command);
    QByteArray Receive_data();
    void WaitMs(int timeout);
    void ClearBuffer();

signals:
   void NewDataReceived(QByteArray);
   void Dummy();
public slots:
    void Connected();
    void ReadyRead();
    void Disconnected();

};

#endif // TCPHANDLER_H
