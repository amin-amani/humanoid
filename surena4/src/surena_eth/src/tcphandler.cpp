#include "tcphandler.h"

TcpHandler::TcpHandler(QObject *parent) : QObject(parent)
{

     qDebug() << "tcp handler";

    QString myIp;
    foreach (const QHostAddress &address, QNetworkInterface::allAddresses()) {
        if (address.protocol() == QAbstractSocket::IPv4Protocol && address != QHostAddress(QHostAddress::LocalHost))
             qDebug() << address.toString();
    myIp=address.toString();
    }

    addr_ip4.setAddress("192.168.1.10");

    _socket = new QUdpSocket(this);
    _socket->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    _socket->bind(QHostAddress::Any,7);

    connect(_socket,SIGNAL(readyRead()),this,SLOT(ReadyRead()));
    IsConnected=false;

}
//====================================================================================

void TcpHandler::WaitMs(int ms)
{
  QEventLoop q;
  QTimer tT;
  tT.setSingleShot(true);
  connect(&tT, SIGNAL(timeout()), &q, SLOT(quit()));
  tT.start(ms); // 5s timeout
  q.exec();
  if(tT.isActive()){
    // download complete
    tT.stop();
  } else {

  }


}
int TcpHandler::WriteData(QByteArray data)
{
    //_socket->waitForBytesWritten();
 int result =1;
 _socket->writeDatagram(data, addr_ip4, 7);
 _socket->flush();
 _socket->waitForBytesWritten();
//qDebug()<< result;
 return  result;
}

void TcpHandler::Disconnect()
{
    _socket->close();
}

QByteArray TcpHandler::SendCommand(QByteArray command)
{
    //incommingData.clear();
    WriteData(command);

WaitMs(5);
//    if (res)
//        qDebug() << "data received! data: "  << incommingData.toHex();
//    else
//        qDebug() << "data NOT received!" ;

    //WaitMs(5);
    return incommingData;
}


QByteArray TcpHandler::Receive_data()
{
    //incommingData.clear();
   // QSignalSpy spy(this,SIGNAL(NewDataReceived(QByteArray)));
    //QSignalSpy spy(,SIGNAL(readyRead()));
WaitMs(5);
    //if (res)
        qDebug() << "data received! data: "  << incommingData.toHex();
   // else
     //   qDebug() << "data NOT received!" ;

//    qDebug() << "Test RD!" ;
//    buffer.clear();
//    buffer.append(incommingData);
//    qDebug() << "incom: " << incommingData;
   // incommingData.clear();
    //qDebug() << "incommingData: " << incommingData.toHex();
    return incommingData;
}


void TcpHandler::Connected()
{
 qDebug()<<"connected" ;
IsConnected=true;
}


void TcpHandler::ClearBuffer()
{
    incommingData.clear();
}
/*

void TcpHandler::ReadyRead()
{
   // qDebug() <<"my incomming data:"<< _socket->readAll();
   // emit NewDataReceived();
    QByteArray datagram;
   // qDebug() << "in !";
   QHostAddress sender;
   quint16 port;
   while (_socket->hasPendingDatagrams())
   {
      //QByteArray datagram;
      datagram.resize(_socket->pendingDatagramSize());
      _socket->readDatagram(datagram.data(),datagram.size(),&sender,&port);

      incommingData.append(datagram);

      qDebug() <<"Message From: " << sender.toString() <<"Port From: "<< port << " size: " << datagram.count();
      qDebug() <<"Message: " << datagram.toHex();
      qDebug() <<"Message: " << incommingData.toHex();
   }
}

 */
void TcpHandler::ReadyRead()
{
   // qDebug() <<"my incomming data:"<< _socket->readAll();
   //
    QByteArray datagram;
    datagram.clear();
   // qDebug() << "in !";
   QHostAddress sender;
   quint16 port;
   while (_socket->hasPendingDatagrams())
   {
      //QByteArray datagram;
      datagram.resize(_socket->pendingDatagramSize());
      _socket->readDatagram(datagram.data(),datagram.size(),&sender,&port);
incommingData.clear();
      incommingData.append(datagram);


      //qDebug() <<"Message From Addr: " << sender.toString() <<"; Port: "<< port << "; size: " << datagram.count();
      //qDebug() <<"Message: " << datagram.toHex();
     // qDebug() <<"Message: " << incommingData.toHex();
      emit NewDataReceived(incommingData);
   }
}

void TcpHandler::Disconnected()
{
     qDebug()<<"disconnected" ;
     IsConnected=false;
}

