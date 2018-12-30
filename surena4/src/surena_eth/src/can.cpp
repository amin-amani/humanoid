#include "can.h"

Can::Can(QObject *parent ) : QObject(parent)
{

}
QByteArray Can::Int2QbyteArray(int value)
{
    QByteArray ba;


    for(int i = 0; i != sizeof(value); ++i)
    {
        ba.append((char)((value & (0xFF << (i*8))) >> (i*8)));
    }
    return ba;

}
bool Can::Init()
{
    //if(hid.Connect(pid,vid,handle))
    tcp.Connected();
    // ui->statusBar->showMessage("Device connected.");
    //  connect(&_timer,SIGNAL(timeout()),this,SLOT(Timeout()));
    // _timer.start(100);
    return true;
}

void Can::WaitMs(int timeout)
{
    QEventLoop q;
    QTimer tT;
    tT.setSingleShot(true);
    connect(&tT, SIGNAL(timeout()), &q, SLOT(quit()));
    tT.start(timeout); // 5s timeout
    q.exec();
    if(tT.isActive()){
      // download complete
      tT.stop();
    } else {

    }

}

void Can::WriteMessage(uint16_t canID,unsigned char devID,QByteArray data)
{
    QByteArray buffer;
    buffer.append(0x02);  // mode 2: Can Write
    buffer.append(0xAA);  // header 1
    buffer.append(0x55);  // header 2
    buffer.append(0xAA);  // header 3

    if(devID==255){
        buffer.append(devID);
        buffer.append((canID & 0xff)); //low
        buffer.append(((canID>>8)& 0xff));//hi
        buffer.append(data);

        buffer.append(0xFF);  // Tailer 1
        buffer.append(0xCC);  // Tailer 2
        buffer.append(0x33);  // Tailer 3
        buffer.append(0xCC);  // Tailer 4

        tcp.WriteData(buffer);
        return;
    }


    buffer.append(devID);
    buffer.append((canID & 0xff)); //low
    buffer.append(((canID>>8)& 0xff));//hi


    for(int i=0; i<8; i++)
    {
        buffer.append(data[i]);
    }

    tcp.WriteData(buffer);

}

 void Can::ReadMessage(uint8_t devID, QByteArray &response)
{
     QByteArray buffer;
     int index_a =10;
     response.clear();
     buffer.append(0x03);  // mode 3: Can Read
     buffer.append(0xAA);  // header 1
     buffer.append(0x55);  // header 2
     buffer.append(0xAA);  // header 3
     buffer.append(devID);

if(devID==255)
{
    index_a = 160;
}

     for (int i = 0; i < index_a; ++i) {
         buffer.append(0x1);  // 10 byte for data payload
     }
     buffer.append(0xFF);  // Tailer 1
     buffer.append(0xCC);  // Tailer 2
     buffer.append(0x33);  // Tailer 3
     buffer.append(0xCC);  // Tailer 4

    //tcp.WriteData(buffer);
    //WaitMs(5);
    response = tcp.SendCommand(buffer);
    response=response.mid(4,index_a);

    if (response.length() != 0)
    {
      //  qDebug() << " len: " << response.length() << "; incomming data is: " << response.toHex();
    }
    else
    {
       //qDebug() << "no data read!";
    }
}


void Can::SetPosition(QList<int>positionList)
{
    QByteArray buffer;
    buffer.append(0x01);  // mode 1: Run
    buffer.append(0xAA);  // header 1
    buffer.append(0x55);  // header 2
    buffer.append(0xAA);  // header 3
    for(int i=0;i<positionList.length();i++)
    {
      buffer.append(Int2QbyteArray(positionList[i]));
    }

    tcp.WriteData(buffer);
}

QByteArray Can::CanReceiveData()
{
    QByteArray buffer;
    //qDebug()<< tcp.Receive_data();
    buffer = tcp.Receive_data();
    return buffer;
}
