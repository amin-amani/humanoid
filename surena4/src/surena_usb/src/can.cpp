#include "can.h"
//===============================================================================================================

Can::Can(QObject *parent ) : QObject(parent)
{

}
//===============================================================================================================

void Can::WaitMs(int ms)
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
//===============================================================================================================

QByteArray Can::Int2QbyteArray(int value)
{
    QByteArray ba;
    for(int i = 0; i != sizeof(value); ++i)
    {
        ba.append((char)((value & (0xFF << (i*8))) >> (i*8)));
    }
    return ba;

}
//===============================================================================================================

bool Can::Init(int pid,int vid)
{

    if(hid.Connect(pid,vid))
    {
        return true;
    }

    return false;
}
//===============================================================================================================

bool Can::WriteMessage(uint16_t canID,unsigned char devID,QByteArray data)
{
    QByteArray buffer;
    buffer.insert(0,data,8);
    buffer[8]=(canID>>8);
    buffer[9]=(canID&0xff);
    buffer[10]=devID;
    buffer[60]=1;
    return hid.Write(buffer);

}

//===============================================================================================================


bool Can::ReadMessage(uint8_t devID,QByteArray &buffer)
{
    bool result=false;
    buffer.resize(64);
    buffer[10]=devID;
    buffer[60]=2;
    //qDebug()<<buffer.toHex();
    result=hid.Write(buffer);
    if(!result)return false;
    WaitMs(10);
    return  hid.Read(buffer);

}
//===============================================================================================================

bool Can::SetPosition(QList<int>positionList)
{
    QByteArray buffer;
    for(int i=0;i<positionList.length();i++)
        buffer.append(Int2QbyteArray(positionList[i]));
    buffer[60]=0;
    if( !hid.Write(buffer))
    {
        qDebug()<<"error send";
        return false;
    }
    return true;
}
//===============================================================================================================
