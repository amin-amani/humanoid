#include "epos.h"
//========================================================================
Epos::Epos(QObject *parent) : QObject(parent)
{
//    while (!tcp.IsConnected) {
//    }
    for(int i=0;i<8;i++)
    {

    bump_sensor_list.append(0x1);

    }
    for(int i=0;i<16;i++)
    {

positions.append((char)0x00);
positionsInc.append((char)0x00);
imu_data_list.append((char)0x00);
  ft.append((char)0x00);
    }


    qDebug()<<"all printed values are hex";
   // connect(&tcp,SIGNAL(NewDataReceived()),this,SIGNAL(NewDataReady()));
    connect(&tcp,SIGNAL(NewDataReceived(QByteArray)),this,SLOT(DataReceived(QByteArray)));

}
//=================================
bool Epos::CheckZynq()
{
    int i=0;
    QByteArray data;
    uchar checkPoint[8]={0x00,0x55,0xaa,0x55,0xff,0xcc,0x33,0xcc};
    QByteArray ck;

    data.append((char)0x00);  // mode 0: Test
    data.append(0xAA);  // Header check
    data.append(0x55);  // Header check
    data.append(0xAA);  // Header check



    data.append(0xFF);  // Parity | 0xFF
    data.append(0xCC);  // Tailer
    data.append(0x33);  // Tailer
    data.append(0xCC);  // Tailer

  QByteArray resp=  tcp.SendCommand(data);  // send 128 Byte Packet
//qDebug()<<"packet size="<<LastPacketreceived.toHex()<<QByteArray::ad("0x00,55aa55ffcc33cc").toHex();
  if(LastPacketreceived.toHex()=="0055aa55ffcc33cc")
  {

  qDebug()<<"valid resp!";
  return true;
  }
  return false;

    // just for test
    // qDebug() << "send data: " << data.toHex();
}
//========================================================================
EPOSErrors Epos::Init(int tryCount)
{
static bool lastresult=false;


    int numberOfSuccess=0;
    int pingCount=0;

    if(lastresult)return OK;
       qDebug()<<"ping";
    while(! ping.Start("192.168.1.10") && pingCount<tryCount )
    {
        QThread::msleep(500);
            qDebug()<<"waiting zynq ethernet..";
        pingCount++;
    }
if(pingCount==tryCount)return NETWOR_ERROR;

      qDebug()<<"Ping OK";
//===========================
QList<int32_t> values;

if(ReadAllRegisters(0x1000,0,1,values,10)!=OK)
{

    qDebug()<<"Read error";
    return NO_ANSER;
}
if(values.count()<12){    qDebug()<<"response lengh error"<<values.count();return SDO_REJECT;}
for(int i=0;i<12;i++){
qDebug()<<"read val "<<QString::number(i)<<"="<<QString::number(values[i],16);
if(values[i]!=0x20192)return SDO_BAD_REPLAY;
}
//===========================
//      for(int i=0;i<12;i++){
//    int32_t val=0;
//    if(ReadRegister(0x1000,0,1,i,val,10,3)==OK && val==0x20192){
//      numberOfSuccess++;
//        qDebug()<<"OK";
//    }
//qDebug()<<"read val "<<QString::number(i)<<"="<<QString::number(val,16);
//    }

//if(numberOfSuccess!=12)
//{
//return EPOS_ERROR;
//}
lastresult=true;
    return OK;

}
//========================================================================
EPOSErrors Epos::HandsInit(int tryCount)
{
static bool lastresult=false;
//if(lastresult)return OK;
    int numberOfSuccess=0;
//===========================
QList<int32_t> values;
for(int i=0;i<4;i++){
    int32_t value=0;
if(ReadRegister(0x1000,0,i+1,13,value,10,3)==OK)
{
//qDebug()<<"read"<<QString::number(i+1)<<"...OK"<<QString::number(value);
values.append(value);
numberOfSuccess++;
}

}
if(numberOfSuccess==4){
    //qDebug()<<"init ok";
    lastresult=true;
    return OK;

}
else return SDO_BAD_REPLAY;

}
//========================================================================
EPOSErrors Epos::ResetNode(int devID) // if devID=255 -> reset all nodes
{
    QByteArray data;
    data.append(0x81);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    can.WriteMessage(0,devID,data);
    return OK;
}
//========================================================================
EPOSErrors Epos::EnableDevice(int devID,EPOSOperationMode mode)
{
    QThread::msleep(700);
    SetPreoperationalMode(devID);
    QThread::msleep(700);
    StartNode(devID);
    QThread::msleep(700);
    SetMode(devID,mode);
    QThread::msleep(700);
    SwitchOff(devID);
    QThread::msleep(700);
    SwitchOn(devID);
    QThread::msleep(700);
    return OK;
}
//========================================================================
void Epos::SetPreoperationalMode(int devID,int nodeID)
{
    QByteArray data;
    data.append(0x80);     // ???????????? check 0x80 (for 0x01)
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    can.WriteMessage(00,devID,data);
}

//========================================================================
void  Epos::ResetComunication(int devID)
{
    QByteArray data;
    data.append(0x82);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    can.WriteMessage(0,devID,data);
}
//========================================================================
void Epos::StartNode(int devID)
{
    QByteArray data;
    data.append(0x01);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    can.WriteMessage(0,devID,data);
}
//========================================================================
void Epos::StopNode(int devID)
{
    QByteArray data;
    data.append(0x02);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    can.WriteMessage(0,devID,data);
}
//========================================================================
void Epos::SetMode(int devID,EPOSOperationMode mode,int canID)
{
    QByteArray data;
    data.append(mode);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);

    can.WriteMessage(0x300+canID,devID,data);
}
//========================================================================
void Epos::SwitchOn(int devID ,int canID)
{
    QByteArray data;
    data.append(0x0f);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);

    can.WriteMessage(0x200+canID,devID,data);
}
//========================================================================
void Epos::SwitchOff(int devID,int canID)
{

    QByteArray data;
    data.append(0x06);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);

    can.WriteMessage(0x200+canID,devID,data);
}
//========================================================================
unsigned char Epos::GetSDOCODE(int len)
{
    if (len == 1) { return 0x2f; }
    if (len == 2) { return 0x2b; }
    if (len == 4) { return 0x23; }
    if (len == -1) { return 0x22; }
    if (len == 0) { return 0x40; }
    if (len == 9) { return 0x81; }
    return 0;
}
//========================================================================
EPOSErrors Epos::SDOWriteCommand( int id,unsigned long int input,int index,unsigned char subIndex,unsigned char len,char devID)
{
    //epos 2 application note collection page 155
    QByteArray replay;
    int16_t inp_index=0;//,inpid,;
    QByteArray data;

    data.append (GetSDOCODE(len));
    data.append((unsigned char)(index & 0xff));
    data.append((index >> 8)&0xff);
    data.append(subIndex&0xff);
    data.append((char)input & 0xff);
    data.append((char)((input >> 8) & 0xff));
    data.append((char)(input >> 16) & 0xff);
    data.append((char)(input >> 24) & 0xff);

    can.WriteMessage(id+0x600, devID,data);


    QThread::msleep(5);

    can.ReadMessage(devID,replay);    // !!!!!!!!!!!!!!!!  check this

    if (replay.length() < 10)
    {
        return SDO_REJECT;
    }

    inp_index=(replay[8]&0xff);
    inp_index<<=8;
    inp_index+=(replay[9]&0xff);


   // qDebug()<<replay[11];
   // qDebug()<<"replay="<<QByteArray::fromRawData((const char *)replay,12).toHex();
//    if(replay[11]!=0){
//        return EPOSErrors::OK;
//    }
    return OK;
}
//========================================================================
void Epos::WaitMs(int ms)
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
//========================================================================
bool Epos::ActiveHand() //13,2
{
    WaitMs(700);
    for(int i=0 ;i<4;i++){
    SetPreoperationalMode(12,i+1);
        SetPreoperationalMode(13,i+1);
}
    WaitMs(700);

    StartNode(12);
    StartNode(13);

    WaitMs(700);
        for(int i=0 ;i<4;i++){
        SetMode(12,PPM,i+1);
        SetMode(13,PPM,i+1);
        }
    WaitMs(700);
        for(int i=0 ;i<4;i++){
        SwitchOff(12,i+1);
        SwitchOff(13,i+1);
        }
    WaitMs(700);
        for(int i=0 ;i<4;i++){
        SwitchOn(12,i+1);
        SwitchOn(13,i+1);
        }
    WaitMs(700);
    return OK;
}

//========================================================================
bool Epos::ActivePPMPDO(int nodeID,int canID) //13,2
{
    WaitMs(700);
    SetPreoperationalMode(nodeID,canID);
    WaitMs(700);
    StartNode(nodeID);
    WaitMs(700);
    SetMode(nodeID,PPM,canID);
    WaitMs(700);
    SwitchOff(nodeID,canID);
    WaitMs(700);
    SwitchOn(nodeID,canID);
    WaitMs(700);
    return OK;
}
//========================================================================
bool Epos::ActiveCSP(int nodeID)
{
    WaitMs(700);
    SetPreoperationalMode(nodeID);
    WaitMs(700);
    StartNode(nodeID);
    WaitMs(700);
    SetMode(nodeID,CSP);
    WaitMs(700);
    SwitchOff(nodeID);
    WaitMs(700);
    SwitchOn(nodeID);
    WaitMs(700);
    return OK;
}


//========================================================================
bool Epos::AllActiveCSP()
{
   int i=0;
   WaitMs(700);
   for(i=0; i< 12; i++) {
    SetPreoperationalMode(i);
   }
   WaitMs(700);
   for(i=0; i< 12; i++) {
    StartNode(i);
   }
   WaitMs(700);
    for(i=0; i< 12; i++) {
        SetMode(i,CSP);
    }
    WaitMs(700);
    for(i=0; i< 12; i++) {
        SwitchOff(i);
    }
    WaitMs(700);
    for(i=0; i< 12; i++) {
        SwitchOn(i);
    }
    WaitMs(700);
    return OK;
}
//========================================================================
void Epos::SDOReadCommand( int id,int index,unsigned char subIndex,char devID)
{
    QByteArray data;
    data.append ((unsigned char)GetSDOCODE(0));
    data.append((unsigned char)(index & 0xff));
    data.append((index >> 8)&0xff);
    data.append(subIndex&0xff);
    data.append((char) 0x00);
    data.append((char) 0x00);
    data.append((char) 0x00);
    data.append((char) 0x00);
    can.WriteMessage(id,devID,data);
}
//========================================================================
EPOSErrors Epos::ReadRegister(int index,int subIndex,int canID, int devID,int32_t &value,int timeout,int trycount)
{
    int tryCounter=0;
    EPOSErrors status=EPOS_ERROR;
    while (status!=OK && tryCounter<trycount) {
     status=   ReadRegisterValue(index,subIndex,canID,devID,value,timeout);
    tryCounter++;
    }
 return status;
}
//========================================================================
EPOSErrors Epos::ReadAllRegisters(int index,int subIndex,int canID,QList< int32_t> &value,int timeout)
{
    uint16_t inpid;
    QByteArray replay;


    can.ReadMessage(255, replay);
    QThread::msleep(timeout);
    can.ReadMessage(255, replay);
    QThread::msleep(timeout);

    can.ReadMessage(255, replay);
    SDOReadCommand(canID+0x600,index,subIndex,255);
    QThread::msleep(timeout);

    SDOReadCommand(canID+0x600,index,subIndex,255);

    QThread::msleep(timeout);
  qDebug()<<"Read all";
    can.ReadMessage(255,replay);
    if(replay.length()!=160)
    {
        //qDebug()<<"Invalid lengh!"<<replay.toHex()<<replay.length();
        return SDO_BAD_REPLAY;
    }
//qDebug()<<"packet!"<<replay.toHex();
for(int i=0;i<12;i++)

{
    inpid=replay[i*10+1]&0xff;
    inpid<<=8;
    inpid+=replay[i*10+0]&0xff;
    if(inpid==(0x580+1))
    {
        // PAGE 150 APPLICATION NOTE COLLECTIOIN
    int tempValue=0;
        tempValue=replay[i*10+9]&0xff;
        tempValue=tempValue<<8;
        tempValue+=replay[i*10+8]&0xff;
        tempValue=(tempValue)<<8;
        tempValue+=replay[i*10+7]&0xff;
        tempValue=(tempValue)<<8;
        tempValue+=replay[i*10+6]&0xff;
        value.append(tempValue);
//       qDebug()<<"inja"<<QString::number(tempValue);

    }
    else
    {
        qDebug()<<"Invalid ID!"<<QString::number( inpid,16);
                return SDO_BAD_REPLAY;
    }
}


    //qDebug()<<"inp id"<<QString::number(inpid,16);// inpid=replay
  // qDebug()<<"inp "<<QString::number(canID+0x580,16);

    return OK;
}
/////////////////////////////////////////
//EPOSErrors Epos::ReadRegister(int index,int subIndex,int canID, int devID,int32_t &value,int timeout,int trycount)
//{
//    int tryCounter=0;
//    EPOSErrors status=EPOS_ERROR;
//    while (status!=OK && tryCounter<trycount) {
//     status=   ReadRegisterValue(index,subIndex,canID,devID,value,timeout);
//    tryCounter++;
//    }
// return status;
//}
//========================================================================
EPOSErrors Epos::ReadRegisterValue(int index,int subIndex,int canID, int devID,int32_t &value,int timeout)
{
    uint16_t inpid;
    QByteArray replay;

    can.ReadMessage(devID, replay);
    can.ReadMessage(devID, replay);
    SDOReadCommand(canID+0x600,index,subIndex,devID);
    //QThread::msleep(timeout);
    WaitMs(timeout);

    can.ReadMessage(devID,replay);

    inpid=replay[1]&0xff;
    inpid<<=8;
    inpid+=replay[0]&0xff;

    //qDebug()<<"inp id"<<QString::number(inpid,16);// inpid=replay
  // qDebug()<<"inp "<<QString::number(canID+0x580,16);
    if((inpid==0x580+canID))
    {
        // PAGE 150 APPLICATION NOTE COLLECTIOIN
        value=0;
        value=replay[9]&0xff;
        value=value<<8;
        value+=replay[8]&0xff;
        value=(value)<<8;
        value+=replay[7]&0xff;
        value=(value)<<8;
        value+=replay[6]&0xff;
      //  qDebug()<<"inja"<<QString::number(value);
        return OK;
    }
    return SDO_BAD_REPLAY;
}
//========================================================================
EPOSErrors Epos::WriteRegister(int index,int subIndex,int canID, int devID,int32_t value,int len)
{
    QByteArray data_read;
    can.ReadMessage(devID, data_read);
    QThread::msleep(5);
    can.ReadMessage(devID, data_read);
    QThread::msleep(5);
    return SDOWriteCommand(canID,value,index,subIndex,len,devID);
}
//========================================================================
EPOSErrors Epos::ActivePPM(int canID,int devId){

    qDebug()<<"pvt";
    //return WriteRegister(0x607A,0,canID,devId,4);
    WriteRegister(0x6060,0,canID,devId,PPM,1);//set mode
    QThread::sleep(1);
    WriteRegister(0x6040,0,canID,devId,0x06,2);//set mode
    QThread::sleep(1);
    WriteRegister(0x6040,0,canID,devId,0x0f,2);//set mode
    QThread::sleep(1);
    WriteRegister(0x6040,0,canID,devId,0x010f,2);//set mode

OK;
}
//========================================================================
QString Epos::ReadCurrentError(int canID,int devID)
{
    EPOSErrors status;
    int value=0;
    // status= ReadRegister(0x1001,0,canID,devID,value);
    status= ReadRegister(0x603f,0,canID,devID,value,10,3);
    if(status!=OK){ return "read error";}
    qDebug()<<"error="<<QString::number( value,16);
    return "ok";
}
//========================================================================
bool Epos::SetPosition(int canID,int devId,int position,int velocity)
{
    bool status=false;
    status=WriteRegister(0x6081,0,canID,devId,velocity,4);
    if(status!=true)//set velocity
    {
        qDebug()<<"set velocity error";
        return status;

    }
    status=WriteRegister(0x6040,0,canID,devId,0x0f,2);
    if(status !=true)
    {
        qDebug()<<"switch off error";
        return status;
    }
    status=WriteRegister(0x607A,0,canID,devId,position,4);
    if(status!=true)
    {
        qDebug()<<"switch on error";
        return status;
    }
    status=WriteRegister(0x6040,0,canID,devId,0x3f,2);
    if(status!=true){
        qDebug()<<"switch on error";
        return status;
    }

    //set mode
    return true;
}
//========================================================================
void Epos::can_write(unsigned char motor_index,unsigned int can_id, unsigned char* data_buff)
{
    qDebug()<<"Can Write to CS:" << motor_index << ", ID:" << can_id ;

    QByteArray data;
    data.append(0x02);  // mode 2: Can Write
    data.append(0xAA);  // header 1
    data.append(0x55);  // header 2
    data.append(0xAA);  // header 3
    data.append(motor_index);
    data.append(((can_id>>8)& 0xff));
    data.append((can_id & 0xff));

    for(int i=0; i<8; i++)
    {
        data.append(data_buff[i]);
    }

    tcp.WriteData(data);
}
//========================================================================
void Epos::can_read_test(unsigned char motor_index)
{
    qDebug()<<"id:" << motor_index;

    QByteArray data;
    data.append(0x03);  // mode 3: Can Read
    data.append(0xAA);  // header 1
    data.append(0x55);  // header 2
    data.append(0xAA);  // header 3
    data.append(motor_index);
    for (int i = 0; i < 9; ++i) {
        data.append(0x01);  // 9+1 byte for data payload
    }
    data.append(0xFF);  // Tailer 1
    data.append(0xCC);  // Tailer 2
    data.append(0x33);  // Tailer 3
    data.append(0xCC);  // Tailer 4

    tcp.WriteData(data);  // 18 byte packet size
}
//========================================================================
void Epos::SetPositionCST(int position,int velocity)
{
    QByteArray data;
    data.append(0x01);  // mode 1: Run
    data.append(0xAA);  // header 1
    data.append(0x55);  // header 2
    data.append(0xAA);  // header 3

    for(int i=0; i<12; i++)  // repeat one data 12 (send 48 byte), we must edit this function
    {
        data.append((position >> 0) & 0xff);
        data.append((position >> 8) & 0xff);
        data.append((position >> 16) & 0xff);
        data.append((position >> 24) & 0xff);
     }
    tcp.WriteData(data);

    // just for test
    //qDebug() << "send data: " << data.toHex();
}
//========================================================================
void  Epos::CheckCanBoardRequest()
{
    int i=0;
    QByteArray data;
    data.append(0x08);  // mode 8: Test
    data.append(0xAA);  // Header check
    data.append(0x55);  // Header check
    data.append(0xAA);  // Header check



    for(i = 0 ; i < 32; i++)
    {
        data.append(0x01);  // fill data packet with 01 byte
    }

    data.append(0xFF);  // Parity | 0xFF
    data.append(0xCC);  // Tailer
    data.append(0x33);  // Tailer
    data.append(0xCC);  // Tailer
//qDebug()<<"packet size="<<data.length();
    tcp.WriteData(data);  // send 128 Byte Packet

    // just for test
    // qDebug() << "send data: " << data.toHex();
}
//========================================================================

//========================================================================
void Epos::SetAllPositionCST(QList<int> all_position, int motor_num)
{
    int i=0;
    QByteArray data;
    static int flag_sensor_pa=0;
    static int handDeviceID=0;


    data.append(0x01);  // mode 1: Run
    data.append(0xAA);  // Header check
    data.append(0x55);  // Header check
    data.append(0xAA);  // Header check

    for(i=0; i< motor_num; i++)  // send 12 data motor (72 byte)
    {
        data.append(0x04); // can id high byte   0401
        data.append(0x01); // can id low byte    0401

        data.append((all_position[i] >> 0) & 0xff);
        data.append((all_position[i] >> 8) & 0xff);
        data.append((all_position[i] >> 16) & 0xff);
        data.append((all_position[i] >> 24) & 0xff);
     }
    //76 t ainja
//qDebug()<<handDeviceID;
  // send 4 data can(each can 10 byte) (40 byte)
    ///////////////////////for 4 maxon motors
if(handDeviceID<4){
//right hand
    data.append(0x4);
    data.append(1+handDeviceID);


    data.append((all_position[12+handDeviceID] >> 0) & 0xff);
    data.append((all_position[12+handDeviceID] >> 8) & 0xff);
    data.append((all_position[12+handDeviceID] >> 16) & 0xff);
    data.append((all_position[12+handDeviceID] >> 24) & 0xff);
    data.append((char)0x3f);
    data.append((char)0x00);
    data.append((char)00);
    data.append((char)0x00);

    ///left hand
    data.append(0x4);
    data.append(0x01+handDeviceID);

    data.append((all_position[12+handDeviceID+8] >> 0) & 0xff);
    data.append((all_position[12+handDeviceID+8] >> 8) & 0xff);
    data.append((all_position[12+handDeviceID+8] >> 16) & 0xff);
    data.append((all_position[12+handDeviceID+8] >> 24) & 0xff);

    data.append((char)0x3f);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);

    handDeviceID++;


}
else if(handDeviceID> 3 && handDeviceID<7)
{
handDeviceID++;
data.append(0x9);
data.append(1+handDeviceID);


data.append((all_position[12+handDeviceID] >> 0) & 0xff);
data.append((all_position[12+handDeviceID] >> 8) & 0xff);
data.append((all_position[12+handDeviceID] >> 16) & 0xff);
data.append((all_position[12+handDeviceID] >> 24) & 0xff);
data.append((char)0x00);
data.append((char)0x00);
data.append((char)00);
data.append((char)0x00);

///left hand
data.append(0x9);
data.append(0x01+handDeviceID);

data.append((all_position[12+handDeviceID+8] >> 0) & 0xff);
data.append((all_position[12+handDeviceID+8] >> 8) & 0xff);
data.append((all_position[12+handDeviceID+8] >> 16) & 0xff);
data.append((all_position[12+handDeviceID+8] >> 24) & 0xff);

data.append((char)0x00);
data.append((char)0x00);
data.append((char)0x00);
data.append((char)0x00);
}
else if(handDeviceID==7){


    data.append(0x9);
    data.append(1+handDeviceID);


    data.append((all_position[12+handDeviceID] >> 0) & 0xff);
    data.append((all_position[12+handDeviceID] >> 8) & 0xff);
    data.append((all_position[12+handDeviceID] >> 16) & 0xff);
    data.append((all_position[12+handDeviceID] >> 24) & 0xff);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)00);
    data.append((char)0x00);

    ///left hand
    data.append(0x9);
    data.append(0x01+handDeviceID);

    data.append((all_position[12+handDeviceID+8] >> 0) & 0xff);
    data.append((all_position[12+handDeviceID+8] >> 8) & 0xff);
    data.append((all_position[12+handDeviceID+8] >> 16) & 0xff);
    data.append((all_position[12+handDeviceID+8] >> 24) & 0xff);

    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);
    data.append((char)0x00);

     handDeviceID++;

}
else if(handDeviceID==8){//start move into controlworld

    data.append(0x5);
    data.append(1);



    data.append((char)0x0f);
    data.append((char)0x00);
    data.append((char)00);
    data.append((char)0x00);
    data.append((char)0x0f);
    data.append((char)0x00);
    data.append((char)00);
    data.append((char)0x00);


    data.append(0x5);
    data.append(1);

    data.append((char)0x0f);
    data.append((char)0x00);
    data.append((char)00);
    data.append((char)0x00);
    data.append((char)0x0f);
    data.append((char)0x00);
    data.append((char)00);
    data.append((char)0x00);

    handDeviceID=0;

}
//                   // CS = 12  ==> Right Hand
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                    // CS = 13  ==> Left Hand
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
                   // CS = 14  ==>  Neck and Other

                data.append(0x01);
                data.append(0x01);
                data.append(0x01);
                data.append(0x01);
                data.append(0x01);
                data.append(0x01);
                data.append(0x01);
                data.append(0x01);
                data.append(0x01);
                data.append(0x01);


                // 2 sensor Pa
                data.append(0x03); // can id high byte   0401
                if (flag_sensor_pa==1)
                {
                    data.append(0x82); // can id low byte    0401
                    flag_sensor_pa = 0;
                }
                else
                {
                    data.append(0x83); // can id low byte    0401
                    flag_sensor_pa =1 ;
                }
                data.append(0x01);
                data.append(0x01);
                data.append(0x01);
                data.append(0x20);  // command to sensor pa
                data.append(0x01);
                data.append(0x01);
                data.append(0x01);
                data.append(0x01);





    for(i = (6*motor_num)+44 ; i < 296; i++)
    {
        data.append(0x01);  // fill data packet with 01 byte
    }

    data.append(0xFF);  // Parity | 0xFF
    data.append(0xCC);  // Tailer
    data.append(0x33);  // Tailer
    data.append(0xCC);  // Tailer
//qDebug()<<"packet size="<<data.length();
   tcp.WriteData(data);  // send 128 Byte Packet

    // just for test
    // qDebug() << "send data: " << data.toHex();
}
//void Epos::SetAllPositionCST(QList<int> all_position, int motor_num)
//{
//    int i=0;
//    QByteArray data;
//    data.append(0x01);  // mode 1: Run
//    data.append(0xAA);  // Header check
//    data.append(0x55);  // Header check
//    data.append(0xAA);  // Header check

//    for(i=0; i< motor_num; i++)  // send 12 data motor (72 byte)
//    {
//        data.append(0x04); // can id high byte   0401
//        data.append(0x01); // can id low byte    0401

//        data.append((all_position[i] >> 0) & 0xff);
//        data.append((all_position[i] >> 8) & 0xff);
//        data.append((all_position[i] >> 16) & 0xff);
//        data.append((all_position[i] >> 24) & 0xff);
//     }

//    for(i=0; i< 4; i++)  // send 4 data can(each can 10 byte) (40 byte)
//        {
//            switch (i) {
//            case 0:                 // CS = 12  ==> Right Hand
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                break;

//            case 1:               // CS = 13  ==> Left Hand
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                break;

//            case 2:               // CS = 14  ==>  Neck and Other

//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                break;

//            case 3:  // 2 sensor Pa
//                data.append(0x03); // can id high byte   0401
//                if (flag_sensor_pa==1)
//                {
//                    data.append(0x82); // can id low byte    0401
//                    flag_sensor_pa = 0;
//                }
//                else
//                {
//                    data.append(0x83); // can id low byte    0401
//                    flag_sensor_pa =1 ;
//                }
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x20);  // command to sensor pa
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                data.append(0x01);
//                break;

//            default:
//                break;
//            }

//         }


//    for(i = (6*motor_num)+44 ; i < 296; i++)
//    {
//        data.append(0x01);  // fill data packet with 01 byte
//    }

//    data.append(0xFF);  // Parity | 0xFF
//    data.append(0xCC);  // Tailer
//    data.append(0x33);  // Tailer
//    data.append(0xCC);  // Tailer
////qDebug()<<"packet size="<<data.length();
//    tcp.WriteData(data);  // send 128 Byte Packet

//    // just for test
//    // qDebug() << "send data: " << data.toHex();
//}
//========================================================================
EPOSErrors Epos::test_arya(int devID) // if devID=255 -> reset all nodes
{
    QByteArray data_read;
    can.ReadMessage(devID, data_read);
    return OK;
}
//========================================================================
QByteArray Epos::EposReceiveData()
{
    QByteArray buffer;
    //tcp.Receive_data();
     buffer = can.CanReceiveData();
     return buffer;
}
//========================================================================
float Epos::QByteArrayToFloat(QByteArray arr)
{
    float output;
    char* point_char = (char*)&output;
    *point_char = (arr[3] & 0xff );
    point_char++;
    *point_char = (arr[2] & 0xff );
    point_char++;
    *point_char = (arr[1] & 0xff );
    point_char++;
    *point_char = (arr[0] & 0xff );
    point_char++;
    return output;
}
//========================================================================
void Epos::DataReceived(QByteArray data)
{
    LastPacketreceived=data;
    incommingPacket=(EthernetReceivedPacketType*)data.data();

    QByteArray mid= data.mid(154,10);
    bumpPacket=(BumpSensorPacket*)mid.data();

    if ( (data[0]&0xff)== 0x01 && (data[1]&0xff)== 0x55 && (data[2]&0xff)== 0xAA && (data[3]&0xff)== 0x55 )
    {
        //--------------------
        for(int i=0;i<16;i++){
    positionsInc[i]=(incommingPacket->MotorData[i].Valu1);
    positions[i]=(incommingPacket->MotorData[i].Valu2);
        }

        //---
        if((bumpPacket->ID&0xffff)==0x0481){

            for(int i=0;i<4;i++){
        bump_sensor_list[i]=bumpPacket->data[i]&0xffff;

        }


        }

        if((bumpPacket->ID&0xffff)==0x0482){

            for(int i=0;i<4;i++){
        bump_sensor_list[i+4]=bumpPacket->data[i]&0xffff;

        }

        }

      //-------
ForceTorqSensorLeft.force.x=incommingPacket->FTsensor[0];
ForceTorqSensorLeft.force.y=incommingPacket->FTsensor[1];
ForceTorqSensorLeft.force.z=incommingPacket->FTsensor[2];
ForceTorqSensorLeft.torque.x=incommingPacket->FTsensor[3];
ForceTorqSensorLeft.torque.y=incommingPacket->FTsensor[4];
ForceTorqSensorLeft.torque.z=incommingPacket->FTsensor[5];

ForceTorqSensorRight.force.x=incommingPacket->FTsensor[6];
ForceTorqSensorRight.force.y=incommingPacket->FTsensor[7];
ForceTorqSensorRight.force.z=incommingPacket->FTsensor[8];
ForceTorqSensorRight.torque.x=incommingPacket->FTsensor[9];
ForceTorqSensorRight.torque.y=incommingPacket->FTsensor[10];
ForceTorqSensorRight.torque.z=incommingPacket->FTsensor[11];
 //5000 * (? / (((double)65535) * (gainFTnow) * (sensitivityFTnow) * (ExFTnow)));
ForceTorqSensorRight.force.x -=offsetFTRight[0];
ForceTorqSensorRight.force.y -=offsetFTRight[1];
ForceTorqSensorRight.force.z -=offsetFTRight[2];
ForceTorqSensorRight.torque.x-=offsetFTRight[3];
ForceTorqSensorRight.torque.y-=offsetFTRight[4];
ForceTorqSensorRight.torque.z-=offsetFTRight[5];

ForceTorqSensorLeft.force.x -=offsetFTLeft[0];
ForceTorqSensorLeft.force.y -=offsetFTLeft[1];
ForceTorqSensorLeft.force.z -=offsetFTLeft[2];
ForceTorqSensorLeft.torque.x-=offsetFTLeft[3];
ForceTorqSensorLeft.torque.y-=offsetFTLeft[4];
ForceTorqSensorLeft.torque.z-=offsetFTLeft[5];

ForceTorqSensorRight.force.x/=(65535*gainFTRight[0]*sensitivityFTRight[0]*ExFTRight[0])/5000;
ForceTorqSensorRight.force.y/=(65535*gainFTRight[1]*sensitivityFTRight[1]*ExFTRight[1])/5000;
ForceTorqSensorRight.force.z/=(65535*gainFTRight[2]*sensitivityFTRight[2]*ExFTRight[2])/5000;
ForceTorqSensorRight.torque.x/=(65535*gainFTRight[3]*sensitivityFTRight[3]*ExFTRight[3])/5000;
ForceTorqSensorRight.torque.y/=(65535*gainFTRight[4]*sensitivityFTRight[4]*ExFTRight[4])/5000;
ForceTorqSensorRight.torque.z/=(65535*gainFTRight[5]*sensitivityFTRight[5]*ExFTRight[5])/5000;

ForceTorqSensorLeft.force.x/=(65535*gainFTLeft[0]*sensitivityFTLeft[0]*ExFTLeft[0])/5000;
ForceTorqSensorLeft.force.y/=(65535*gainFTLeft[1]*sensitivityFTLeft[1]*ExFTLeft[1])/5000;
ForceTorqSensorLeft.force.z/=(65535*gainFTLeft[2]*sensitivityFTLeft[2]*ExFTLeft[2])/5000;
ForceTorqSensorLeft.torque.x/=(65535*gainFTLeft[3]*sensitivityFTLeft[3]*ExFTLeft[3])/5000;
ForceTorqSensorLeft.torque.y/=(65535*gainFTLeft[4]*sensitivityFTLeft[4]*ExFTLeft[4])/5000;
ForceTorqSensorLeft.torque.z/=(65535*gainFTLeft[5]*sensitivityFTLeft[5]*ExFTLeft[5])/5000;



//ForceTorqSensorLeft.force.x=incommingPacket->FTsensor[6];
//ForceTorqSensorLeft.force.y=incommingPacket->FTsensor[7];
//ForceTorqSensorLeft.force.z=incommingPacket->FTsensor[8];
//ForceTorqSensorLeft.torque.x=incommingPacket->FTsensor[9];
//ForceTorqSensorLeft.torque.y=incommingPacket->FTsensor[10];
//ForceTorqSensorLeft.torque.z=incommingPacket->FTsensor[11];



        tf2::Quaternion myQuaternion;
myQuaternion.setRPY( incommingPacket->IMU.roll,incommingPacket->IMU.pitch,incommingPacket->IMU.yaw);
//--------------------orientation
IMU.orientation.x=myQuaternion.getX();
IMU.orientation.y=myQuaternion.getY();
IMU.orientation.z=myQuaternion.getZ();
IMU.orientation.w=myQuaternion.getW();
//--------------------lenear acc
IMU.linear_acceleration.x=incommingPacket->IMU.ax;
IMU.linear_acceleration.y=incommingPacket->IMU.ay;
 IMU.linear_acceleration.z=incommingPacket->IMU.az;
 //--------------------angular velocity
 IMU.angular_velocity.x=incommingPacket->IMU.wx;
 IMU.angular_velocity.y=incommingPacket->IMU.wy;
 IMU.angular_velocity.z=incommingPacket->IMU.wz;
 //free acc
 Acceleration.linear.x=incommingPacket->IMU.fax;
 Acceleration.linear.y=incommingPacket->IMU.fay;
 Acceleration.linear.z=incommingPacket->IMU.faz;


 //mag feild
MagneticSensor.magnetic_field.x=incommingPacket->IMU.mx;
MagneticSensor.magnetic_field.y=incommingPacket->IMU.my;
MagneticSensor.magnetic_field.z=incommingPacket->IMU.mz;

//qDebug()<<"IMU New:"<<incommingPacket->IMU.roll<<incommingPacket->IMU.pitch<<incommingPacket->IMU.yaw;

emit FeedBackReceived(ft,positions,positionsInc,bump_sensor_list, imu_data_list);
          //  emit FeedBackReceived(ft,positions,positionsInc);
       }

    // mode 3: CAN Read
    else if ((data[0]== 0x03) && (data[1]== 0x55) && (data[2]== 0xAA) && (data[3]== 0x55) )
    {
        int16_t can_id =((data[4] & 0xFF) << 8) | (data[5] & 0xFF) ;        // can id
        qDebug()<<"Can Read from id"<< can_id << "is : "
                <<QString::number(data[6]&0xff,16)<<QString::number(data[7]&0xff,16)
                <<QString::number(data[8]&0xff,16)<<QString::number(data[9]&0xff,16)
                <<QString::number(data[10]&0xff,16)<<QString::number(data[11]&0xff,16)
                <<QString::number(data[12]&0xff,16)<<QString::number(data[13]&0xff,16);
    }
    else if ((data[0]== 0x08) && (data[1]== 0x55) && (data[2]== 0xAA) && (data[3]== 0x55) )
    {
    //first  16 bytes are can errors and 16 bytes are mcp error 1 means error
    }
        else
    {
        qDebug()<<"epose data Nok:"<<data.toHex();
    }

    tcp.ClearBuffer();
}
//========================================================================
//bool Epos::StartFeedBack()
//{
//   int i=0;
//   WaitMs(700);
//   for(i=0; i< 12; i++) {
//    SetPreoperationalMode(i);
//   }
//   WaitMs(700);
//   for(i=0; i< 12; i++) {
//    StartNode(i);
//   }
//   WaitMs(700);
//    for(i=0; i< 12; i++) {
//        SetMode(i,CSP);
//    }
//    WaitMs(700);
//    for(i=0; i< 12; i++) {
//        SwitchOff(i);
//    }
//    WaitMs(700);

//    return OK;
//}
//========================================================================
bool Epos::StartFeedBack()
{
int i=0;
WaitMs(700);
SetPreoperationalMode(255);
WaitMs(700);
StartNode(255);
WaitMs(700);
SetMode(255,CSP);
WaitMs(700);
SwitchOff(255);
WaitMs(700);
//SwitchOn(255);
return OK;
}
//========================================================================
bool Epos::ActiveAllCSP()
{
int i=0;
WaitMs(700);
SetPreoperationalMode(255);
WaitMs(700);
StartNode(255);
WaitMs(700);
SetMode(255,CSP);
WaitMs(700);
SwitchOff(255);
WaitMs(700);
SwitchOn(255);
return OK;
}
//========================================================================

