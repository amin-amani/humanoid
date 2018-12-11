#include "epos.h"
//========================================================================
Epos::Epos(QObject *parent) : QObject(parent)
{
//    while (!tcp.IsConnected) {
//    }
    qDebug()<<"all printed values are hex";
   // connect(&tcp,SIGNAL(NewDataReceived()),this,SIGNAL(NewDataReady()));
    connect(&tcp,SIGNAL(NewDataReceived(QByteArray)),this,SLOT(DataReceived(QByteArray)));

}
//========================================================================
EPOSErrors Epos::Init()
{
    PingModel ping;

    if(! ping.Start("192.168.1.10"))
        return NETWOR_ERROR;
    if(can.Init())
    {
        return OK;
    }
    return CAN_ERROR;
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
void Epos::SetPreoperationalMode(int devID)
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
void Epos::SetMode(int devID,EPOSOperationMode mode)
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

    can.WriteMessage(0x301,devID,data);
}
//========================================================================
void Epos::SwitchOn(int devID)
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

    can.WriteMessage(0x201,devID,data);
}
//========================================================================
void Epos::SwitchOff(int devID)
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

    can.WriteMessage(0x201,devID,data);
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
EPOSErrors Epos::ReadRegister(int index,int subIndex,int canID, int devID,int32_t &value)
{
    uint16_t inpid;
    QByteArray replay;

    can.ReadMessage(devID, replay);
    can.ReadMessage(devID, replay);
    SDOReadCommand(canID+0x600,index,subIndex,devID);
    QThread::msleep(5);

    can.ReadMessage(devID,replay);

    inpid=replay[0]&0xff;
    inpid<<=8;
    inpid+=replay[1]&0xff;

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
    status= ReadRegister(0x603f,0,canID,devID,value);
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
void Epos::SetAllPositionCST(QList<int> all_position, int motor_num)
{
    int i=0;
    QByteArray data;
    data.append(0x01);  // mode 1: Run
    data.append(0xAA);  // Header check
    data.append(0x55);  // Header check
    data.append(0xAA);  // Header check

    for(i=0; i< motor_num; i++)  // send 12 data motor (48 byte)
    {
        data.append(0x04); // can id high byte   0401
        data.append(0x01); // can id low byte    0401

        data.append((all_position[i] >> 0) & 0xff);
        data.append((all_position[i] >> 8) & 0xff);
        data.append((all_position[i] >> 16) & 0xff);
        data.append((all_position[i] >> 24) & 0xff);
     }

    for(i=0; i< 4; i++)  // send 4 data can(each can 8 byte) (40 byte)
    {
        data.append(0x04); // can id high byte   0401
        data.append(0x01); // can id low byte    0401

        data.append(0x01);
        data.append(0x01);
        data.append(0x01);
        data.append(0x01);
        data.append(0x01);
        data.append(0x01);
        data.append(0x01);
        data.append(0x01);
     }

    for(i = (6*motor_num)+44 ; i < 252; i++)
    {
        data.append(0x01);  // fill data packet with 01 byte
    }
    for(int i=data.count();i<300-4;i++)
        data.append(1);
    data.append(0xFF);  // Parity | 0xFF
    data.append(0xCC);  // Tailer
    data.append(0x33);  // Tailer
    data.append(0xCC);  // Tailer

//QByteArray reponse=tcp.SendCommand(data);
//qDebug()<<"this is len:"<<data.length();
    tcp.WriteData(data);  // send 128 Byte Packet

    // just for test
    // qDebug() << "send data: " << data.toHex();
}
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
void Epos::DataReceived(QByteArray data)
{

    QList<int32_t> positions;
    QList<int32_t> positionsInc;
    QList<int16_t> ft;
    int id_can_read[16];
        int32_t pos_inc_value=0;
    int32_t pos_value=0;
    int can_id =0;
//qDebug()<<"tick tick";

    if ( (data[0]&0xff)== 0x01 && (data[1]&0xff)== 0x55 && (data[2]&0xff)== 0xAA && (data[3]&0xff)== 0x55 )
    {

       // qDebug()<<"epose data  OK: ";
        //data.remove(0,4);
        for (int i= 0; i< 16; i++)
        {

    can_id = (data[(10*i)+5] & 0xFF) ;        // can id low byte
    can_id += (data[(10*i)+4] & 0xFF) << 8;   // can id high byte

    pos_value= (data[(10*i)+13] & 0xFF);
    pos_value<<=8;
    pos_value|= (data[(10*i)+12] & 0xFF);
    pos_value<<=8;
    pos_value|= (data[(10*i)+11] & 0xFF);
    pos_value<<=8;
    pos_value|= (data[(10*i)+10] & 0xFF);

    //////
    pos_inc_value= (data[(10*i)+9] & 0xFF);
    pos_inc_value<<=8;
    pos_inc_value|= (data[(10*i)+8] & 0xFF);
    pos_inc_value<<=8;
    pos_inc_value|= (data[(10*i)+7] & 0xFF);
    pos_inc_value<<=8;
    pos_inc_value|= (data[(10*i)+6] & 0xFF);
    ///////////


    positions.append(pos_value);
    positionsInc.append(pos_inc_value);
    id_can_read[i] = can_id;
        }
       //qDebug()<<data_can_read[0]<<" "<<data_can_read[1]<<" "<<data_can_read[2]<<" "<<data_can_read[3]<<" "<<data_can_read[4]<<" "<<data_can_read[5]<<" "<<data_can_read[6]<<" "<<data_can_read[7]<<" "<<data_can_read[8]<<" "<<data_can_read[9]<<" "<<data_can_read[10]<<" "<<data_can_read[11]<<" "<<data_can_read[12]<<" "<<data_can_read[13]<<" "<<data_can_read[14]<<" "<<data_can_read[15];
       //qDebug()<<id_can_read[0]<<":"<<data_can_read[0]<<" "<<id_can_read[1]<<":"<<data_can_read[1]<<" "<<id_can_read[2]<<":"<<data_can_read[2]<<" "<<id_can_read[3]<<":"<<data_can_read[3]<<" "<<id_can_read[4]<<":"<<data_can_read[4]<<" "<<id_can_read[5]<<":"<<data_can_read[5]<<" "<<id_can_read[6]<<":"<<data_can_read[6]<<" "<<id_can_read[7]<<":"<<data_can_read[7]<<" "<<id_can_read[8]<<":"<<data_can_read[8]<<" "<<id_can_read[9]<<":"<<data_can_read[9]<<" "<<id_can_read[10]<<":"<<data_can_read[10]<<" "<<id_can_read[11]<<":"<<data_can_read[11]<<" "<<id_can_read[12]<<":"<<data_can_read[12]<<" "<<id_can_read[13]<<":"<<data_can_read[13]<<" "<<id_can_read[14]<<":"<<data_can_read[14]<<" "<<id_can_read[15]<<":"<<data_can_read[15];
       //qDebug()<<QString::number(data_can_read[2]);
        for (int j = 0; j < 6; j++)
        {
            int16_t tempFT=(((data[(2*j)+164] & 0xFF))<<8 ) | (data[(2*j)+165] & 0xFF);
            ft.append(tempFT);
            tempFT=(((data[(2*j)+176] & 0xFF))<<8 ) | (data[(2*j)+177] & 0xFF);
            ft.append(tempFT);
//           ft_sensor1[j] = (((data[(2*j)+163] & 0xFF))<<8 ) | (data[(2*j)+164] & 0xFF);
//           ft_sensor2[j] = (((data[(2*j)+175] & 0xFF))<<8 ) | (data[(2*j)+176] & 0xFF);
//           ft.append(ft_sensor1[j]);
//           ft.append(ft_sensor2[j]);
       }
        //if(id_can_read[2]==0x181){
     //  qDebug()<<"data is 0x181"<<data.mid(24   ,10).toHex();
            emit FeedBackReceived(ft,positions,positionsInc);
      //  }
       //qDebug()<<"Sensor val="<<QString::number(data[163]&0xff,16)<<QString::number(data[164]&0xff,16);
       //qDebug()<<" ft_sensor1: " << ft_sensor1[0] << ", " << ft_sensor1[1] << ", " << ft_sensor1[2] << ", " << ft_sensor1[3] << ", " << ft_sensor1[4] << ", " << ft_sensor1[5] << ";" ;
       //qDebug()<<" ft_sensor2: " << ft_sensor2[0] << ", " << ft_sensor2[1] << ", " << ft_sensor2[2] << ", " << ft_sensor2[3] << ", " << ft_sensor2[4] << ", " << ft_sensor2[5] << ";" ;
    }

    // mode 3: CAN Read
    else if ((data[0]== 0x03) && (data[1]== 0x55) && (data[2]== 0xAA) && (data[3]== 0x55) )
    {
        can_id =((data[4] & 0xFF) << 8) | (data[5] & 0xFF) ;        // can id
        qDebug()<<"Can Read from id"<< can_id << "is : "
                <<QString::number(data[6]&0xff,16)<<QString::number(data[7]&0xff,16)
                <<QString::number(data[8]&0xff,16)<<QString::number(data[9]&0xff,16)
                <<QString::number(data[10]&0xff,16)<<QString::number(data[11]&0xff,16)
                <<QString::number(data[12]&0xff,16)<<QString::number(data[13]&0xff,16);
    }

    else
    {
        qDebug()<<"epose data Nok:"<<data.toHex();
    }

    tcp.ClearBuffer();
}
//========================================================================
bool Epos::StartFeedBack()
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

    return OK;
}
//========================================================================
bool Epos::ActiveAllCSP()
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
    return OK;
}
//========================================================================
