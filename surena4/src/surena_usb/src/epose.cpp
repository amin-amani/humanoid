#include "epose.h"
//===============================================================================================================

Epose::Epose(QObject *parent) : QObject(parent)
{
  InitErrorMap();

}

//===============================================================================================================

bool Epose::Init()
{
  if(can.Init(0xc251,0x1c03))
  {

    return true;

  }

  return false;
}
bool Epose::SetTXPDO(int dev,bool value)
{
  int val=0;
  WriteRegister(0x1a00,0,1,dev,0);
  WaitMs(10);
  WriteRegister(0x1a00,1,1,dev,0);
  WaitMs(10);
  WriteRegister(0x1a00,2,1,dev,0);
  WaitMs(10);
  return true;
  WriteRegister(0x1a00,0,1,dev,0x2);
  WaitMs(10);
  if(value){
    qDebug()<<"position inc - abs feedback";
    WriteRegister(0x1a00,1,1,dev,0x60e40120);
    WaitMs(10);
    WriteRegister(0x1a00,2,1,dev,0x60e40220);
    ReadRegister(0x1a00,1,1,dev,val);
    qDebug()<<QString::number( val,16);
    ReadRegister(0x1a00,2,1,dev,val);
    qDebug()<<QString::number( val,16);
    return true;
  }
  qDebug()<<"current - follow";

  //current - follow
  WriteRegister(0x1a00,1,1,dev,0x30d10220);
  WaitMs(10);
  WriteRegister(0x1a00,2,1,dev,0x60f40020);
  ReadRegister(0x1a00,1,1,dev,val);
  qDebug()<<QString::number( val,16);
  ReadRegister(0x1a00,2,1,dev,val);
  qDebug()<<QString::number( val,16);
  return true;
}
//====================================================================================

bool Epose::ResetNode(int devID)
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
  return true;
}

//===============================================================================================================

bool Epose::EnableDevice(int devID,EPOSOperationMode mode)
{

  WaitMs(700);

  SetPreoperationalMode(devID);

  WaitMs(700);

  StartNode(devID);

  WaitMs(700);

  SetMode(devID,mode);

  WaitMs(700);

  SwitchOff(devID);

  WaitMs(700);

  SwitchOn(devID);

  WaitMs(700);
  return true;
}

//===============================================================================================================

void Epose::SetPreoperationalMode(int devID)
{
  QByteArray data;
  data.append(0x80);
  data.append((char)0x00);
  data.append((char)0x00);
  data.append((char)0x00);
  data.append((char)0x00);
  data.append((char)0x00);
  data.append((char)0x00);
  data.append((char)0x00);


  can.WriteMessage(00,devID,data);
}

//===============================================================================================================

void  Epose::ResetComunication(int devID)
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

//====================================================================================

void Epose::StartNode(int devID)
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

//====================================================================================

void Epose::StopNode(int devID)
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

//====================================================================================

void Epose::SetMode(int devID,int mode)
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

//===============================================================================================================

void Epose::SwitchOn(int devID)
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

//====================================================================================

void Epose::SwitchOff(int devID)
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

//====================================================================================

bool Epose::ActiveCSP(int nodeID)
{
  WaitMs(700);
  SetPreoperationalMode(nodeID);
  WaitMs(700);
  StartNode(nodeID);
  WaitMs(700);
  SetMode(nodeID,8);
  WaitMs(700);
  SwitchOff(nodeID);
  WaitMs(700);
  SwitchOn(nodeID);
  WaitMs(700);
  return true;
}

//====================================================================================

unsigned char Epose::GetSDOCODE(int len)
{
  if (len == 1) { return 0x2f; }
  if (len == 2) { return 0x2b; }
  if (len == 4) { return 0x23; }
  if (len == -1) { return 0x22; }
  if (len == 0) { return 0x40; }
  return 0;
}

//====================================================================================

bool Epose::SDOWriteCommand( int id,unsigned long int input,int index,unsigned char subIndex,unsigned char len,unsigned char devID)
{
  //epos 2 application note collection page 155
  //unsigned char replay[64];
  QByteArray replay;
  int16_t inp_index=0;//,inpid,;
  //uint8_t inp_len;
  //int val=0;
  QByteArray data;
  data.append (GetSDOCODE(len));
  data.append((unsigned char)(index & 0xff));
  data.append((index >> 8)&0xff);
  data.append(subIndex&0xff);
  data.append((char)input & 0xff);
  data.append((char)((input >> 8) & 0xff));
  data.append((char)(input >> 16) & 0xff);
  data.append((char)(input >> 24) & 0xff);



  if(!can.WriteMessage(id+0x600, devID,data))
  { qDebug()<<"sdo r1"; return false;}
  WaitMs(5);
  if( !can.ReadMessage(devID,replay))
  {qDebug()<<"sdo r2"; return false;}

  inp_index=(replay[8]&0xff);
  inp_index<<=8;
  inp_index+=(replay[9]&0xff);
  //qDebug()<<"dev:"<<QString::number( devID);
  // qDebug()<<"rep 11"<<QString::number( replay[11],16);

  // qDebug()<<"replay="<<QByteArray::fromRawData((const char *)replay,12).toHex();
  if(devID==255)return true;
  if(replay[11]==(char)0){
    qDebug()<<"sdo r3";
    return false;

  }
  // qDebug()<<"sdo r4";
  return true;


}

//====================================================================================

bool Epose::SDOReadCommand( int id,int index,unsigned char subIndex,unsigned char devID)
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
  return can.WriteMessage(id,devID,data);
}

//====================================================================================

bool Epose::ReadRegister(int index,int subIndex,int canID, int devID,int32_t&value)
{
  uint16_t inpid;
  bool result=false;
  QByteArray replay;
  can.ReadMessage(devID,replay);
  can.ReadMessage(devID,replay);
  result=SDOReadCommand(canID+0x600,index,subIndex,devID);
  if(!result){
    return false;
  }
  WaitMs(5);
  result=can.ReadMessage(devID,replay);
  if(!result){
    return false;
  }
  inpid=replay[8]&0xff;
  inpid<<=8;
  inpid+=replay[9]&0xff;

  if(replay[11]==(char)0)//inputlen
    return false;

  if(replay[10]==(char)0)//inputlen
    return false;
  //  qDebug()<<QString::number(inpid,16);// inpid=replay
  //qDebug()<<QString::number(canID+0x580,16);
  if((inpid==0x580+canID))
  {
    // PAGE 150 APPLICATION NOTE COLLECTIOIN
    value=0;
    value=replay[7]&0xff;
    value=value<<8;
    value+=replay[6]&0xff;
    value=(value)<<8;
    value+=replay[5]&0xff;
    value=(value)<<8;
    value+=replay[4]&0xff;
    // qDebug()<<"inja";
    return true;

  }
  return false;
}

//====================================================================================

bool Epose::WriteRegister(int index,int subIndex,int canID, int devID,int32_t value,int len)
{

  QByteArray replay;
  can.ReadMessage(devID,replay);
  WaitMs(5);
  can.ReadMessage(devID,replay);
  WaitMs(5);
  return SDOWriteCommand(canID,value,index,subIndex,len,devID);

}

void Epose::InitErrorMap()
{
  ErrorCodes.insert(  0x0,"OK");
  ErrorCodes.insert(  0x1000,"Generic error");
  ErrorCodes.insert(  0x1080,"Generic initialization error");
  ErrorCodes.insert(  0x1081,"Generic initialization error");
  ErrorCodes.insert(  0x1082,"Generic initialization error");
  ErrorCodes.insert(  0x1083,"Generic initialization error");
  ErrorCodes.insert(0x1090,"Firmware incompatibility error");
  ErrorCodes.insert(0x2310,"Overcurrent error");
  ErrorCodes.insert(0x2320,"Power stage protection error");
  ErrorCodes.insert(0x3210,"Overvoltage error");
  ErrorCodes.insert(0x3220,"Undervoltage error");
  ErrorCodes.insert(0x4210,"Thermal overload error");
  ErrorCodes.insert(0x5113,"Logic supply voltage too low error");
  ErrorCodes.insert(0x5280,"Hardware defect error");
  ErrorCodes.insert(0x5281,"Hardware incompatibility error");
  ErrorCodes.insert(0x5480,"Hardware error");
  ErrorCodes.insert(0x5481,"Hardware error");
  ErrorCodes.insert(0x5482,"Hardware error");
  ErrorCodes.insert(0x5483,"Hardware error");
  ErrorCodes.insert(0x6080,"Sign of life error");
  ErrorCodes.insert(0x6081,"Extension 1 watchdog error");
  ErrorCodes.insert(0x6320,"Software parameter error");
  ErrorCodes.insert(0x7320,"Position sensor error");
  ErrorCodes.insert(0x7380,"Position sensor breach error");
  ErrorCodes.insert(0x7381,"Position sensor resolution error");
  ErrorCodes.insert(0x7382,"Position sensor index error");
  ErrorCodes.insert(0x7388,"Hall sensor error");
  ErrorCodes.insert(0x7389,"Hall sensor not found error");
  ErrorCodes.insert(0x738A,"Hall angle detection error");
  ErrorCodes.insert(0x738C,"SSI sensor error");
  ErrorCodes.insert(0x7390,"Missing main sensor error");
  ErrorCodes.insert(0x7391,"Missing commutation sensor error");
  ErrorCodes.insert(0x7392,"Main sensor direction error");
  ErrorCodes.insert(0x8110,"CAN overrun error (object lost)");
  ErrorCodes.insert(0x8111,"CAN overrun error");
  ErrorCodes.insert(0x8120,"CAN passive mode error");
  ErrorCodes.insert(0x8130,"CAN heartbeat error");
  ErrorCodes.insert(0x8150,"CAN PDO COB-ID collision");
  ErrorCodes.insert(0x8180,"EtherCAT communication error");
  ErrorCodes.insert(0x8181,"EtherCAT initialization error");
  ErrorCodes.insert(0x81FD,"CAN bus turned off");
  ErrorCodes.insert(0x81FE,"CAN Rx queue overflow");
  ErrorCodes.insert(0x81FF,"CAN Tx queue overflow");
  ErrorCodes.insert(0x8210,"CAN PDO length error");
  ErrorCodes.insert(0x8250,"RPDO timeout");
  ErrorCodes.insert(0x8280,"EtherCAT PDO communication error");
  ErrorCodes.insert(0x8281,"EtherCAT SDO communication error");
  ErrorCodes.insert(0x8611,"Following error");
  ErrorCodes.insert(0x8A80,"Negative limit switch error");
  ErrorCodes.insert(0x8A81,"Positive limit switch error");
  ErrorCodes.insert(0x8A82,"Software position limit error");
  ErrorCodes.insert(0x8A88,"STO error");
  ErrorCodes.insert(0xFF01,"System overloaded error");
  ErrorCodes.insert(0xFF02,"Watchdog error");
  ErrorCodes.insert(0xFF0B,"System peak overloaded error");
  ErrorCodes.insert(0xFF10,"Controller gain error");
  ErrorCodes.insert(0xFF12,"Auto tuning current limit error");
  ErrorCodes.insert(0xFF13,"Auto tuning identification current error");
  ErrorCodes.insert(0xFF14,"Auto tuning buffer overflow error");
  ErrorCodes.insert(0xFF15,"Auto tuning sample mismatch error");
  ErrorCodes.insert(0xFF16,"Auto tuning parameter error");
  ErrorCodes.insert(0xFF17,"Auto tuning amplitude mismatch error");
  ErrorCodes.insert(0xFF18,"Auto tuning period length error");
  ErrorCodes.insert(0xFF19,"Auto tuning timeout error");
  ErrorCodes.insert(0xFF20,"Auto tuning standstill error");
  ErrorCodes.insert(0xFF21,"Auto tuning torque invalid error");
}
//====================================================================================

void Epose::WaitMs(int ms)
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
//====================================================================================

bool Epose::ActivePPM(int canID,int devId){
  int32_t value;
  bool result=false;
  //qDebug()<<"pvt";
  //return WriteRegister(0x607A,0,canID,devId,4);
  result=WriteRegister(0x6060,0,canID,devId,1,1);//set mode
  if(!result){
    qDebug()<<"wr error";
    if(!WriteRegister(0x6060,0,canID,devId,1,1))//set mode
      return false;
  }
  WaitMs(1000);
  result=WriteRegister(0x6040,0,canID,devId,0x06,2);//set mode
  if(!result)return false;
  WaitMs(1000);
  result=WriteRegister(0x6040,0,canID,devId,0x0f,2);//set mode
  if(!result)return false;
  WaitMs(1000);
  result=WriteRegister(0x6040,0,canID,devId,0x010f,2);//set mode
  if(!result)return false;
  WaitMs(1000);
  result=WriteRegister(0x6065,0,canID,devId,10000,4);//set max follow error
  if(!result)return false;
  WaitMs(1000);
  if(devId==255){
    for(int i=0;i<12;i++){
      value=0;
      WaitMs(20);
      result=ReadRegister(0x6060,0,canID,i,value);//set mode
      if(!result)return false;
      if(value!=1){
        qDebug()<<i<<" m="<<QString::number(value,16);
        return false;}
      result=ReadRegister(0x6041,0,canID,i,value);//set mode
      if(!result)return false;

      if(value!=0x437)
      {
        qDebug()<<i<<" s="<<QString::number(value,16);
        return false;}
      WaitMs(10);
      result=ReadRegister(0x6065,0,canID,i,value);//set mode
      if(!result){qDebug()<<"reaf follw error";return false;}
      qDebug()<<"max follw="<<value;

    }
  }
  else{

    value=0;
    result=ReadRegister(0x6060,0,canID,devId,value);//set mode
    if(!result)return false;
    if(value!=1){
      return false;
      qDebug()<<"mode="<<QString::number(value,16);
    }
    result=ReadRegister(0x6041,0,canID,devId,value);//set mode
    if(!result)return false;
    {
      qDebug()<<"s="<<QString::number(value,16);
      if(value!=0x437)
      {
        return false;

      }
    }
    result=ReadRegister(0x6065,0,canID,devId,value);//set mode
    if(!result)return false;
    qDebug()<<"max follw="<<value;
  }
  return true;
}

//====================================================================================

QString Epose::ReadCurrentError(int canID,int devID)
{bool status;
  int value=0;
  // status= ReadRegister(0x1001,0,canID,devID,value);
  status= ReadRegister(0x603f,0,canID,devID,value);
  if(status!=true){ return "read error";}
  QString result= QString::number( value,16)+"->"+ErrorCodes[value];
  return result;


}

//====================================================================================

bool Epose::SetPosition(int canID,int devId,int position,int velocity)
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

//====================================================================================

bool Epose::SetPositionIPM(QList<int>positionList)
{
  can.SetPosition(positionList);
  return true;

}

//====================================================================================

bool Epose::GetAbsPosition(int nodeID,int &value)
{

  return ReadRegister(0x60e4,2,1,nodeID,value);//read abs enc


}

//====================================================================================

bool Epose::GetCurrentActualValue(int nodeID,int &value)
{

  return ReadRegister(0x30d0,0,1,nodeID,value);//read abs enc


}

//====================================================================================

bool Epose::GetIncPosition(int nodeID,int &value)
{

  return ReadRegister(0x60e4,1,1,nodeID,value);//)//read inc enc


}

//====================================================================================
