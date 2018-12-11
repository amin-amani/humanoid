#ifndef EPOS_H
#define EPOS_H

#include <QObject>
#include <QDebug>
#include <qthread.h>
#include <qlist.h>
#include "can.h"
#include "pingmodel.h"
//#include "tcphandler.h"
#define USE_NET

enum EPOSErrors { OK=0,USB_ERROR, BOARD_ERROR, CAN_ERROR,SDO_REJECT,SDO_BAD_REPLAY,NO_ANSER,NETWOR_ERROR};
enum EPOSOperationMode {PPM=1,PVM=3,HMM=6,CSP=8,CST=10};

class Epos : public QObject
{
    Q_OBJECT
    TcpHandler tcp;

public:
	Can can;
    explicit Epos(QObject *parent = 0);

    EPOSErrors Init();
    EPOSErrors EnableDevice(int devID,EPOSOperationMode mode);
    void SetPreoperationalMode(int devID);
    void ResetComunication(int devID);
    void StartNode(int devID);
    void SetMode(int devID, EPOSOperationMode mode);
    void StopNode(int devID);
    void SwitchOn(int devID);
    void SwitchOff(int devID);
    EPOSErrors ResetNode(int devID);
    unsigned char GetSDOCODE(int len);
    EPOSErrors SDOWriteCommand(int id, unsigned long input, int index, unsigned char sub_index, unsigned char len, char devID);
    void SDOReadCommand(int id, int index, unsigned char subIndex, char devID);
    EPOSErrors ReadRegister(int index, int subIndex, int canID, int devID, int32_t &value);
    EPOSErrors WriteRegister(int index, int subIndex, int canID, int devID, int32_t value, int len=4);

    EPOSErrors ActivePPM(int canID, int devId);
    bool SetPosition(int canID, int devId, int position, int velocity);
    QString ReadCurrentError(int canID, int devID);
	
    EPOSErrors test_arya(int devID) ;

    bool ActiveCSP(int nodeID);
    bool AllActiveCSP();

    // bool ResetAllNodes();
    // void SetPreoperationalMode();
    // void StartNode();
    // void SetMode(unsigned char mode);
    // void SwitchOff();
    // void SwitchOn();
    // void ActiveCST();
    // void ActivePPM();
    // void SetMode(ModesOfOperations mode);
    // void MoveToTarget();
    // void MoveToTargetPDO();
    // void SetPositionSDO(long position, long velocity);
    // void DisconnectTCP();
        void SetPositionCST(int position, int velocity);
        void SetAllPositionCST(QList<int> all_position, int motor_num);
        void can_write(unsigned char motor_index,unsigned int can_id, unsigned char* data_buff);
        void can_read_test(unsigned char motor_index);

        void WaitMs(int ms);
        QByteArray EposReceiveData();

        bool StartFeedBack();
        bool ActiveAllCSP();
signals:

    void NewDataReady();
    void Dummy();
    void FeedBackReceived(QList<int16_t>ft,QList<int32_t>positionAbs,QList<int32_t>positionInc);

public slots:


    void DataReceived(QByteArray data);

};



#endif // EPOS_H
