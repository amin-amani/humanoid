#ifndef EPOSE_H
#define EPOSE_H

#include <QObject>
//#include <QtTest/QSignalSpy>
#include <QMap>
#include "can.h"

enum EPOSErrorsEnum { OK=0,USB_ERROR, BOARD_ERROR, CAN_ERROR,SDO_REJECT,SDO_BAD_REPLAY,NO_ANSER};

enum EPOSOperationMode {PPM=1,PVM=3,HMM=6,CSP=8,CST=10};

class Epose : public QObject
{
    Q_OBJECT


public:
EPOSErrorsEnum EPOSErrors;
    Can can;
    QMap< int,QString> ErrorCodes;


    // Q_ENUM(EPOSErrors)

    explicit Epose(QObject *parent = 0);
     #if defined(Q_OS_WIN)
    ///=======================================================================================
    /// \brief Init
    /// \param handle
    /// \return
    ///=======================================================================================

    EPOSErrors Init(HANDLE handle);
#endif
    ///=======================================================================================
    /// \brief EnableDevice
    /// \param devID
    /// \param mode
    /// \return
    ///=======================================================================================
    bool EnableDevice(int devID,EPOSOperationMode mode);
    ///=======================================================================================
    /// \brief SetPreoperationalMode
    /// \param devID
    ///=======================================================================================
    void SetPreoperationalMode(int devID);
    ///=======================================================================================
    /// \brief ResetComunication
    /// \param devID
    ///=======================================================================================
    void ResetComunication(int devID);
    ///=======================================================================================
    /// \brief StartNode
    /// \param devID
    ///=======================================================================================
    void StartNode(int devID);
    ///=======================================================================================
    /// \brief SetMode
    /// \param devID
    /// \param mode
    ///=======================================================================================
    void SetMode(int devID, int mode);
    ///=======================================================================================
    /// \brief StopNode
    /// \param devID
    ///=======================================================================================
    void StopNode(int devID);
    ///=======================================================================================
    /// \brief SwitchOn
    /// \param devID
    ///=======================================================================================
    void SwitchOn(int devID);
    ///=======================================================================================
    /// \brief SwitchOff
    /// \param devID
    ///=======================================================================================
    void SwitchOff(int devID);
    ///=======================================================================================
    /// \brief ResetNode
    /// \param devID 0~11 -> 255 broadcast
    /// \return
    ///=======================================================================================
    bool ResetNode(int devID);
    ///=======================================================================================
    /// \brief GetSDOCODE
    /// \param len
    /// \return
    ///=======================================================================================
    unsigned char GetSDOCODE(int len);
    ///=======================================================================================
    /// \brief SDOWriteCommand
    /// \param id
    /// \param input
    /// \param index
    /// \param sub_index
    /// \param len
    /// \param devID
    /// \return
    ///=======================================================================================
    bool SDOWriteCommand(int id, unsigned long input, int index, unsigned char sub_index, unsigned char len, unsigned char devID);
    ///=======================================================================================
    /// \brief SDOReadCommand
    /// \param id
    /// \param index
    /// \param subIndex
    /// \param devID
    ///=======================================================================================
    bool SDOReadCommand(int id, int index, unsigned char subIndex, unsigned char devID);
    ///=======================================================================================
    /// \brief ReadRegister
    /// \param index
    /// \param subIndex
    /// \param canID
    /// \param devID
    /// \param value
    /// \return
    ///=======================================================================================
    bool ReadRegister(int index, int subIndex, int canID, int devID, int32_t &value);
    ///=======================================================================================
    /// \brief WriteRegister
    /// \param index
    /// \param subIndex
    /// \param canID
    /// \param devID
    /// \param value
    /// \param len
    /// \return
    ///=======================================================================================
    bool WriteRegister(int index, int subIndex, int canID, int devID, int32_t value, int len=4);
    void InitErrorMap();
    bool ActivePPM(int canID, int devId);
    bool SetPosition(int canID, int devId, int position, int velocity);
    QString ReadCurrentError(int canID, int devID);
    bool ActiveCSP(int nodeID);
    bool SetPositionIPM(QList<int> positionList);
    bool Init();
    bool GetIncPosition(int nodeID, int &value);
    bool GetAbsPosition(int nodeID, int &value);
    bool GetCurrentActualValue(int nodeID, int &value);
    void WaitMs(int ms);


    bool SetTXPDO(int dev, bool value);
signals:
void Dummy();
public slots:
};

#endif // EPOSE_H
