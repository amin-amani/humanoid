#ifndef USBHID_H
#define USBHID_H

#include <QObject>
#include <QDebug>

#include <QtCore>
#include <QtGui>
#include <QtDebug>
#include <QTimer>
#include "libusbwin/libusb.h"

 #if defined(Q_OS_WIN)
#include <dbt.h>
static const GUID GUID_DEVINTERFACE_LAUNCHPAD =
{ 0xfd96fadb, 0x9246, 0x4017, { 0x8d, 0x76, 0x3e, 0x30, 0x77, 0x80, 0xf6, 0xeb } };
#endif

//
// Hardware specific variables
//


//{{fd96fadb-9246-4017-8d76-3e307780f6eb}}
//
// Those are used to find our specific device on a bus
// VID = TI Stellaris
static const uint16_t VENDOR_ID  = 0xc251;
static const uint16_t PRODUCT_ID = 0x1c02;
class USBHID : public QObject
{
    Q_OBJECT
public:
     libusb_device_handle* _devh;
    int  _maxPacketSize;
    explicit USBHID(QObject *parent = 0);
    ///
    /// \brief Connect to usb hid device
    /// \param VendorID : usb hid vendor ID
    /// \param ProductID : usb hid product ID
    /// \return success
    ///
    bool Connect(int VendorID, int ProductID);
     bool Write(QByteArray data);
     bool Read(QByteArray &data);
signals:

public slots:
};

#endif // USBHID_H
