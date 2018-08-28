#include "usbhid.h"

//====================================================================================

USBHID::USBHID(QObject *parent) : QObject(parent)
{
   _maxPacketSize=64;
}

//====================================================================================

bool USBHID::Connect(int VendorID,int ProductID)
{
    int rc;


        /* Initialize libusb
         */
        rc = libusb_init(NULL);
        if (rc < 0) {
            qDebug()<< "Error initializing libusb:"<< libusb_error_name(rc);
           return false;
    }

        /* Set debugging output to max level.
           */
        //  libusb_set_debug(NULL, 3);

          /* Look for a specific device and open it.
           */
          _devh = libusb_open_device_with_vid_pid(NULL, VendorID, ProductID);
          if (!_devh) {
              fprintf(stderr, "Error finding USB device\n");
            return false;
          }
          libusb_detach_kernel_driver(_devh, 0);
          libusb_claim_interface(_devh, 0);
          return true;
   return true;
}

//====================================================================================

bool USBHID::Write(QByteArray data)
{
    int rc=0;

    if (!_devh) return false;

    int packetSize=data.count();
if(packetSize>_maxPacketSize)packetSize=_maxPacketSize;

    rc = libusb_control_transfer(_devh,0x21,0x9,0x200,0,(unsigned char*)data.data(),packetSize,0);
    if (rc < 0) {
        fprintf(stderr, "Error during write control transfer: %s\n",
                libusb_error_name(rc));
        return false;
    }


return true;

}

//==============================================================================================================

bool USBHID::Read(QByteArray &data)
{
       // QByteArray data;
            unsigned char buff[_maxPacketSize];
        int rc;
        data.clear();
    if (!_devh) return false;

 rc = libusb_control_transfer(_devh,0xa1,0x1,0x100,0,buff,_maxPacketSize,0);
  if (rc < 64) {
      fprintf(stderr, "Error during read control transfer: %s\n",
              libusb_error_name(rc));
   return false;
  }
      data.append((const char*)buff,_maxPacketSize);
  return true;


}

//====================================================================================
