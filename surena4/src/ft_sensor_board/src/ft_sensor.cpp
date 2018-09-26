#include "ft_sensor.h"

ft_sensor::ft_sensor(QObject *parent) : QObject(parent)
{

}

bool ft_sensor::Init(int pid, int vid)
{
    if(hid.Connect(pid,vid))
    {
        return true;
    }

    return false;
}
bool ft_sensor::Read(QVector<double> &data)
{
    QByteArray buffer;
    QByteArray inputData;
    buffer.resize(65);
    buffer[62]=5;
    buffer[63]=50;
    hid.Write(buffer);
    hid.Read(inputData);

    double m_dResultChValue2[6];
    double m_dResultChValue1[6];


    m_dResultChValue1[0] = (double)((inputData[0]&0xff) << 8 | (inputData[1]&0xff));
    m_dResultChValue1[1] = (double)((inputData[2]&0xff) << 8 | (inputData[3]&0xff));
    m_dResultChValue1[2] = (double)((inputData[4]&0xff) << 8 | (inputData[5]&0xff));
    m_dResultChValue1[3] = (double)((inputData[6]&0xff) << 8 | (inputData[7]&0xff));
    m_dResultChValue1[4] = (double)((inputData[8]&0xff) << 8 | (inputData[9]&0xff));
    m_dResultChValue1[5] = (double)((inputData[10]&0xff) << 8 | (inputData[11]&0xff));

    m_dResultChValue2[0] = (double)((inputData[12]&0xff) << 8 | (inputData[13]&0xff));
    m_dResultChValue2[1] = (double)((inputData[14]&0xff) << 8 | (inputData[15]&0xff));
    m_dResultChValue2[2] = (double)((inputData[16]&0xff) << 8 | (inputData[17]&0xff));
    m_dResultChValue2[3] = (double)((inputData[18]&0xff) << 8 | (inputData[19]&0xff));
    m_dResultChValue2[4] = (double)((inputData[20]&0xff) << 8 | (inputData[21]&0xff));
    m_dResultChValue2[5] = (double)((inputData[22]&0xff) << 8 | (inputData[23]&0xff));
    double gainFTnow;
     double sensitivityFTnow;
     double ExFTnow;
     double gainFTnow1;
     double sensitivityFTnow1;
     double ExFTnow1;
     double offsetnow;
     double offsetnow1;
data.clear();
       for (int i = 0; i < 6; i++)
       {
           gainFTnow = gainFT[i][0];
           sensitivityFTnow = sensitivityFT[i][0];
           ExFTnow = ExFT[i][0];
           gainFTnow1 = gainFT1[i][0];
           sensitivityFTnow1 = sensitivityFT1[i][0];
           ExFTnow1 = ExFT1[i][0];
           offsetnow = offsetFT[i][0];
           offsetnow1 = offsetFT1[i][0];

           m_dResultChValue1[i] = 5000 * (( m_dResultChValue1[i] - offsetnow) / (((double)65535) * (gainFTnow) * (sensitivityFTnow) * (ExFTnow)));
           m_dResultChValue2[i] = 5000 * ((m_dResultChValue2[i] - offsetnow1) / (((double)65535) * (gainFTnow1) * (sensitivityFTnow1) * (ExFTnow1)));
data.append(m_dResultChValue1[i]);
data.append(m_dResultChValue2[i]);
       }



}
