/******************************************/
/*
  record.cpp
  by Gary P. Scavone, 2007

  This program records audio from a device and writes it to a
  header-less binary file.  Use the 'playraw', with the same
  parameters and format settings, to playback the audio.
*/
/******************************************/
#include <qapplication.h>
#include "ros/ros.h"
#include "roham-asr.h"
#include "RtAudio.h"
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <stdio.h>
#include <dlfcn.h>
#include <QDebug>
#include <std_msgs/String.h>
#include <string>
typedef signed short MY_TYPE;
#define FORMAT RTAUDIO_SINT16
#include <unistd.h>
#define SLEEP( milliseconds ) usleep( (unsigned long) (milliseconds * 1000.0) )

struct InputData {
    MY_TYPE* buffer;
    unsigned long bufferBytes;
    unsigned long totalFrames;
    unsigned long frameCounter;
    unsigned int channels;
};


RtAudio adc;
InputData data;
ros::Publisher SpeechPublisher;
//================================================================================
void run_command(char* command) {
    ROS_INFO ("run_command: %s" , command );//<<std::endl;
    std::cout << "run_command: " << command <<std::endl;
    qDebug()<< "run_command: " << command ;
    std_msgs::String message;
    std::stringstream ss;
    ss<<"hi amin";
    message.data=command;

    SpeechPublisher.publish(message);
   // sleep(7);

}
//================================================================================
void CleanUp()
{
    if ( adc.isStreamOpen() ) adc.closeStream();
    if ( data.buffer ) free( data.buffer );
}
//================================================================================
int input( void * /*outputBuffer*/, void *inputBuffer, unsigned int nBufferFrames,
           double /*streamTime*/, RtAudioStreamStatus /*status*/, void *data )
{

    MY_TYPE *samples = new  MY_TYPE[nBufferFrames];
    MY_TYPE *samples0 = (MY_TYPE *)inputBuffer;
    memcpy(samples, samples0, nBufferFrames * sizeof( MY_TYPE ) );
    vad_collector(samples, run_command);
    return 0;

}
//================================================================================
void SpeechInit()
{
    char* dir = (char*)"sorena";
    char*  status = init("/home/cast/humanoid/surena4/src/speech/src/sorena");
    ROS_INFO( "Init status: %s" , status);
    if ( adc.getDeviceCount() < 1 ) {
        ROS_FATAL( "No audio devices found!");
        exit( 1 );
    }
    unsigned int channels=1, fs=16000, bufferFrames, device = 0, offset = 0;
    double time = 60.0;

    adc.showWarnings( true );
    bufferFrames = 1600;
    RtAudio::StreamParameters iParams;
    if ( device == 0 )
        iParams.deviceId = adc.getDefaultInputDevice();
    else
        iParams.deviceId = device;
    iParams.nChannels = channels;
    iParams.firstChannel = offset;
    data.buffer = 0;
    try {
        adc.openStream( NULL, &iParams, FORMAT, fs, &bufferFrames, &input, (void *)&data );
    }
    catch ( RtAudioError& e ) {
        std::cout << '\n' << e.getMessage() << '\n' << std::endl;
        CleanUp();
        exit(1);
    }

    data.bufferBytes = bufferFrames * channels * sizeof( MY_TYPE );
    data.totalFrames = (unsigned long) (fs * time);
    data.frameCounter = 0;
    data.channels = channels;
    unsigned long totalBytes;
    totalBytes = data.totalFrames * channels * sizeof( MY_TYPE );

    data.buffer = (MY_TYPE*) malloc( totalBytes );
    if ( data.buffer == 0 ) {
        std::cout << "Memory allocation error ... quitting!\n";
        CleanUp();
        exit(1);
    }

    try {
        adc.startStream();
    }
    catch ( RtAudioError& e ) {
        std::cout << '\n' << e.getMessage() << '\n' << std::endl;
        CleanUp();
        exit(1);
    }
}
//================================================================================
int main(int argc, char **argv)
{
    QApplication a(argc, argv);
    ros::init(argc, argv, "speech");
    ros::NodeHandle nh;
    SpeechPublisher=nh.advertise<std_msgs::String>("surena/talk",1000);
    ros::Rate loop_rate(1);
    SpeechInit();
    while (ros::ok())
    {
        adc.isStreamRunning();
        ros::spinOnce();
        loop_rate.sleep();
    }

    CleanUp();

    return 0;
}
