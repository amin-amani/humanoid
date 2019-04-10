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

/*
typedef char MY_TYPE;
#define FORMAT RTAUDIO_SINT8
*/

typedef signed short MY_TYPE;
#define FORMAT RTAUDIO_SINT16

/*
typedef S24 MY_TYPE;
#define FORMAT RTAUDIO_SINT24

typedef signed long MY_TYPE;
#define FORMAT RTAUDIO_SINT32

typedef float MY_TYPE;
#define FORMAT RTAUDIO_FLOAT32

typedef double MY_TYPE;
#define FORMAT RTAUDIO_FLOAT64
*/

// Platform-dependent sleep routines.
#if defined( WIN32 )
  #include <windows.h>
  #define SLEEP( milliseconds ) Sleep( (DWORD) milliseconds )
#else // Unix variants
  #include <unistd.h>
  #define SLEEP( milliseconds ) usleep( (unsigned long) (milliseconds * 1000.0) )
#endif



struct InputData {
  MY_TYPE* buffer;
  unsigned long bufferBytes;
  unsigned long totalFrames;
  unsigned long frameCounter;
  unsigned int channels;
};





RtAudio adc;
  InputData data;

void run_command(char* command) {
ROS_INFO ("run_command: %s" , command );//<<std::endl;
  std::cout << "run_command: " << command <<std::endl;
  qDebug()<< "run_command: " << command ;
}
void CleanUp()
{
    if ( adc.isStreamOpen() ) adc.closeStream();
    if ( data.buffer ) free( data.buffer );



}
// Interleaved buffers
int input( void * /*outputBuffer*/, void *inputBuffer, unsigned int nBufferFrames,
           double /*streamTime*/, RtAudioStreamStatus /*status*/, void *data )
{

    MY_TYPE *samples = new  MY_TYPE[nBufferFrames];
    MY_TYPE *samples0 = (MY_TYPE *)inputBuffer;
    memcpy(samples, samples0, nBufferFrames * sizeof( MY_TYPE ) );
    vad_collector(samples, run_command);
    return 0;

}



int main(int argc, char **argv)
{


    QApplication a(argc, argv);

    ros::init(argc, argv, "speech");
    ros::NodeHandle nh;
  ros::Rate loop_rate(1);

  char* dir = (char*)"sorena";
  char*  status = init("/home/cast/humanoid/surena4/src/speech/src/sorena");
  ROS_INFO( "Init status: %s" , status);
   //ROS_INFO( "Init is done. Press any key to start recording...");

  RtAudio adc;
  if ( adc.getDeviceCount() < 1 ) {
    ROS_FATAL( "No audio devices found!");
    exit( 1 );
  }
  unsigned int channels=1, fs=16000, bufferFrames, device = 0, offset = 0;
  double time = 60.0;
  // Let RtAudio print messages to stderr.
  adc.showWarnings( true );

  // Set our stream parameters for input only.
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

  // Allocate the entire data buffer before starting stream.
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

  //ROS_INFO("Recording for  %d  seconds ... writing file 'record.raw' (buffer frames = " << bufferFrames << ")." << std::endl;
//  while ( adc.isStreamRunning() ) {
//    SLEEP( 100 ); // wake every 100 ms to check if we're done
//  }
  while (ros::ok())
    {
//ROS_INFO("pub");
adc.isStreamRunning();
ros::spinOnce();
loop_rate.sleep();



    }

CleanUp();

    return 0;
}
