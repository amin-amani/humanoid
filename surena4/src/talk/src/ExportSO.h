//-----------------------------------------------------------------------------

#ifndef EXPORTSOH
#define EXPORTSOH

#include "stdint.h"
//---------------------------------------------------------------------------
extern "C" {
    void RHM_INIT(const char *aDatabasePath);
    void RHM_SetSpeed(int aSpeed);
    void RHM_SetVolume(double aVolume);
    void RHM_SetSpeakerVoice(int aVoiceNumber);
    long RHM_Speak(const wchar_t* aText, void** aWaveBuff);
}

#endif
