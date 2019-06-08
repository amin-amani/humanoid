//#ifdef CALLBACKPROC_EXPORTS
//#define CALLBACKPROC_API __declspec(dllexport)
//#else
//#define CALLBACKPROC_API __declspec(dllimport)
//#endif


char*  init(char* dir);
char* recognize(char* bytes, long count, char* key);
char* getErrorCodes();
char* getModelStatus();
//void vad_collector( signed short* samples);



void vad_collector( signed short* samples, void(*callbackfunc)(char*));

//extern void run_command(char*);

//typedef void (CALLBACK * fnCallBackFunc)(std::string value);
//extern "C"
//{
//	CALLBACKPROC_API void vad_collector(signed short* samples, fnCallBackFunc func);
//}
