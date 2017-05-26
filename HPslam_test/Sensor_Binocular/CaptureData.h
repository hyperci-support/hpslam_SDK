#ifndef _CaptureData_h_
#define _CaptureData_h_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <fstream.h>
#include "CMutex.h"
#include "MsgQueue.h"
#include "maker_binocular.h"

/*
单目参数
*/
/*
#define FRAMELEN    309500
#define PRESTRLEN   620000
#define READSTRLEN  309248
*/

/*
双目参数（depth）
*/
#define FRAMELEN    614600
#define PRESTRLEN   1229560
#define READSTRLEN  614400


//bool camera_type=0;//camera类型0:双目，1单目
int record_flag=0; 



int                 m_makerbinocular_update=0;

unsigned char msg_data_temp[PRESTRLEN]={0};

unsigned char msg_data_temp_last[PRESTRLEN]={0};


int msg_data_temp_last_len=0;



int First_Run=0;

float GValue=9.805;
//    float GValue=1.0;

CMutex p_Mutex;

int Sensor_Mutex=0;


typedef struct {    
    unsigned short      RowSize;
    unsigned short      ColSize;
    unsigned int        DataNum;
    float               ImuValue[10][6];
    long long           ImuTimeStamp[10];
    long long           ImgTimeStamp;
    unsigned char       LeftImgBuffer[640*480];
    unsigned char       RightImgBuffer[640*480];
    unsigned char       DepthImgBuffer[640*480];
} Sensor_Frame;

typedef struct {    
    Sensor_Frame Sensor_Frame_Buffer[10];
    int Sensor_Frame_Num;
} USBElements;


USBElements USBElement_data;
Sensor_Frame Sensor_Frame_data;

union var{  
    unsigned char cValue[2];  
    unsigned short iValue;  
};

union ts_vardata{  
        unsigned char cValue[4];  
        unsigned int iValue;  
};

typedef struct {
    bool                WeightFlag=0;
    float               WeightValue=0.6;
    float               LastImuValue[6];
} WeightElements;

union Para{  
    unsigned char cValue[8];  
    double dValue;  
};

class CaptureData
{
	public:
		CaptureData();
		~CaptureData();

        int Get_CaptureData(USBElements *USBCapture,long long OutImgTimeStamp);


        
        int transferd;
        int imu_strlen=180;


       
         FILE* ImuFile_Get;
         FILE* ImuFile;

        char LeftImgFilePath[512];
        char RightImgFilePath[512];



        int find_Frameheader=0;
        makerbinocular      m_makerbinocular;


    private:


        MsgQueue *Thread_q=new MsgQueue;
        pthread_t threadIdA;
        pthread_t threadIdB;
        pthread_t threadIdC;

        bool Monocular=0;
        bool Binocular=0;

       


        int                 recorrect_capturedata=0;

        
        FILE* ImgFile;
        FILE* LeftImgFile;
        FILE* RightImgFile;

       

        long long ImgTimestamp;
        long long ImgTimestamp_last;
        

        WeightElements weight_data;
        

        static void* _RunThreadA_Mon(void *arg);
        static void* _RunThreadB_Mon(void *arg);
        static void* _RunThread_resolve_Mon(void *arg);
        static void* _RunThreadC(void *arg);
        static void* _RunKeyThread(void* arg);
        int CaptureData_main();

};


#endif