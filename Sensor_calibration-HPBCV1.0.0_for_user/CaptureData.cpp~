
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <pthread.h>
#include <iostream>
#include <cyusb.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>


#include "MsgQueue.h"
#include "maker_binocular.h"
#include "CaptureData.h"
#include "CMutex.h"

#include <sys/time.h>

#include <stdarg.h>

#include <fstream>

#define TRACE_DEBUG 1;


union Para{  
    unsigned char cValue[8];  
    double dValue;  
};


using namespace std;


/*

时间戳函数

*/

int Print_TimeStamp(void)
{
        struct timeval tBegin;
        gettimeofday(&tBegin, NULL);
        
//        gettimeofday(&tEnd, NULL);
        
        long deltaTime = 1000L*tBegin.tv_sec + tBegin.tv_usec;

        printf("Time Stamp: %ldus ,", deltaTime);
}



void my_trace(const char *cmd, ...)  
{  
//    printf("%s %s ", __DATE__, __TIME__); 
#ifdef TRACE_DEBUG
    Print_TimeStamp();  
    va_list args;       //定义一个va_list类型的变量，用来储存单个参数  
    va_start(args,cmd); //使args指向可变参数的第一个参数  
    vprintf(cmd,args);  //必须用vprintf等带V的  
    va_end(args);       //结束可变参数的获取 
#endif 
} 


void ReadEEPROM(cyusb_handle *handl,unsigned char *RegAdd,unsigned char *RegData)
{
    int res_err=0;
    res_err=cyusb_control_write(handl, 0x40,0xE9,0,0,RegAdd,2,100);

    res_err=cyusb_control_read(handl, 0x40,0xEB,0,0,RegData,1,100);

//    printf("Read add 0x%02x%02x RegData 0x%02x\n",RegAdd[0],RegAdd[1],*RegData);

}

/*******************************************/


/********************************************/


string DoubleToString(double Input)  
{  
    stringstream Oss;  
    Oss<<Input;  
    return Oss.str();  
}  

void Save_Config_Parameter(cyusb_handle *handl)
{
    unsigned char Reg_data[8]={0x00};
    
    Para fx,fy,cx,cy,k1,k2,p1,p2,k3,qSc1,qSc2,qSc3,qSc4,T0,T1,T2,ans[3][3];
    Para Rfx,Rfy,Rcx,Rcy,Rk1,Rk2,Rp1,Rp2,Rk3,RqSc1,RqSc2,RqSc3,RqSc4,RT0,RT1,RT2,Rans[3][3];
    Para fx_r,fy_r,cx_r,cy_r,k1_r,k2_r,p1_r,p2_r,k3_r;
    Para Rfx_r,Rfy_r,Rcx_r,Rcy_r,Rk1_r,Rk2_r,Rp1_r,Rp2_r,Rk3_r;
    
    ofstream fs1("./configStereo.yaml");
    string sfx,sfy,scx,scy,sk1,sk2,sp1,sp2,sk3,sqSc1,sqSc2,sqSc3,sqSc4,sans0,sans1,sans2,sT0,sT1,sT2;
    string sfx_r,sfy_r,scx_r,scy_r,sk1_r,sk2_r,sp1_r,sp2_r,sk3_r;

    int res_err=cyusb_control_read(handl, 0x40,0xEF,0,0,Reg_data,8,100);
    printf("res %d 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x  \n",res_err,Reg_data[0],Reg_data[1],Reg_data[2],Reg_data[3],Reg_data[4],Reg_data[5],Reg_data[6],Reg_data[7]);
    printf("res %d %c%c%c%c%c%c%c%c\n",res_err,Reg_data[0],Reg_data[1],Reg_data[2],Reg_data[3],Reg_data[4],Reg_data[5],Reg_data[6],Reg_data[7]);

/*********************读出*************************/
    unsigned char Reg_add[2]={0x00};

    for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x11;
        Reg_add[1]=0x00+i;

        ReadEEPROM(handl,Reg_add,Rfx.cValue+i);
    }
    
    for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x11;
        Reg_add[1]=0x08+i;

        ReadEEPROM(handl,Reg_add,Rfy.cValue+i);
    }
    
    
        for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x11;
        Reg_add[1]=0x10+i;

        ReadEEPROM(handl,Reg_add,Rcx.cValue+i);
    }
        for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x11;
        Reg_add[1]=0x18+i;

        ReadEEPROM(handl,Reg_add,Rcy.cValue+i);
    }
        for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x12;
        Reg_add[1]=0x00+i;
        ReadEEPROM(handl,Reg_add,Rk1.cValue+i);
    }
        for(int i=0;i<8;i++)
    {  
        Reg_add[0]=0x12;
        Reg_add[1]=0x08+i;

        ReadEEPROM(handl,Reg_add,Rk2.cValue+i);
    }
        for(int i=0;i<8;i++)
    {  
        Reg_add[0]=0x12;
        Reg_add[1]=0x10+i;

        ReadEEPROM(handl,Reg_add,Rp1.cValue+i);
    }
        for(int i=0;i<8;i++)
    {    
        Reg_add[0]=0x12;
        Reg_add[1]=0x18+i;

        ReadEEPROM(handl,Reg_add,Rp2.cValue+i);
    }
        for(int i=0;i<8;i++)
    {    
        Reg_add[0]=0x13;
        Reg_add[1]=0x00+i;

        ReadEEPROM(handl,Reg_add,Rk3.cValue+i);
    }
        for(int i=0;i<8;i++)
    {    

        Reg_add[0]=0x14;
        Reg_add[1]=0x00+i;

        ReadEEPROM(handl,Reg_add,RqSc1.cValue+i);
    }
        for(int i=0;i<8;i++)
    {    

        Reg_add[0]=0x14;
        Reg_add[1]=0x08+i;

        ReadEEPROM(handl,Reg_add,RqSc2.cValue+i);
    }
        for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x14;
        Reg_add[1]=0x10+i;

        ReadEEPROM(handl,Reg_add,RqSc3.cValue+i);
    }
        for(int i=0;i<8;i++)
    {  

        Reg_add[0]=0x14;
        Reg_add[1]=0x18+i;

        ReadEEPROM(handl,Reg_add,RqSc4.cValue+i);
    }

    for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x15;
        Reg_add[1]=0x00+i;

        ReadEEPROM(handl,Reg_add,RT0.cValue+i);
    }
    for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x15;
        Reg_add[1]=0x08+i;

        ReadEEPROM(handl,Reg_add,RT1.cValue+i);
    }
    for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x15;
        Reg_add[1]=0x10+i;

        ReadEEPROM(handl,Reg_add,RT2.cValue+i);
    }

    for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x15;
        Reg_add[1]=0x18+i;

        ReadEEPROM(handl,Reg_add,Rans[0][0].cValue+i);
    }

    for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x16;
        Reg_add[1]=0x00+i;

        ReadEEPROM(handl,Reg_add,Rans[0][1].cValue+i);
    }

    for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x16;
        Reg_add[1]=0x08+i;

        ReadEEPROM(handl,Reg_add,Rans[0][2].cValue+i);
    }

    for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x16;
        Reg_add[1]=0x10+i;

        ReadEEPROM(handl,Reg_add,Rans[1][0].cValue+i);
    }

    for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x16;
        Reg_add[1]=0x18+i;

        ReadEEPROM(handl,Reg_add,Rans[1][1].cValue+i);
    }
    for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x17;
        Reg_add[1]=0x00+i;

        ReadEEPROM(handl,Reg_add,Rans[1][2].cValue+i);
    }
    for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x17;
        Reg_add[1]=0x08+i;

        ReadEEPROM(handl,Reg_add,Rans[2][0].cValue+i);
    }

        for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x17;
        Reg_add[1]=0x10+i;

        ReadEEPROM(handl,Reg_add,Rans[2][1].cValue+i);
    }

        for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x17;
        Reg_add[1]=0x18+i;

        ReadEEPROM(handl,Reg_add,Rans[2][2].cValue+i);
    }

    for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x18;
        Reg_add[1]=0x00+i;

        ReadEEPROM(handl,Reg_add,Rfx_r.cValue+i);
    }
    
    for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x18;
        Reg_add[1]=0x08+i;

        ReadEEPROM(handl,Reg_add,Rfy_r.cValue+i);
    }
    
    
        for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x18;
        Reg_add[1]=0x10+i;

        ReadEEPROM(handl,Reg_add,Rcx_r.cValue+i);
    }
        for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x18;
        Reg_add[1]=0x18+i;

        ReadEEPROM(handl,Reg_add,Rcy_r.cValue+i);
    }
        for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x19;
        Reg_add[1]=0x00+i;

        ReadEEPROM(handl,Reg_add,Rk1_r.cValue+i);
    }
        for(int i=0;i<8;i++)
    {  
        Reg_add[0]=0x19;
        Reg_add[1]=0x08+i;

        ReadEEPROM(handl,Reg_add,Rk2_r.cValue+i);
    }
        for(int i=0;i<8;i++)
    {  
        Reg_add[0]=0x19;
        Reg_add[1]=0x10+i;

        ReadEEPROM(handl,Reg_add,Rp1_r.cValue+i);
    }
        for(int i=0;i<8;i++)
    {    
        Reg_add[0]=0x19;
        Reg_add[1]=0x18+i;

        ReadEEPROM(handl,Reg_add,Rp2_r.cValue+i);
    }
        for(int i=0;i<8;i++)
    {    
        Reg_add[0]=0x1a;
        Reg_add[1]=0x00+i;

        ReadEEPROM(handl,Reg_add,Rk3_r.cValue+i);
    }

//printf("RT0 %f, RT1  %f, RT2  %f\n ans[0] %f %f %f\n",RT0.dValue,RT1.dValue,RT2.dValue,Rans[0][0].dValue,Rans[0][1].dValue,Rans[0][2].dValue);
//printf(" ans[1] %f %f %f \n",Rans[1][0].dValue,Rans[1][1].dValue,Rans[1][2].dValue);
//printf(" ans[2] %f %f %f \n",Rans[2][0].dValue,Rans[2][1].dValue,Rans[2][2].dValue);

sfx=DoubleToString(Rfx.dValue);
sfy=DoubleToString(Rfy.dValue);
scx=DoubleToString(Rcx.dValue);
scy=DoubleToString(Rcy.dValue);
sk1=DoubleToString(Rk1.dValue);
sk2=DoubleToString(Rk2.dValue);
sp1=DoubleToString(Rp1.dValue);
sp2=DoubleToString(Rp2.dValue);
sk3=DoubleToString(Rk3.dValue);
sqSc1=DoubleToString(RqSc1.dValue);
sqSc2=DoubleToString(RqSc2.dValue);
sqSc3=DoubleToString(RqSc3.dValue);
sqSc4=DoubleToString(RqSc4.dValue);
sans0=DoubleToString(Rans[0][0].dValue)+","+DoubleToString(Rans[0][1].dValue)+","+DoubleToString(Rans[0][2].dValue);
sans1=DoubleToString(Rans[1][0].dValue)+","+DoubleToString(Rans[1][1].dValue)+","+DoubleToString(Rans[1][2].dValue);
sans2=DoubleToString(Rans[2][0].dValue)+","+DoubleToString(Rans[2][1].dValue)+","+DoubleToString(Rans[2][2].dValue);
sT0=DoubleToString(RT0.dValue/1000.0);
sT1=DoubleToString(RT1.dValue/1000.0);
sT2=DoubleToString(RT2.dValue/1000.0);
sfx_r=DoubleToString(Rfx_r.dValue);
sfy_r=DoubleToString(Rfy_r.dValue);
scx_r=DoubleToString(Rcx_r.dValue);
scy_r=DoubleToString(Rcy_r.dValue);
sk1_r=DoubleToString(Rk1_r.dValue);
sk2_r=DoubleToString(Rk2_r.dValue);
sp1_r=DoubleToString(Rp1_r.dValue);
sp2_r=DoubleToString(Rp2_r.dValue);
sk3_r=DoubleToString(Rk3_r.dValue);
//sfx="408.1438";sfy="407.9779";
/*
scx="306.5000";scy="228.2840";sk1="-0.3206";sk2="0.0852";sp1="-0.00051618";sp2="-0.0015";sk3="0.0";
sqSc1="-0.028228";sqSc2="-0.71285";sqSc3="-0.70043";sqSc4="0.02128";sans0="0.9999,-0.0018,0.0155";sans1="0.0023,0.9995,-0.0321";
sans2="-0.0155,0.0321,0.9994";sT0="-0.06793291";sT1="-0.00253469";sT2="0.00519543";
*/
/*
*****写入文件
*************
*/
    fs1<<"%YAML:1.0\n";
    fs1<<"HPBCV:1.0\n";
    fs1<<"image_width: 640\n";
    fs1<<"image_height: 480 \n";
    fs1<<"fx: "<<sfx<<"\n";
    fs1<<"fy: "<<sfy<<"\n";
    fs1<<"cx: "<<scx<<"\n";
    fs1<<"cy: "<<scy<<"\n";
    fs1<<"k1: "<<sk1<<"\n";
    fs1<<"k2: "<<sk2<<"\n";
    fs1<<"p1: "<<sp1<<"\n";
    fs1<<"p2: "<<sp2<<"\n";
    fs1<<"k3: "<<sk3<<"\n";
    fs1<<"imu_rate: 200\n";
    fs1<<"camera_rate: 20\n";
    fs1<<"sigma_gyro: 1.7e-03 #2.7e-4\n";
    fs1<<"sigma_accel: 2.0e-02 #1.29e-2\n";
    fs1<<"sigma_pixel: 2.0\n";
    fs1<<"gravity: 9.805\n";
    fs1<<"IWantYou: 50\n";
    fs1<<"fast_threshold: 20\n";
    fs1<<"hps_ini: 25\n";
    fs1<<"hps_mat: 12\n";
    fs1<<"ncc_threshold: 0.8\n";
    fs1<<"zncc_threshold: 0.9\n";
    fs1<<"ssd_threshold: 0.8\n";
    fs1<<"max_angle: 90.0\n";
    fs1<<"prior_inverse_depth: 1.0\n";
    fs1<<"max_inverse_depth: 10.0\n";
    fs1<<"min_inverse_depth: 0.05\n";
    fs1<<"q_Sc: !!opencv-matrix\n";
    fs1<<"   rows: 4\n";
    fs1<<"   cols: 1\n";
    fs1<<"   dt: d\n";
    fs1<<"   data:["<<sqSc1<<","<<sqSc2<<","<<sqSc3<<","<<sqSc4<<" ]\n";
    fs1<<"LEFT.height: 480\n";
    fs1<<"LEFT.width: 640\n";
    fs1<<"LEFT.D: !!opencv-matrix\n";
    fs1<<"   rows: 1\n";
    fs1<<"   cols: 5\n";
    fs1<<"   dt: d\n";
    fs1<<"   data:["<<sk1<<","<<sk2<<","<<sp1<<","<<sp2<<","<<sk3<<" ]\n";
    fs1<<"LEFT.K: !!opencv-matrix\n";
    fs1<<"   rows: 3\n";
    fs1<<"   cols: 3\n";
    fs1<<"   dt: d\n";
    fs1<<"   data: ["<<sfx<<", 0.0, "<<scx<<", 0.0, "<<sfy<<", "<<scy<<" , 0.0, 0.0, 1.0]\n";
    fs1<<"R_wl:    !!opencv-matrix\n";
    fs1<<"   rows: 3\n";
    fs1<<"   cols: 3\n";
    fs1<<"   dt: d\n";
    fs1<<"   data: ["<<sans0<<",\n";
    fs1<<"          "<<sans1<<",\n";   
    fs1<<"          "<<sans2<<"]\n";
    fs1<<"t_wl:    !!opencv-matrix\n";
    fs1<<"   rows: 3\n";
    fs1<<"   cols: 1\n";
    fs1<<"   dt: d\n";
    fs1<<"   data: ["<<sT0<<","<<sT1<<","<<sT2<<"]\n";
    fs1<<"T_wl:    !!opencv-matrix\n";
    fs1<<"   rows: 3\n";
    fs1<<"   cols: 4\n";
    fs1<<"   dt: d\n";
    fs1<<"   data: ["<<sans0<<","<<sT0<<",\n";
    fs1<<"          "<<sans1<<","<<sT1<<",\n";   
    fs1<<"          "<<sans2<<","<<sT2<<"]\n";

    fs1<<"RIGHT.height: 480\n";
    fs1<<"RIGHT.width: 640\n";
    fs1<<"RIGHT.D: !!opencv-matrix\n";
    fs1<<"   rows: 1\n";
    fs1<<"   cols: 5\n";
    fs1<<"   dt: d\n";
    fs1<<"   data:["<<sk1_r<<","<<sk2_r<<","<<sp1_r<<","<<sp2_r<<","<<sk3_r<<"]\n";
    fs1<<"RIGHT.K: !!opencv-matrix\n";
    fs1<<"   rows: 3\n";
    fs1<<"   cols: 3\n";
    fs1<<"   dt: d\n";
    fs1<<"   data: ["<<sfx_r<<", 0.0, "<<scx_r<<", 0.0, "<<sfy_r<<", "<<scy_r<<" , 0.0, 0.0, 1.0]\n";
    fs1<<"R_wr:    !!opencv-matrix\n";
    fs1<<"   rows: 3\n";
    fs1<<"   cols: 3\n";
    fs1<<"   dt: d\n";
    fs1<<"   data: [1.0,0.0,0.0,\n";
    fs1<<"          0.0,1.0,0.0,\n";
    fs1<<"          0.0,0.0,1.0]\n";
    fs1<<"t_wr:    !!opencv-matrix\n";
    fs1<<"   rows: 3\n";
    fs1<<"   cols: 1\n";
    fs1<<"   dt: d\n";
    fs1<<"   data: [0.0,0.0,0.0]\n";
    fs1<<"T_wr:    !!opencv-matrix\n";
    fs1<<"   rows: 3\n";
    fs1<<"   cols: 4\n";
    fs1<<"   dt: d\n";
    fs1<<"   data: [1.0, 0.0, 0.0, 0.0,\n";
    fs1<<"          0.0, 1.0, 0.0, 0.0,\n";
    fs1<<"          0.0, 0.0, 1.0, 0.0]\n";

    fs1.close();

    exit(1);
    

/*
    for(int i;i<8;i++)
    fy.cValue[i]=fx.cValue[i];
    printf("fx.dValue %f,fy.dValue %f\n",fx.dValue,fy.dValue);
*/

//    printf("fx.dValue %f,fy.dValue %f,Rcx.dValue %f,Rcy.dValue %f,Rk1.dValue %f ,Rk2.dValue %f,Rp1.dValue %f,Rp2.dValue %f,Rk3.dValue %f\n",Rfx.dValue,Rfy.dValue,Rcx.dValue,Rcy.dValue,Rk1.dValue,Rk2.dValue,Rp1.dValue,Rp2.dValue,Rk3.dValue);


}


CaptureData::CaptureData()
{
    


    imu_strlen=180;
    weight_data.WeightFlag=0;
    weight_data.WeightValue=0.6;
    record_flag=0;

    CaptureData_main();


}

CaptureData::~CaptureData()
{


}

/*
int CaptureData::Get_CaptureData(USBElements *USBCapture,long long OutImgTimeStamp)
{

    int imu_datanum=0,imu_datanum_diff=0;
    long long           ImuTimeStamp_tmp[10];
    float               ImuValue_tmp[10][6];
    int i=0,j=0,debug_i=0;
    int k=0;
    assert(USBCapture);
    
    

    if(USBElement_data.Sensor_Frame_Buffer.size()==0)
    {
        return 1;
    }

    p_Mutex.Lock(); //lock 保护Sensor_Frame_Buffer 为原子操作
    USBCapture->Sensor_Frame_Buffer.assign(USBElement_data.Sensor_Frame_Buffer.begin(), USBElement_data.Sensor_Frame_Buffer.end());

    USBElement_data.Sensor_Frame_Buffer.clear();

    p_Mutex.UnLock();

      
    return USBCapture->Sensor_Frame_Buffer.size()*10;

}
*/




void* CaptureData::_RunThreadC(void* arg)
{
    USBElements Imu_buffer;
    unsigned char Img_Leftbuffer[640*480];
    unsigned char Img_Rigntbuffer[640*480];
    unsigned char Img_buffer[480*1280];
//    long long Img_timestamp=0;
    CaptureData *CaptureData_this=(CaptureData *)arg;
    while(1)
    {
        sleep(2);
//        CaptureData_this->Get_CaptureData(&Imu_buffer,Img_timestamp);
//        printf("Datanum %d,ImuTimeStamp %lld,ImuValue[0] id %f,ImgTimeStamp %lld\n",Imu_buffer.DataNum,Imu_buffer.ImuTimeStamp[0],Imu_buffer.ImuValue[0][0],Imu_buffer.ImgTimeStamp);
    }
}


int img_TimeStamp_last=0;
void ShowCaptureData(unsigned char *CaptureData,int CaptureData_Len)
{
    int i=0,j=0;
//    int img_TimeStamp_last=0;
    int tag1=0,tag2=0;

    union ts_vardata img_TimeStamp;
    union ts_vardata imu_TimeStamp[10]; 
    union ts_vardata imu_num; 
    union var row_size,columnsize; 

     FILE* ImuFile=NULL;



/*
    imu_num.cValue[3]=CaptureData[2]&0xff;
    imu_num.cValue[2]=CaptureData[3]&0xff;
    imu_num.cValue[1]=CaptureData[4]&0xff;
    imu_num.cValue[0]=CaptureData[5]&0xff;

    Sensor_Frame_data.DataNum=imu_num.iValue;
*/
//    printf("CaptureData[5]&0xff is 0x%02x\n",CaptureData[5]&0xff);
    Sensor_Frame_data.DataNum=10;
//    cout<<"Sensor_Frame_data.DataNum "<<Sensor_Frame_data.DataNum<<endl;

    row_size.cValue[1]=CaptureData[6]&0xff;
    row_size.cValue[0]=CaptureData[7]&0xff;

    Sensor_Frame_data.RowSize=row_size.iValue;

//    cout<<"Sensor_Frame_data.RowSize "<<Sensor_Frame_data.RowSize<<endl;

    columnsize.cValue[1]=CaptureData[8]&0xff;
    columnsize.cValue[0]=CaptureData[9]&0xff;

    Sensor_Frame_data.ColSize=columnsize.iValue;
//    cout<<"Sensor_Frame_data.ColSize "<<Sensor_Frame_data.ColSize<<endl;

    img_TimeStamp.cValue[3]=CaptureData[10]&0xff;
    img_TimeStamp.cValue[2]=CaptureData[11]&0xff;
    img_TimeStamp.cValue[1]=CaptureData[12]&0xff;
    img_TimeStamp.cValue[0]=CaptureData[13]&0xff;

    if(img_TimeStamp_last!=0)
    {
        if(img_TimeStamp.iValue-img_TimeStamp_last>=1000)
        {
            cout<<"error lost frame!!!! img_TimeStamp is "<<img_TimeStamp.iValue<<"   "<<img_TimeStamp.iValue-img_TimeStamp_last<<endl;
        }
    }
    img_TimeStamp_last=img_TimeStamp.iValue;
    Sensor_Frame_data.ImgTimeStamp=img_TimeStamp_last;
    Sensor_Frame_data.ImgTimeStamp*=100000;
    if(CaptureData_Len!=FRAMELEN)
    {
        ImuFile=fopen("../imu/imu.txt","a+");
        fprintf(ImuFile, "error frame len\n");

        fprintf(ImuFile, "imu mum %d RowSize %d ColSize %d ImgTimeStamp %lld\n",Sensor_Frame_data.DataNum,Sensor_Frame_data.RowSize,Sensor_Frame_data.ColSize,Sensor_Frame_data.ImgTimeStamp);
        for(i=0;i<500;i++)
        {
            fprintf(ImuFile, "0x%02x ",CaptureData[i]);
        }
        for(i=0;i<CaptureData_Len;i++)
        {
            if((CaptureData[i]==0x55)&&(CaptureData[i+1]==0xaa))
            fprintf(ImuFile, "img %d [0x%02x 0x%02x] %d\n",(CaptureData[i+2]&0xff)<<8+CaptureData[i+3],CaptureData[i+2],CaptureData[i+3],i);
        }
        fprintf(ImuFile, "img len %d\n",CaptureData_Len);
        fclose(ImuFile);
    }

#if 1
    /*
    IMU数据
    */
    /*
    if((CaptureData[16]==0x66)&&(CaptureData[17]==0xdd))
    {
        int mii=0;
        for(mii=0;mii<500;mii++)
        //printf("CaptureData[182+16]=0x%02x,CaptureData[183+16]=0x%02x\n",CaptureData[182+16],CaptureData[183+16]);
        if((CaptureData[16+mii]==0x44)&&(CaptureData[16+mii+1]==0xbb))
        {
            cout<<"CaptureData  "<<mii<<"\n";
        }
    }
    */

    if((CaptureData[16]==0x66)&&(CaptureData[17]==0xdd)&&(CaptureData[182+16]==0x44)&&(CaptureData[183+16]==0xbb))
    {
        i=18;//IMU数据起始坐标


        //for(j=0;j<Sensor_Frame_data.DataNum;j++)
        for(j=0;j<10;j++)
        {


            imu_TimeStamp[j].cValue[3]=(CaptureData[i+18*j]&0xff);
            imu_TimeStamp[j].cValue[2]=(CaptureData[i+1+18*j]&0xff);
            imu_TimeStamp[j].cValue[1]=(CaptureData[i+2+18*j]&0xff);
            imu_TimeStamp[j].cValue[0]=(CaptureData[i+3+18*j]&0xff);

            Sensor_Frame_data.ImuTimeStamp[j]=imu_TimeStamp[j].iValue;
            Sensor_Frame_data.ImuTimeStamp[j]*=100000;

            Sensor_Frame_data.ImuValue[j][0]=(short)(((CaptureData[i+18*j+4]&0xff)<<8)+(CaptureData[i+18*j+5]&0xff))/8192.0*GValue;
            Sensor_Frame_data.ImuValue[j][1]=(short)(((CaptureData[i+18*j+6]&0xff)<<8)+(CaptureData[i+18*j+7]&0xff))/8192.0*GValue;
            Sensor_Frame_data.ImuValue[j][2]=(short)(((CaptureData[i+18*j+8]&0xff)<<8)+(CaptureData[i+18*j+9]&0xff))/8192.0*GValue;

            Sensor_Frame_data.ImuValue[j][3]=(short)(((CaptureData[i+18*j+12]&0xff)<<8)+(CaptureData[i+18*j+13]&0xff))*3.14159/5904;
            Sensor_Frame_data.ImuValue[j][4]=(short)(((CaptureData[i+18*j+14]&0xff)<<8)+(CaptureData[i+18*j+15]&0xff))*3.14159/5904;
            Sensor_Frame_data.ImuValue[j][5]=(short)(((CaptureData[i+18*j+16]&0xff)<<8)+(CaptureData[i+18*j+17]&0xff))*3.14159/5904;

    //                cout<<"imu timestamp"<<Sensor_Frame_data.ImuTimeStamp[j]<<"\n";
    //                cout<<Sensor_Frame_data.ImuTimeStamp[j];
    //                printf("%f %f %f\n",CaptureData_this->Sensor_Frame_data.ImuValue[i-1][0],CaptureData_this->Sensor_Frame_data.ImuValue[i-1][1],CaptureData_this->Sensor_Frame_data.ImuValue[i-1][2]);
    //                cout<<CaptureData_this->CaptureData_this->Sensor_Frame_data.ImuValue[i-1][0]<<","<<CaptureData_this->Sensor_Frame_data.ImuValue[i-1][3]<<endl;
            

        }
    }
#endif
#if 1

    int mj=0;
//    if((CaptureData[200]==0x55)&&(CaptureData[201]==0xaa))
//    {
        i=184+16;//图像数据的起始位置

        
#if 1

        memmove(Sensor_Frame_data.LeftImgBuffer,CaptureData+i,307200);
    
        memmove(Sensor_Frame_data.RightImgBuffer,CaptureData+i+307200,307200);


        if(First_Run==1)
        {
            p_Mutex.Lock();


            memmove(&(USBElement_data.Sensor_Frame_Buffer[USBElement_data.Sensor_Frame_Num++]),&Sensor_Frame_data,sizeof(Sensor_Frame));
            if(USBElement_data.Sensor_Frame_Num==10)
                USBElement_data.Sensor_Frame_Num=0;
            usleep(100);

            p_Mutex.UnLock();

        }
#endif


//    }


#endif 

}

void* CaptureData::_RunThreadB_Mon(void* arg)
{

	unsigned long timestamp=0,timestamp_last=0;

    unsigned char msg_data[927876]={0};
	int lastmsg_lan=0,frame_lan=0;
	int i=0;
    CaptureData *CaptureData_this=(CaptureData *)arg;
    union var RowSize;
    union var ColSize;
    int recvn=0;


    struct timeval tBegin,tEnd;
    long long deltaTime=0;



    USBElement_data.Sensor_Frame_Num=0;

    int head_Tag=0;
    int i_last=0;
    int data_len_tmp=0;
    int frame_lan_err_flag=0;

//    unsigned char Reg_data[3]={0x1F,0x00,0x03};
    
    Save_Config_Parameter(CaptureData_this->m_makerbinocular.h1); //将标定参数写到EEPROM中。

    
}





int CaptureData::CaptureData_main()
{

    
    
    char input[] = "ThreadA";
    char inputB[] = "ThreadB";


//    int retA = pthread_create(&threadIdA, NULL, _RunThreadA_Mon, this);
    int retB = pthread_create(&threadIdB, NULL, _RunThreadB_Mon, this);

//    int retC = pthread_create(&threadIdB, NULL, _RunKeyThread, this);


//    int retC = pthread_create(&threadIdC, NULL, _RunThreadC, this);

 
    return 0;
}
