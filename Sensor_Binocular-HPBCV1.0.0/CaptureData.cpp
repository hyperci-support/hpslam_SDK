
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <pthread.h>
#include <iostream>
#include <cyusb.h>
#include <opencv2/opencv.hpp>


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


void WriteEEPROM(cyusb_handle *handl,unsigned char *RegAdd,unsigned char *RegData)
{
    int res_err=0;
    
    cyusb_control_write(handl, 0x40,0xE9,0,0,RegAdd,2,100);
    res_err=cyusb_control_write(handl, 0x40,0xEA,0,0,RegData,1,100);

    printf("Write add 0x%02x%02x RegData 0x%02x\n",RegAdd[0],RegAdd[1],*RegData);

}

void ReadEEPROM(cyusb_handle *handl,unsigned char *RegAdd,unsigned char *RegData)
{
    int res_err=0;
    res_err=cyusb_control_write(handl, 0x40,0xE9,0,0,RegAdd,2,100);

    res_err=cyusb_control_read(handl, 0x40,0xEB,0,0,RegData,1,100);

    printf("Read add 0x%02x%02x RegData 0x%02x\n",RegAdd[0],RegAdd[1],*RegData);

}

void Save_Config_Parameter(cyusb_handle *handl)
{
    unsigned char Reg_data[8]={0x00};
    
    Para fx,fy,cx,cy,k1,k2,p1,p2,k3,qSc1,qSc2,qSc3,qSc4;
    Para Rfx,Rfy,Rcx,Rcy,Rk1,Rk2,Rp1,Rp2,Rk3;
    int res_err=cyusb_control_read(handl, 0x40,0xEF,0,0,Reg_data,8,100);
    printf("res %d 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x  \n",res_err,Reg_data[0],Reg_data[1],Reg_data[2],Reg_data[3],Reg_data[4],Reg_data[5],Reg_data[6],Reg_data[7]);
    printf("res %d %c%c%c%c%c%c%c%c\n",res_err,Reg_data[0],Reg_data[1],Reg_data[2],Reg_data[3],Reg_data[4],Reg_data[5],Reg_data[6],Reg_data[7]);

    fx.dValue=408.1438;
    fy.dValue=407.9779;
    cx.dValue=306.5000;
    cy.dValue=228.2840;
    k1.dValue=-0.3206;
    k2.dValue=0.0852;
    p1.dValue=-0.00051618;
    p2.dValue=-0.0015;
    k3.dValue=0.0;
    qSc1.dValue=-0.70113;
    qSc2.dValue=-0.010768;
    qSc3.dValue=0.0021732;
    qSc4.dValue=0.71295;

    unsigned char Reg_add[2]={0x00};
    unsigned char Reg_data1[1]={0xFF};
    /*
    Reg_add[0]=0x11;
    Reg_add[1]=0x00;
    cyusb_control_write(handl, 0x40,0xE9,0,0,Reg_add,2,100);
    cyusb_control_write(handl, 0x40,0xEA,0,0,fx.cValue,1,100);
    Reg_add[0]=0x11;
    Reg_add[1]=0x01;
    cyusb_control_write(handl, 0x40,0xE9,0,0,Reg_add,2,100);
    cyusb_control_write(handl, 0x40,0xEA,0,0,fx.cValue+1,1,100);
    Reg_add[0]=0x11;
    Reg_add[1]=0x02;
    cyusb_control_write(handl, 0x40,0xE9,0,0,Reg_add,2,100);
    cyusb_control_write(handl, 0x40,0xEA,0,0,fx.cValue+2,1,100);
    Reg_add[0]=0x11;
    Reg_add[1]=0x03;
    cyusb_control_write(handl, 0x40,0xE9,0,0,Reg_add,2,100);
    cyusb_control_write(handl, 0x40,0xEA,0,0,fx.cValue+3,1,100);
*/





    for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x11;
        Reg_add[1]=0x00+i;
        WriteEEPROM(handl,Reg_add,fx.cValue+i);
     //   ReadEEPROM(handl,Reg_add,Rfx.cValue+i);
    }
    
        for(int i=0;i<8;i++)
    { 
        Reg_add[0]=0x11;
        Reg_add[1]=0x08+i;
        WriteEEPROM(handl,Reg_add,fy.cValue+i);
     //   ReadEEPROM(handl,Reg_add,Rfy.cValue+i);
    }
    
        for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x11;
        Reg_add[1]=0x10+i;
        WriteEEPROM(handl,Reg_add,cx.cValue+i);
    //    ReadEEPROM(handl,Reg_add,Rcx.cValue+i);
    }
        for(int i=0;i<8;i++)
    {    
        Reg_add[0]=0x11;
        Reg_add[1]=0x18+i;
        WriteEEPROM(handl,Reg_add,cy.cValue+i);
   //     ReadEEPROM(handl,Reg_add,Rcy.cValue+i);
    }
    
    
        for(int i=0;i<8;i++)
    {    
        Reg_add[0]=0x12;
        Reg_add[1]=0x00+i;
        WriteEEPROM(handl,Reg_add,k1.cValue+i);
   //     ReadEEPROM(handl,Reg_add,Rk1.cValue+i);
    }
    
        for(int i=0;i<8;i++)
    {   
        Reg_add[0]=0x12;
        Reg_add[1]=0x08+i;
        WriteEEPROM(handl,Reg_add,k2.cValue+i);
   //     ReadEEPROM(handl,Reg_add,Rk2.cValue+i);
    }
        for(int i=0;i<8;i++)
    {  
        Reg_add[0]=0x12;
        Reg_add[1]=0x10+i;
        WriteEEPROM(handl,Reg_add,p1.cValue+i);
   //     ReadEEPROM(handl,Reg_add,Rp1.cValue+i);
    }
        for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x12;
        Reg_add[1]=0x18+i;
        WriteEEPROM(handl,Reg_add,p2.cValue+i);
   //     ReadEEPROM(handl,Reg_add,Rp2.cValue+i);
    }
    
        for(int i=0;i<8;i++)
    {   
        Reg_add[0]=0x13;
        Reg_add[1]=0x00+i;
        WriteEEPROM(handl,Reg_add,k3.cValue+i);
   //     ReadEEPROM(handl,Reg_add,Rk3.cValue+i);


//    printf("fx.dValue %f,fy.dValue %f,Rcx.dValue %f,Rcy.dValue %f,Rk1.dValue %f ,Rk2.dValue %f,Rp1.dValue %f,Rp2.dValue %f,Rk3.dValue %f\n",Rfx.dValue,Rfy.dValue,Rcx.dValue,Rcy.dValue,Rk1.dValue,Rk2.dValue,Rp1.dValue,Rp2.dValue,Rk3.dValue);
    }
    
        for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x14;
        Reg_add[1]=0x00+i;
        WriteEEPROM(handl,Reg_add,qSc1.cValue+i);
   //     ReadEEPROM(handl,Reg_add,qSc1.cValue+i);
    }
        for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x14;
        Reg_add[1]=0x08+i;
        WriteEEPROM(handl,Reg_add,qSc2.cValue+i);
   //     ReadEEPROM(handl,Reg_add,qSc2.cValue+i);
    }
        for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x14;
        Reg_add[1]=0x10+i;
        WriteEEPROM(handl,Reg_add,qSc3.cValue+i);
   //     ReadEEPROM(handl,Reg_add,qSc3.cValue+i);
    }
        for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x14;
        Reg_add[1]=0x18+i;
        WriteEEPROM(handl,Reg_add,qSc4.cValue+i);
   //     ReadEEPROM(handl,Reg_add,qSc4.cValue+i);

    }


/*
    for(int i;i<8;i++)
    fy.cValue[i]=fx.cValue[i];
    printf("fx.dValue %f,fy.dValue %f\n",fx.dValue,fy.dValue);
*/

    for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x11;
        Reg_add[1]=0x00+i;
//        WriteEEPROM(handl,Reg_add,fx.cValue+i);
        ReadEEPROM(handl,Reg_add,Rfx.cValue+i);
    }
    
    for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x11;
        Reg_add[1]=0x08+i;
//        WriteEEPROM(handl,Reg_add,fy.cValue+i);
        ReadEEPROM(handl,Reg_add,Rfy.cValue+i);
    }
    
    
        for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x11;
        Reg_add[1]=0x10+i;
//        WriteEEPROM(handl,Reg_add,cx.cValue+i);
        ReadEEPROM(handl,Reg_add,Rcx.cValue+i);
    }
        for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x11;
        Reg_add[1]=0x18+i;
//        WriteEEPROM(handl,Reg_add,cy.cValue+i);
        ReadEEPROM(handl,Reg_add,Rcy.cValue+i);
    }
        for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x12;
        Reg_add[1]=0x00+i;
//        WriteEEPROM(handl,Reg_add,k1.cValue+i);
        ReadEEPROM(handl,Reg_add,Rk1.cValue+i);
    }
        for(int i=0;i<8;i++)
    {  
        Reg_add[0]=0x12;
        Reg_add[1]=0x08+i;
//        WriteEEPROM(handl,Reg_add,k2.cValue+i);
        ReadEEPROM(handl,Reg_add,Rk2.cValue+i);
    }
        for(int i=0;i<8;i++)
    {  
        Reg_add[0]=0x12;
        Reg_add[1]=0x10+i;
//        WriteEEPROM(handl,Reg_add,p1.cValue+i);
        ReadEEPROM(handl,Reg_add,Rp1.cValue+i);
    }
        for(int i=0;i<8;i++)
    {    
        Reg_add[0]=0x12;
        Reg_add[1]=0x18+i;
//        WriteEEPROM(handl,Reg_add,p2.cValue+i);
        ReadEEPROM(handl,Reg_add,Rp2.cValue+i);
    }
        for(int i=0;i<8;i++)
    {    
        Reg_add[0]=0x13;
        Reg_add[1]=0x00+i;
//        WriteEEPROM(handl,Reg_add,k3.cValue+i);
        ReadEEPROM(handl,Reg_add,Rk3.cValue+i);
    }
        for(int i=0;i<8;i++)
    {    


//    printf("fx.dValue %f,fy.dValue %f,Rcx.dValue %f,Rcy.dValue %f,Rk1.dValue %f ,Rk2.dValue %f,Rp1.dValue %f,Rp2.dValue %f,Rk3.dValue %f\n",Rfx.dValue,Rfy.dValue,Rcx.dValue,Rcy.dValue,Rk1.dValue,Rk2.dValue,Rp1.dValue,Rp2.dValue,Rk3.dValue);

        Reg_add[0]=0x14;
        Reg_add[1]=0x00+i;
 //       WriteEEPROM(handl,Reg_add,qSc1.cValue+i);
        ReadEEPROM(handl,Reg_add,qSc1.cValue+i);
    }
        for(int i=0;i<8;i++)
    {    

        Reg_add[0]=0x14;
        Reg_add[1]=0x08+i;
//        WriteEEPROM(handl,Reg_add,qSc2.cValue+i);
        ReadEEPROM(handl,Reg_add,qSc2.cValue+i);
    }
        for(int i=0;i<8;i++)
    {
        Reg_add[0]=0x14;
        Reg_add[1]=0x10+i;
//        WriteEEPROM(handl,Reg_add,qSc3.cValue+i);
        ReadEEPROM(handl,Reg_add,qSc3.cValue+i);
    }
        for(int i=0;i<8;i++)
    {  

        Reg_add[0]=0x14;
        Reg_add[1]=0x18+i;
//        WriteEEPROM(handl,Reg_add,qSc4.cValue+i);
        ReadEEPROM(handl,Reg_add,qSc4.cValue+i);
    }

    printf("fx.dValue %f,fy.dValue %f,Rcx.dValue %f,Rcy.dValue %f,Rk1.dValue %f ,Rk2.dValue %f,Rp1.dValue %f,Rp2.dValue %f,Rk3.dValue %f\n",Rfx.dValue,Rfy.dValue,Rcx.dValue,Rcy.dValue,Rk1.dValue,Rk2.dValue,Rp1.dValue,Rp2.dValue,Rk3.dValue);


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

        memmove(Sensor_Frame_data.RightImgBuffer,CaptureData+i,307200);
    
        memmove(Sensor_Frame_data.LeftImgBuffer,CaptureData+i+307200,307200);


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
    
    while(1)
    {
//        cout<<">>>>>>>_RunThreadB_Mon"<<endl;
        usleep(10); //切换线程状态

//        CaptureData_this->Thread_q->recvMsg(code,p_msg);
        gettimeofday(&tBegin, NULL);
    
//        gettimeofday(&tEnd, NULL);
    

//	    int error = libusb_bulk_transfer(CaptureData_this->g_usb_handle, 0x86, msg_data_temp_last+16,buffer_size,&(CaptureData_this->transferd),0);
        i_last=0;
        head_Tag=0;
        int error = cyusb_bulk_transfer(CaptureData_this->m_makerbinocular.h1, 0x86, msg_data_temp_last+msg_data_temp_last_len, READSTRLEN, &(CaptureData_this->transferd),1000);
//        cout<<"CaptureData_this->transferd "<<CaptureData_this->transferd<<endl;
        if(CaptureData_this->transferd!=READSTRLEN)
        {
            gettimeofday(&tEnd, NULL);
            deltaTime = 1000000L*(long long)tEnd.tv_sec + tEnd.tv_usec-(1000000L*(long long)tBegin.tv_sec + tBegin.tv_usec);
            cout<<"<<<<cyusb_bulk_transfer error>>>>>>"<<endl;
            printf("deltaTime: %lld us m_makerbinocular_update %d error %d\n",deltaTime,m_makerbinocular_update,error);

        }
        if(CaptureData_this->transferd==READSTRLEN) //接受到数据
        {
            
//            memcpy(msg_data_temp_last+16,CaptureData_this->m_makerbinocular.datain,CaptureData_this->transferd);
        
            gettimeofday(&tEnd, NULL);
            deltaTime = 1000000L*(long long)tEnd.tv_sec + tEnd.tv_usec-(1000000L*(long long)tBegin.tv_sec + tBegin.tv_usec);

    //        printf("deltaTime: %lld us m_makerbinocular_update %d error %d\n",deltaTime,m_makerbinocular_update,error);
            if(deltaTime>100000) //接受数据之间消耗的时间50帧，20ms左右，超过40毫秒说明丢帧
            {
                cout<<"<<<<handle too slow lost data>>>>>>"<<endl;
                printf("deltaTime: %lld us m_makerbinocular_update %d error %d\n",deltaTime,m_makerbinocular_update,error);
            }

        

                gettimeofday(&tBegin, NULL);
    
                CaptureData_this->find_Frameheader=0;

                data_len_tmp=CaptureData_this->transferd+msg_data_temp_last_len;
//                memset(msg_data_temp_last+data_len_tmp,0,sizeof(msg_data_temp_last)-data_len_tmp);
                for(i=0;i<CaptureData_this->transferd+msg_data_temp_last_len;i++)
                {

                    if(((msg_data_temp_last[i]&0xff)==0x33)&&((msg_data_temp_last[i+1]&0xff)==0xcc)&&((msg_data_temp_last[i+14]&0xff)==0x22)&&((msg_data_temp_last[i+15]&0xff)==0xdd))
                    {
    //                    cout<<"find Frameheader lastmsg_lan is"<<lastmsg_lan<<"find head i ="<<i<<endl;
                        if(i==0)
                        {
//                            i+=100;
                            continue;
                        }
                       
                        
                        
                        frame_lan=i-i_last;
                        
//                        cout<<"frame_lan is "<<frame_lan<<endl;
                        if(frame_lan==FRAMELEN)
                        {
                            frame_lan_err_flag=1;
                        }
                        if(frame_lan!=FRAMELEN)
                        {
                            if(frame_lan_err_flag==1)
                            {
                                cout<<"###############frame error#####"<<frame_lan<<"lastmsg_lan"<<i_last<<"i="<<i<<endl;
                            }
                        }

                        memcpy(msg_data,msg_data_temp_last+i_last,frame_lan);

                        i_last=i;

                        if(First_Run==1)
                        {
                         
                            ShowCaptureData(msg_data,frame_lan); 
                
                        }
                        head_Tag=i;
                        memset(msg_data,0,sizeof(msg_data));

                    }
                }
                if(i==CaptureData_this->transferd+msg_data_temp_last_len)
                {
                    msg_data_temp_last_len=CaptureData_this->transferd+msg_data_temp_last_len-head_Tag;
                    //cout<<"msg_data_temp_last_len "<<msg_data_temp_last_len<<"head_Tag "<<head_Tag<<"size"<<sizeof(msg_data_temp)<<endl;
                    memcpy(msg_data_temp,msg_data_temp_last+head_Tag,msg_data_temp_last_len);
                   

                    memset(msg_data_temp_last,0,sizeof(msg_data_temp_last));
                    memcpy(msg_data_temp_last,msg_data_temp,msg_data_temp_last_len);
                }

                gettimeofday(&tEnd, NULL);

                deltaTime = 1000000L*(long long)tEnd.tv_sec + tEnd.tv_usec-(1000000L*(long long)tBegin.tv_sec + tBegin.tv_usec);
                if(deltaTime>25000) //解析数据消耗的时间
                {
//                    cout<<"handle time too long maybe lost data deltaTime is "<<deltaTime<<endl;
                }
        }
//        cout<<">>>>>>>_RunThreadB_Mon  end"<<endl;
    }
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
