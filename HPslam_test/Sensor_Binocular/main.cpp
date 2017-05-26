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
//#include "maker_binocular.h"
#include "CaptureData.h"
#include "CMutex.h"

int record_NUM=0;


/*

键盘输入抓取图像线程

*/
void* RunKeyThread(void* arg)
{

    while(1)
    {
        int c=getchar();
//        cout<<"get img"<<endl;
        if(c==10)
        {
            std::cout<<"record data\n"<<std::endl;

            record_flag=1;

        }
    }
}


int main(int argc,char * argv[])
{
    CaptureData *CaptureData_p=new CaptureData;
    USBElements sensorCapture;

    int Sensor_Frame_Buffer_data_Len=0;
    int kk = 0;
    char record_imu_path[32];

    int i=0,j=0;
    long long ImuTimeStamp_last=0;
    pthread_t threadIdKey;

    int retC = pthread_create(&threadIdKey, NULL, RunKeyThread, NULL);
    while(1)
    {
        usleep(10); //切换线程状态

        if(First_Run==0)
        {
            sleep(2);
            First_Run=1;
        }

        while(USBElement_data.Sensor_Frame_Num==0)
        {
            usleep(100);
        }

            p_Mutex.Lock();
//            std::cout<<"USBElement_data.Sensor_Frame_Num"<<USBElement_data.Sensor_Frame_Num<<std::endl;
            Sensor_Frame_Buffer_data_Len=USBElement_data.Sensor_Frame_Num;
 

            USBElement_data.Sensor_Frame_Num=0;
 
            p_Mutex.UnLock();
            #if 1    
//        std::cout<<"get num "<<Sensor_Frame_Buffer_data.size()<<std::endl;
            CaptureData_p->ImuFile_Get=fopen("../imu/imu_get.txt","a+");
            sprintf(record_imu_path,"../imu/imu.txt");

            for(i=0;i<Sensor_Frame_Buffer_data_Len;i++)
            {
                
                fprintf(CaptureData_p->ImuFile_Get, "IMG %lld\n",USBElement_data.Sensor_Frame_Buffer[i].ImgTimeStamp);
                for(j=0;j<10;j++)
                {
                    if(ImuTimeStamp_last!=0)
                    {
                        if(USBElement_data.Sensor_Frame_Buffer[i].ImuTimeStamp[j]-ImuTimeStamp_last!=5000000)
                        {
                            printf("imu error!! Last %lld new %lld img timerstamp %lld\n",ImuTimeStamp_last,USBElement_data.Sensor_Frame_Buffer[i].ImuTimeStamp[j],USBElement_data.Sensor_Frame_Buffer[i].ImgTimeStamp);
                        }                   
                    }   
                    if(record_flag==1)
                    {  
//                        sprintf(record_imu_path,"../imu/%lld.txt",USBElement_data.Sensor_Frame_Buffer[i].ImuTimeStamp[j]);
                        CaptureData_p->ImuFile=fopen(record_imu_path,"a+");
                        fprintf(CaptureData_p->ImuFile, "%lld,%f,%f,%f,%f,%f,%f\n",USBElement_data.Sensor_Frame_Buffer[i].ImuTimeStamp[j],USBElement_data.Sensor_Frame_Buffer[i].ImuValue[j][3],USBElement_data.Sensor_Frame_Buffer[i].ImuValue[j][4],
                                USBElement_data.Sensor_Frame_Buffer[i].ImuValue[j][5],USBElement_data.Sensor_Frame_Buffer[i].ImuValue[j][0],USBElement_data.Sensor_Frame_Buffer[i].ImuValue[j][1],USBElement_data.Sensor_Frame_Buffer[i].ImuValue[j][2]);	
                    
                            fclose(CaptureData_p->ImuFile);
                    }

                    ImuTimeStamp_last=USBElement_data.Sensor_Frame_Buffer[i].ImuTimeStamp[j];
                }
                
                
                cv::Mat frame(480,640,CV_8UC1,USBElement_data.Sensor_Frame_Buffer[i].LeftImgBuffer);
                cv::Mat frame1(480,640,CV_8UC1,USBElement_data.Sensor_Frame_Buffer[i].RightImgBuffer);
//                cv::Mat frame2(480,640,CV_8UC1,USBElement_data.Sensor_Frame_Buffer[i].DepthImgBuffer);
                
                if(record_flag==1)
                {	            
//                    sprintf(CaptureData_p->RightImgFilePath,"../Right/right%d.jpg",kk);
//                    sprintf(CaptureData_p->LeftImgFilePath,"../Left/left%d.jpg",kk);
                    sprintf(CaptureData_p->RightImgFilePath,"../Right/%lld.jpg",USBElement_data.Sensor_Frame_Buffer[i].ImgTimeStamp);
                    sprintf(CaptureData_p->LeftImgFilePath,"../Left/%lld.jpg",USBElement_data.Sensor_Frame_Buffer[i].ImgTimeStamp);
                    cv::imwrite(CaptureData_p->RightImgFilePath, frame1);
                    cv::imwrite(CaptureData_p->LeftImgFilePath, frame);
                    kk++;
                }
                cv::imshow("Left",frame);
                cv::imshow("Right",frame1);
//                cv::imshow("Depth",frame2);

                cv::waitKey(1);

                record_flag=0; //0 表示存储每次存储一张图片；1 表示连续存储图片。

            }


            fclose(CaptureData_p->ImuFile_Get);

#endif       
    }


    return 0;
}


