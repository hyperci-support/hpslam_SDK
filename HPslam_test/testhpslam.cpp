#include <stdio.h>
#include <iostream>
#include "hpslam.hpp"
#include <pthread.h>
#include <sys/time.h>
#include <unistd.h>



int main()
{
    

    pthread_t threadIdhpslam;

    int rethp = pthread_create(&threadIdhpslam, NULL, &hpslam_main, NULL);//hpslam 线程
    while(1)
    {
	    if(hpslamout_flag==1)
	    {
	    	printf("6DOF out xyz(%f,%f，%f),Quaternion[%f,%f,%f,%f]\n",SixDOF[0],SixDOF[1],SixDOF[2],SixDOF[3],SixDOF[4],SixDOF[5],SixDOF[6]);
		hpslamout_flag=0;
	    }
            else
               usleep(20000);
    }
}
