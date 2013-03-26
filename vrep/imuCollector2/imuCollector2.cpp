#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>

extern "C" {
    #include "extApi.h"
/*      #include "extApiCustom.h" if you wanna use custom remote API functions! */
}


#define PACKET_LENGHT 129


typedef struct data
{
        int gyroX;
        int gyroY;
        int gyroZ;
        int accelX;
        int accelY;
        int accelZ;
        int magnX;
        int magnY;
        int magnZ;

} inertial_data;                                // Actually not used!


int imu_data[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint32_t tRoll,tPitch,tYaw;
float Roll,Pitch,Yaw;



int tokenizer(char *s)
{

        char dels[] = ":";
        int i=0;
        int j=0;
        char *p;

        for(p=strtok(s,dels); p!=NULL; p=strtok(NULL,dels))
        {
                //      printf("%s\n",p);
                if(j>0 && j<20)
                        imu_data[i++] = atoi(p);
                j++;
        }

        return 1;
}



int main(int argc, char *argv[])
{

	int portNb=0;
	int graphHandle;

	char s;
        int serialfd,n;
	char packet[PACKET_LENGHT];
        int packet_flag=0;

        int i = 0;
        int j = 0;
	int k = 0;
		

	if(argc>=3)
	{
		portNb = atoi(argv[1]);
		graphHandle = atoi(argv[2]);
	}
	else
	{
		printf("Indicate following params: 'portNumber', 'graphHandle'\n");
		extApi_sleepMs(5000);
		return 0;
	}
	
	if(simxStart("127.0.0.1",portNb,true,true,2000)!=-1)
	{
		
		serialfd = open("/dev/ttyACM0", O_RDONLY | O_NOCTTY | O_SYNC /*| O_NONBLOCK*/);
		if (serialfd == -1)
		{
			perror("open_port: Unable to open the file descriptor");
		}


		
		while((simxGetConnectionId()!=-1) && (read(serialfd, &s, 1)))
		{

			if((packet_flag == 0) && (s == 'S'))
			{
				packet_flag = 1;
			}
			else if((packet_flag == 1) && (s == 'E'))
			{
				packet_flag = 0;
				i = 0;
				packet[129] = '\0';
				tokenizer(packet);
								
				tRoll = (uint32_t)((imu_data[9] << 31 & 0x80000000) | (imu_data[10] << 23 & 0x7F800000) | imu_data[11] & 0x007FFFFF);
	                        tPitch = (uint32_t)((imu_data[12] << 31 & 0x80000000) | (imu_data[13] << 23 & 0x7F800000) | imu_data[14] & 0x007FFFFF);
        	                tYaw = (uint32_t)((imu_data[15] << 31 & 0x80000000) | (imu_data[16] << 23 & 0x7F800000) | imu_data[17] & 0x007FFFFF);
	
        	                Roll = (*(float*)&tRoll);
                	        Pitch = (*(float*)&tPitch);
                        	Yaw = (*(float*)&tYaw);
				
				if(Roll > 360.0 || Roll < -360.0)
					Roll = 0.0;

				if(Pitch > 360.0 || Pitch < -360.0)
					Pitch = 0.0;
				
				if(Yaw > 360.0 || Yaw < -360.0)
					Yaw = 0.0;

				simxSetIntegerSignal("gyroX",imu_data[0],simx_opmode_oneshot);	
				simxSetIntegerSignal("gyroY",imu_data[1],simx_opmode_oneshot);	
				simxSetIntegerSignal("gyroZ",imu_data[2],simx_opmode_oneshot);	
				simxSetIntegerSignal("accelX",imu_data[3],simx_opmode_oneshot);	
				simxSetIntegerSignal("accelY",imu_data[4],simx_opmode_oneshot);	
				simxSetIntegerSignal("accelZ",imu_data[5],simx_opmode_oneshot);	
				simxSetIntegerSignal("magnX",imu_data[6],simx_opmode_oneshot);	
				simxSetIntegerSignal("magnY",imu_data[7],simx_opmode_oneshot);	
				simxSetIntegerSignal("magnZ",imu_data[8],simx_opmode_oneshot);	
				simxSetFloatSignal("Roll",Roll,simx_opmode_oneshot);
				simxSetFloatSignal("Pitch",Pitch,simx_opmode_oneshot);
				simxSetFloatSignal("Yaw",Yaw,simx_opmode_oneshot);
				
				//      printf("%s\n",packet);
				printf(":%6d:%6d:%6d:%6d:%6d:%6d:%6d:%6d:%6d:%6f:%6f:%6f\n",imu_data[0],imu_data[1],imu_data[2],imu_data[3],imu_data[4],imu_data[5],imu_data[6],imu_data[7],imu_data[8],Roll,Pitch,Yaw);
				extApi_sleepMs(5);
			}

			if(packet_flag == 1)
				packet[i++] = s;

			

		}
		simxFinish();
	}

}		
