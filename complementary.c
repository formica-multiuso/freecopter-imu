/*
    FreeCopter IMU firmware - Copyright (C) 2012, +inf
			      Roberto Marino

    FreeCopter IMU is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    FreeCopter IMU is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
	    
    FreeCopter is based on

    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/


#include <stdlib.h>
#include <ch.h>
#include <hal.h>

#define TAU_C 0.075		// Time-constant
#define LOOPTIME 20 		// Expressed in milli-seconds
#define DT 0.05			// Expressed in seconds
#define X_ACC_OFFSET 0
#define Y_ACC_OFFSET -1
#define Z_ACC_OFFSET 16000
#define X_GYRO_OFFSET -30
#define Y_GYRO_OFFSET -20
#define Z_GYRO_OFFSET 0 
#define X_ACC_SENS_2G 0.001 // [g/digit]
#define Y_ACC_SENS_2G 0.001 // [g/digit]
#define Z_ACC_SENS_2G 0.001 // [g/digit]
#define X_GYRO_SENS 0.00875 // [dps/digit]
#define Y_GYRO_SENS 0.00875 // [dps/digit]
#define Z_GYRO_SENS 0.00875 // [dps/digit]

float Roll, Pitch, Yaw;
float tau_c, a_compl;
extern int16_t accelX, accelY, accelZ;
extern int16_t gyroX, gyroY, gyroZ;
extern char cRoll[2], cPitch[2],cYaw[2];

float x_acc, y_acc, z_acc;
float x_gyro, y_gyro, z_gyro;
	

void complementary_filter(void)
{

	tau_c = (float) TAU_C;
	a_compl = 0.0;
	a_compl = (float) (TAU_C/(TAU_C+DT)); //0.882352941 calcolato in calcolatrice
	

	x_acc = ((float)(accelX - X_ACC_OFFSET)*X_ACC_SENS_2G*9.8);
	y_acc = ((float)(accelY - Y_ACC_OFFSET)*Y_ACC_SENS_2G*9.8);
	z_acc = ((float)(accelZ - Z_ACC_OFFSET)*Z_ACC_SENS_2G*9.8);
	x_gyro = ((float)(gyroX - X_GYRO_OFFSET)*X_GYRO_SENS);
	y_gyro = ((float)(gyroY - Y_GYRO_OFFSET)*Y_GYRO_SENS);
	z_gyro = ((float)(gyroZ - Z_GYRO_OFFSET)*Z_GYRO_SENS);


	Roll = (a_compl)*(Roll + x_gyro*DT) + (1-a_compl)*(x_acc);
	Pitch = (a_compl)*(Pitch + y_gyro*DT) + (1-a_compl)*(y_acc);
	Yaw = (a_compl)*(Yaw + z_gyro*DT) + (1-a_compl)*(z_acc);	
}
