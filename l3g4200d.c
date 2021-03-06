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

#include "l3g4200d.h"

int16_t gyroX, gyroY, gyroZ;

static i2cflags_t errors[5] = {0,0,0,0,0};
static i2cflags_t g_errors[6] = {0,0,0,0,0,0};


// INIT VERSION 0
/*
int l3g4200d_init(void)
{

	msg_t status = RDY_OK;
        uint8_t buffer_tx[2];
        systime_t tmo = MS2ST(4);

        i2cAcquireBus(&I2CD1);
	buffer_tx[0] = CTRL_REG2;
        buffer_tx[1] = 0x00;
        status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,buffer_tx,2,NULL,0,tmo);
        if (status != RDY_OK){
                 errors[0] = i2cGetErrors(&I2CD1);}
        buffer_tx[0] = CTRL_REG3;
        buffer_tx[1] = 0x00;
        status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,buffer_tx,2,NULL,0,tmo);
        if (status != RDY_OK){
                 errors[1] = i2cGetErrors(&I2CD1);}
	buffer_tx[0] = CTRL_REG4;
        buffer_tx[1] = 0x00;	// 0x00 250 deg/sec, 0x10 500 deg/sec, 0x30 2000 deg/sec
        status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,buffer_tx,2,NULL,0,tmo);
        if (status != RDY_OK){
                 errors[2] = i2cGetErrors(&I2CD1);}
        buffer_tx[0] = CTRL_REG5;
        buffer_tx[1] = 0x00;
        status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,buffer_tx,2,NULL,0,tmo);
        if (status != RDY_OK){
                 errors[3] = i2cGetErrors(&I2CD1);}
	buffer_tx[0] = CTRL_REG1;
        buffer_tx[1] = 0x0F;
        status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,buffer_tx,2,NULL,0,tmo);
        if (status != RDY_OK){
                 errors[4] = i2cGetErrors(&I2CD1);}
	i2cReleaseBus(&I2CD1);

	return 0;
}
*/

// INIT VERSION 1
int l3g4200d_init(void)
{

	msg_t status = RDY_OK;
        uint8_t buffer_tx[2];
        systime_t tmo = MS2ST(4);

        i2cAcquireBus(&I2CD1);
	buffer_tx[0] = CTRL_REG2;
        buffer_tx[1] = 0x09;
        status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,buffer_tx,2,NULL,0,tmo);
        if (status != RDY_OK){
                 errors[0] = i2cGetErrors(&I2CD1);}
        buffer_tx[0] = CTRL_REG3;
        buffer_tx[1] = 0x00;
        status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,buffer_tx,2,NULL,0,tmo);
        if (status != RDY_OK){
                 errors[1] = i2cGetErrors(&I2CD1);}
	buffer_tx[0] = CTRL_REG4;
        buffer_tx[1] = 0x50;	// 0x00 250 deg/sec, 0x10 500 deg/sec, 0x30 2000 deg/sec
        status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,buffer_tx,2,NULL,0,tmo);
        if (status != RDY_OK){
                 errors[2] = i2cGetErrors(&I2CD1);}
        buffer_tx[0] = CTRL_REG5;
        buffer_tx[1] = 0x00;
        status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,buffer_tx,2,NULL,0,tmo);
        if (status != RDY_OK){
                 errors[3] = i2cGetErrors(&I2CD1);}
	buffer_tx[0] = CTRL_REG1;
        buffer_tx[1] = 0x0F;
        status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,buffer_tx,2,NULL,0,tmo);
        if (status != RDY_OK){
                 errors[4] = i2cGetErrors(&I2CD1);}
	i2cReleaseBus(&I2CD1);

	return 0;
}



// Read Version 1
/*
void l3g4200d_read_gyro(void)
{
        msg_t status = RDY_OK;
        uint8_t buffer_tx;
	uint8_t buffer_rx_X[2];
	uint8_t buffer_rx_Y[2];
	uint8_t buffer_rx_Z[2];
        systime_t tmo = MS2ST(4);

        i2cAcquireBus(&I2CD1);
	buffer_tx = OUT_X_L;
        status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,&buffer_tx,1,&buffer_rx_X[0],1,tmo);
        if (status != RDY_OK){
                 g_errors[0] = i2cGetErrors(&I2CD1);}
       	buffer_tx = OUT_X_H;
	status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,&buffer_tx,1,&buffer_rx_X[1],1,tmo);
        if (status != RDY_OK){
                 g_errors[1] = i2cGetErrors(&I2CD1);}
        buffer_tx = OUT_Y_L;
	status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,&buffer_tx,1,&buffer_rx_Y[0],1,tmo);
        if (status != RDY_OK){
                 g_errors[2] = i2cGetErrors(&I2CD1);}
        buffer_tx = OUT_Y_H;
	status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,&buffer_tx,1,&buffer_rx_Y[1],1,tmo);
    	if (status != RDY_OK){
                 g_errors[3] = i2cGetErrors(&I2CD1);}
        buffer_tx = OUT_Z_L;
	status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,&buffer_tx,1,&buffer_rx_Z[0],1,tmo);
        if (status != RDY_OK){
                 g_errors[4] = i2cGetErrors(&I2CD1);}
	buffer_tx = OUT_Z_H;
        status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,&buffer_tx,1,&buffer_rx_Z[1],1,tmo);
        if (status != RDY_OK){
                 g_errors[5] = i2cGetErrors(&I2CD1);}
        i2cReleaseBus(&I2CD1);

	gyroX = (buffer_rx_X[1] << 8) | buffer_rx_X[0];
        gyroY = (buffer_rx_Y[1] << 8) | buffer_rx_Y[0];
        gyroZ = (buffer_rx_Z[1] << 8) | buffer_rx_Z[0];
}
*/
// READ VERSION 0
void l3g4200d_read_gyro(void)
{
        msg_t status = RDY_OK;
        uint8_t buffer_tx;
	uint8_t status_reg;
	uint8_t buffer_rx_X[2];
	uint8_t buffer_rx_Y[2];
	uint8_t buffer_rx_Z[2];
        systime_t tmo = MS2ST(4);

        i2cAcquireBus(&I2CD1);

	//buffer_tx = STATUS_REG;
	//status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,&buffer_tx,1,&status_reg,1,TIME_INFINITE);	
	//if (status != RDY_OK){
        //         g_errors[1] = i2cGetErrors(&I2CD1);}
       

	//if((status_reg | (1<<3)) == 0 || (status_reg | (1<<7)) == 1) return;

	buffer_tx = OUT_X_L;
        status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,&buffer_tx,1,buffer_rx_X,2,tmo);
        if (status != RDY_OK){
                 g_errors[0] = i2cGetErrors(&I2CD1);}
       
	//status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,OUT_X_L,1,&buffer_rx[1],1,tmo);
        //if (status != RDY_OK){
        //         g_errors[1] = i2cGetErrors(&I2CD1);}
        buffer_tx = OUT_Y_L;
	status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,&buffer_tx,1,buffer_rx_Y,2,tmo);
        if (status != RDY_OK){
                 g_errors[2] = i2cGetErrors(&I2CD1);}
        //status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,OUT_Y_L,1,&buffer_rx[3],1,tmo);
        //if (status != RDY_OK){
        //         g_errors[3] = i2cGetErrors(&I2CD1);}
        buffer_tx = OUT_Z_L;
	status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,&buffer_tx,1,buffer_rx_Z,2,tmo);
        if (status != RDY_OK){
                 g_errors[4] = i2cGetErrors(&I2CD1);}
       // status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,OUT_Z_L,1,&buffer_rx[5],1,tmo);
       // if (status != RDY_OK){
       //          g_errors[5] = i2cGetErrors(&I2CD1);}
        i2cReleaseBus(&I2CD1);

	gyroX = (buffer_rx_X[1] << 8) | buffer_rx_X[0];
        gyroY = (buffer_rx_Y[1] << 8) | buffer_rx_Y[0];
        gyroZ = (buffer_rx_Z[1] << 8) | buffer_rx_Z[0];
}

void l3g4200d_gyro_burst(void)
{
	msg_t status = RDY_OK;
        uint8_t buffer_tx;
	uint8_t buffer_txx[2];
	uint8_t status_reg;
	uint8_t buffer_rx[6];
        systime_t tmo = MS2ST(4);

        i2cAcquireBus(&I2CD1);

	buffer_txx[0] = FIFO_CTRL_REG;
        buffer_txx[1] = 0x20;
        status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,buffer_txx,2,NULL,0,tmo);
        if (status != RDY_OK){
                g_errors[0] = i2cGetErrors(&I2CD1);}

	buffer_tx = 0xA8;
        status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,&buffer_tx,1,buffer_rx,6,tmo);
        if (status != RDY_OK){
                 g_errors[1] = i2cGetErrors(&I2CD1);}

	buffer_txx[0] = FIFO_CTRL_REG;
        buffer_txx[1] = 0x00;
        status = i2cMasterTransmitTimeout(&I2CD1,L3G_ADDR,buffer_txx,2,NULL,0,tmo);
        if (status != RDY_OK){
                g_errors[2] = i2cGetErrors(&I2CD1);}
	
       	gyroX = (int16_t)((buffer_rx[0] << 8) | buffer_rx[1]);
        gyroY = (int16_t)((buffer_rx[2] << 8) | buffer_rx[3]);
        gyroZ = (int16_t)((buffer_rx[4] << 8) | buffer_rx[5]);
	
	i2cReleaseBus(&I2CD1);
}
