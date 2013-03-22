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

#include "lsm303dlh.h"

int16_t accelX, accelY, accelZ;
int16_t magnX, magnY, magnZ;

static i2cflags_t errors[6] = {0,0,0,0,0,0};
static i2cflags_t a_errors[3] = {0,0,0};
static i2cflags_t m_errors[3] = {0,0,0};

int lsm303dlh_init(void){

	msg_t status = RDY_OK;
        uint8_t buffer_tx[2];
        systime_t tmo = MS2ST(4);

		
	i2cAcquireBus(&I2CD1);
	// Configuring Accelerometer
	buffer_tx[0] = CTRL_REG1_A;
	buffer_tx[1] = 0x27;	// Normal PM, Datarate:50hz, 3axis enabled
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_ACC,buffer_tx,2,NULL,0,tmo);
	if (status != RDY_OK){
                 errors[0] = i2cGetErrors(&I2CD1);}
	buffer_tx[0] = CTRL_REG2_A;
	buffer_tx[1] = 0x00;	 
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_ACC,buffer_tx,2,NULL,0,tmo);
	if (status != RDY_OK){
                 errors[1] = i2cGetErrors(&I2CD1);}
	buffer_tx[0] = CTRL_REG4_A;
	buffer_tx[1] = 0x40;	//0x00 -> 2g (previous 0x40)
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_ACC,buffer_tx,2,NULL,0,tmo);
	if (status != RDY_OK){
                 errors[2] = i2cGetErrors(&I2CD1);}
	

	// Configuring Magnetometer
	buffer_tx[0] = CRA_REG_M;
	buffer_tx[1] = 0x18;	//75Hz (previous 0x10)
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_MAG,buffer_tx,2,NULL,0,tmo);
	if (status != RDY_OK){
                 errors[3] = i2cGetErrors(&I2CD1);}
	buffer_tx[0] = CRB_REG_M;
	buffer_tx[1] = 0x20;	//1055-950 Gain selection
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_MAG,buffer_tx,2,NULL,0,tmo);
	if (status != RDY_OK){
                 errors[4] = i2cGetErrors(&I2CD1);}
	buffer_tx[0] = MR_REG_M;
	buffer_tx[1] = 0x00;
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_MAG,buffer_tx,2,NULL,0,tmo);
	if (status != RDY_OK){
                 errors[5] = i2cGetErrors(&I2CD1);}
	i2cReleaseBus(&I2CD1);

	return 0;
}

void lsm303dlh_read_acceleration(void)
{
	msg_t status = RDY_OK;
        uint8_t buffer_rx_X[2];
	uint8_t buffer_rx_Y[2];
	uint8_t buffer_rx_Z[2];
	uint8_t buffer_tx;
        systime_t tmo = MS2ST(4);

	i2cAcquireBus(&I2CD1);
	buffer_tx = OUT_X_L_A;
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_ACC,&buffer_tx,1,buffer_rx_X,2,tmo);
	if (status != RDY_OK){
                 a_errors[0] = i2cGetErrors(&I2CD1);}
	//status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_ACC,OUT_X_L_A,1,&buffer_rx[1],1,tmo);
//	if (status != RDY_OK){
//                 a_errors[1] = i2cGetErrors(&I2CD1);}
	buffer_tx = OUT_Y_L_A;
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_ACC,&buffer_tx,1,buffer_rx_Y,2,tmo);
	if (status != RDY_OK){
                 a_errors[1] = i2cGetErrors(&I2CD1);}
//	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_ACC,OUT_Y_L_A,1,&buffer_rx[3],1,tmo);
//	if (status != RDY_OK){
//                 a_errors[3] = i2cGetErrors(&I2CD1);}
	buffer_tx = OUT_Z_L_A;
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_ACC,&buffer_tx,1,buffer_rx_Z,2,tmo);
	if (status != RDY_OK){
                 a_errors[2] = i2cGetErrors(&I2CD1);}
//	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_ACC,OUT_Z_L_A,1,&buffer_rx[5],1,tmo);
//	if (status != RDY_OK){
//                 a_errors[5] = i2cGetErrors(&I2CD1);}
	i2cReleaseBus(&I2CD1);


	accelX = (int16_t) ((buffer_rx_X[0] << 8) | buffer_rx_X[1]);
	accelY = (int16_t) ((buffer_rx_Y[0] << 8) | buffer_rx_Y[1]);
	accelZ = (int16_t) ((buffer_rx_Z[0] << 8) | buffer_rx_Z[1]);
}


void lsm303dlh_read_magfield(void)
{
	uint8_t buffer_rx_X[2];
	uint8_t buffer_rx_Y[2];
	uint8_t buffer_rx_Z[2];
	uint8_t buffer_tx;
 	msg_t status = RDY_OK;
        systime_t tmo = MS2ST(4);

	i2cAcquireBus(&I2CD1);
	buffer_tx=OUT_X_H_M;
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_MAG,&buffer_tx,1,buffer_rx_X,2,tmo);
	if (status != RDY_OK){
                 m_errors[0] = i2cGetErrors(&I2CD1);}
	//status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_MAG,OUT_X_H_M,1,&buffer_rx[1],1,tmo);
	//if (status != RDY_OK){
        //         m_errors[1] = i2cGetErrors(&I2CD1);}
	buffer_tx=OUT_Y_H_M;
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_MAG,&buffer_tx,1,buffer_rx_Y,2,tmo);
	if (status != RDY_OK){
                 m_errors[1] = i2cGetErrors(&I2CD1);}
	//status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_MAG,OUT_Y_H_M,1,&buffer_rx[3],1,tmo);
	//if (status != RDY_OK){
        //         m_errors[3] = i2cGetErrors(&I2CD1);}
        buffer_tx=OUT_Z_H_M;
	status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_MAG,&buffer_tx,1,buffer_rx_Z,2,tmo);
	if (status != RDY_OK){
                 m_errors[2] = i2cGetErrors(&I2CD1);}
	//status = i2cMasterTransmitTimeout(&I2CD1,LSM_ADDR_MAG,OUT_Z_H_M,1,&buffer_rx[5],1,tmo);
	//if (status != RDY_OK){
        //         m_errors[5] = i2cGetErrors(&I2CD1);}
	i2cReleaseBus(&I2CD1);

	magnX = (int16_t) ((buffer_rx_X[0] << 8) | buffer_rx_X[1]);
	magnY = (int16_t) ((buffer_rx_Y[0] << 8) | buffer_rx_Y[1]);
	magnZ = (int16_t) ((buffer_rx_Z[0] << 8) | buffer_rx_Z[1]);

}

