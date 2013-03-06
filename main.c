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

#include <stdio.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "test.h"

#include "usb_cdc.h"
#include "shell.h"
#include "chprintf.h"
#include "bmp085.h"
#include "lsm303dlh.h"
#include "l3g4200d.h"
#include "i2c_setup.h"
#include "filter.h"

#define DEBUG
//#define DEBUG2


//================================
// Sensors Values
// ================================
extern int32_t temperature, pressure;
extern int16_t accelX, accelY, accelZ;
extern int16_t gyroX, gyroY, gyroZ;
extern int16_t magnX, magnY, magnZ;
extern float   Roll, Pitch, Yaw;
extern float   x_acc,y_acc,z_acc,x_gyro,y_gyro,z_gyro;
//==================================

int8_t cRoll[2],cPitch[2],cYaw[2];
int8_t cRPY[6];
int16_t cRPY_16bit[5];
int16_t intRoll,intPitch,intYaw;
int16_t rxbuf_16bit[5];

static Mutex mtx_bmp, mtx_lsm, mtx_l3g, mtx_filter;


//=====SPI BUFFERS=========

static uint8_t txbuf[5] = {1,56,156,201,124};
static uint8_t rxbuf;


/*===========================================================================*/
/* USB related stuff.                                                        */
/*===========================================================================*/

/*
 * USB Driver structure.
 */
static SerialUSBDriver SDU1;

/*
 * USB Device Descriptor.
 */
static const uint8_t vcom_device_descriptor_data[18] = {
  USB_DESC_DEVICE       (0x0110,        /* bcdUSB (1.1).                    */
                         0x02,          /* bDeviceClass (CDC).              */
                         0x00,          /* bDeviceSubClass.                 */
                         0x00,          /* bDeviceProtocol.                 */
                         0x40,          /* bMaxPacketSize.                  */
                         0x0483,        /* idVendor (ST).                   */
                         0x5740,        /* idProduct.                       */
                         0x0200,        /* bcdDevice.                       */
                         1,             /* iManufacturer.                   */
                         2,             /* iProduct.                        */
                         3,             /* iSerialNumber.                   */
                         1)             /* bNumConfigurations.              */
};

/*
 * Device Descriptor wrapper.
 */
static const USBDescriptor vcom_device_descriptor = {
  sizeof vcom_device_descriptor_data,
  vcom_device_descriptor_data
};

/* Configuration Descriptor tree for a CDC.*/
static const uint8_t vcom_configuration_descriptor_data[67] = {
  /* Configuration Descriptor.*/
  USB_DESC_CONFIGURATION(67,            /* wTotalLength.                    */
                         0x02,          /* bNumInterfaces.                  */
                         0x01,          /* bConfigurationValue.             */
                         0,             /* iConfiguration.                  */
                         0xC0,          /* bmAttributes (self powered).     */
                         50),           /* bMaxPower (100mA).               */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x00,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x01,          /* bNumEndpoints.                   */
                         0x02,          /* bInterfaceClass (Communications
                                           Interface Class, CDC section
                                           4.2).                            */
                         0x02,          /* bInterfaceSubClass (Abstract
                                         Control Model, CDC section 4.3).   */
                         0x01,          /* bInterfaceProtocol (AT commands,
                                           CDC section 4.4).                */
                         0),            /* iInterface.                      */
  /* Header Functional Descriptor (CDC section 5.2.3).*/
  USB_DESC_BYTE         (5),            /* bLength.                         */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x00),         /* bDescriptorSubtype (Header
                                           Functional Descriptor.           */
  USB_DESC_BCD          (0x0110),       /* bcdCDC.                          */
  /* Call Management Functional Descriptor. */
  USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x01),         /* bDescriptorSubtype (Call Management
                                           Functional Descriptor).          */
  USB_DESC_BYTE         (0x00),         /* bmCapabilities (D0+D1).          */
  USB_DESC_BYTE         (0x01),         /* bDataInterface.                  */
  /* ACM Functional Descriptor.*/
  USB_DESC_BYTE         (4),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x02),         /* bDescriptorSubtype (Abstract
                                           Control Management Descriptor).  */
  USB_DESC_BYTE         (0x02),         /* bmCapabilities.                  */
  /* Union Functional Descriptor.*/
  USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x06),         /* bDescriptorSubtype (Union
                                           Functional Descriptor).          */
  USB_DESC_BYTE         (0x00),         /* bMasterInterface (Communication
                                           Class Interface).                */
  USB_DESC_BYTE         (0x01),         /* bSlaveInterface0 (Data Class
                                           Interface).                      */
  /* Endpoint 2 Descriptor.*/
  USB_DESC_ENDPOINT     (USB_CDC_INTERRUPT_REQUEST_EP|0x80,
                         0x03,          /* bmAttributes (Interrupt).        */
                         0x0008,        /* wMaxPacketSize.                  */
                         0xFF),         /* bInterval.                       */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x01,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x02,          /* bNumEndpoints.                   */
                         0x0A,          /* bInterfaceClass (Data Class
                                           Interface, CDC section 4.5).     */
                         0x00,          /* bInterfaceSubClass (CDC section
                                           4.6).                            */
                         0x00,          /* bInterfaceProtocol (CDC section
                                           4.7).                            */
                         0x00),         /* iInterface.                      */
  /* Endpoint 3 Descriptor.*/
  USB_DESC_ENDPOINT     (USB_CDC_DATA_AVAILABLE_EP,     /* bEndpointAddress.*/
                         0x02,          /* bmAttributes (Bulk).             */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x00),         /* bInterval.                       */
  /* Endpoint 1 Descriptor.*/
  USB_DESC_ENDPOINT     (USB_CDC_DATA_REQUEST_EP|0x80,  /* bEndpointAddress.*/
                         0x02,          /* bmAttributes (Bulk).             */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x00)          /* bInterval.                       */
};

/*
 * Configuration Descriptor wrapper.
 */
static const USBDescriptor vcom_configuration_descriptor = {
  sizeof vcom_configuration_descriptor_data,
  vcom_configuration_descriptor_data
};

/*
 * U.S. English language identifier.
 */
static const uint8_t vcom_string0[] = {
  USB_DESC_BYTE(4),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  USB_DESC_WORD(0x0409)                 /* wLANGID (U.S. English).          */
};

/*
 * Vendor string.
 */
static const uint8_t vcom_string1[] = {
  USB_DESC_BYTE(38),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'S', 0, 'T', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'e', 0,
  'l', 0, 'e', 0, 'c', 0, 't', 0, 'r', 0, 'o', 0, 'n', 0, 'i', 0,
  'c', 0, 's', 0
};

/*
 * Device Description string.
 */
static const uint8_t vcom_string2[] = {
  USB_DESC_BYTE(56),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'C', 0, 'h', 0, 'i', 0, 'b', 0, 'i', 0, 'O', 0, 'S', 0, '/', 0,
  'R', 0, 'T', 0, ' ', 0, 'V', 0, 'i', 0, 'r', 0, 't', 0, 'u', 0,
  'a', 0, 'l', 0, ' ', 0, 'C', 0, 'O', 0, 'M', 0, ' ', 0, 'P', 0,
  'o', 0, 'r', 0, 't', 0
};

/*
 * Serial Number string.
 */
static const uint8_t vcom_string3[] = {
  USB_DESC_BYTE(8),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  '0' + CH_KERNEL_MAJOR, 0,
  '0' + CH_KERNEL_MINOR, 0,
  '0' + CH_KERNEL_PATCH, 0
};

/*
 * Strings wrappers array.
 */
static const USBDescriptor vcom_strings[] = {
  {sizeof vcom_string0, vcom_string0},
  {sizeof vcom_string1, vcom_string1},
  {sizeof vcom_string2, vcom_string2},
  {sizeof vcom_string3, vcom_string3}
};

/*
 * Handles the GET_DESCRIPTOR callback. All required descriptors must be
 * handled here.
 */
static const USBDescriptor *get_descriptor(USBDriver *usbp,
                                           uint8_t dtype,
                                           uint8_t dindex,
                                           uint16_t lang) {

  (void)usbp;
  (void)lang;
  switch (dtype) {
  case USB_DESCRIPTOR_DEVICE:
    return &vcom_device_descriptor;
  case USB_DESCRIPTOR_CONFIGURATION:
    return &vcom_configuration_descriptor;
  case USB_DESCRIPTOR_STRING:
    if (dindex < 4)
      return &vcom_strings[dindex];
  }
  return NULL;
}

/**
 * @brief   EP1 initialization structure (IN only).
 */
static const USBEndpointConfig ep1config = {
  USB_EP_MODE_TYPE_BULK | USB_EP_MODE_PACKET,
  NULL,
  sduDataTransmitted,
  NULL,
  0x0040,
  0x0000,
  NULL,
  NULL
};

/**
 * @brief   EP2 initialization structure (IN only).
 */
static const USBEndpointConfig ep2config = {
  USB_EP_MODE_TYPE_INTR | USB_EP_MODE_PACKET,
  NULL,
  sduInterruptTransmitted,
  NULL,
  0x0010,
  0x0000,
  NULL,
  NULL
};

/**
 * @brief   EP3 initialization structure (OUT only).
 */
static const USBEndpointConfig ep3config = {
  USB_EP_MODE_TYPE_BULK | USB_EP_MODE_PACKET,
  NULL,
  NULL,
  sduDataReceived,
  0x0000,
  0x0040,
  NULL,
  NULL
};

/*
 * Handles the USB driver global events.
 */
static void usb_event(USBDriver *usbp, usbevent_t event) {

  switch (event) {
  case USB_EVENT_RESET:
    return;
  case USB_EVENT_ADDRESS:
    return;
  case USB_EVENT_CONFIGURED:
    /* Enables the endpoints specified into the configuration.
       Note, this callback is invoked from an ISR so I-Class functions
       must be used.*/
    chSysLockFromIsr();
    usbInitEndpointI(usbp, USB_CDC_DATA_REQUEST_EP, &ep1config);
    usbInitEndpointI(usbp, USB_CDC_INTERRUPT_REQUEST_EP, &ep2config);
    usbInitEndpointI(usbp, USB_CDC_DATA_AVAILABLE_EP, &ep3config);
    chSysUnlockFromIsr();
    return;
  case USB_EVENT_SUSPEND:
    return;
  case USB_EVENT_WAKEUP:
    return;
  case USB_EVENT_STALLED:
    return;
  }
  return;
}

/*
 * Serial over USB driver configuration.
 */
static const SerialUSBConfig serusbcfg = {
  &USBD1,
  {
    usb_event,
    get_descriptor,
    sduRequestsHook,
    NULL
  }
};


static const SPIConfig hs_spicfg = {
  NULL,
  GPIOB,
  GPIOB_SPI2NSS,
  0
};

static const SPIConfig ls_spicfg = {
  NULL,
  GPIOB,
  GPIOB_SPI2NSS,
  SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0
};



static const ShellCommand commands[] = {
 // {"mem", cmd_mem},
 // {"threads", cmd_threads},
 // {"test", cmd_test},
 // {"read-data", cmd_read_data},
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseChannel *)&SDU1,
  commands
};

/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

/*
 * Red LED blinker thread, times are in milliseconds.
 */

static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (TRUE) {
    palClearPad(IOPORT3, GPIOC_LED);
    chThdSleepMilliseconds(500);
    palSetPad(IOPORT3, GPIOC_LED);
    chThdSleepMilliseconds(500);
  }
  return 0;
}

static WORKING_AREA(waFilterThread, 128);
static msg_t FilterThread(void *arg) {

  int8_t spitemp = 0b01010101;
  (void)arg;
  chRegSetThreadName("Filter");
	

  // State initialization
  Roll = 0.0;
  Pitch = 0.0;
  Yaw = 0.0;


  while (TRUE) {
	  chMtxLock(&mtx_filter);
	
	  complementary_filter();
	  
	  intRoll = (int16_t)(Roll*1000);
	  intPitch = (int16_t)(Pitch*1000);
	  intYaw = (int16_t)(Yaw*1000);

	  /* This part is necessary for 8BIT SPI MODE	   
	 
	  cRPY[1] = tempRoll & 255;              //LSB
	  cRPY[0] = tempRoll >> 8;               //MSB
	                            
	  cRPY[3] = tempPitch & 255;            //LSB
	  cRPY[2] = tempPitch >> 8;             //MSB
	                                                    
	  cRPY[5] = tempYaw & 255;                //LSB
	  cRPY[4] = tempYaw >> 8;                 //MSB
	  
	  */
	
	  cRPY_16bit[0] = 128;
	  cRPY_16bit[1] = intRoll;
	  cRPY_16bit[2] = intPitch;
	  cRPY_16bit[3] = intYaw;
	  cRPY_16bit[4] = 128;
	  	  
	  spiAcquireBus(&SPID2);              				/* Acquire ownership of the bus.    */
	  spiStart(&SPID2, &ls_spicfg);       				/* Setup transfer parameters.       */
	  spiSelect(&SPID2);                  				/* Slave Select assertion.          */
	  spiExchange(&SPID2,10, cRPY_16bit, rxbuf_16bit);       	/* Atomic transfer operations.      */
	  //spiSend(&SPID2,6,cRPY);
	  spiUnselect(&SPID2);                				/* Slave Select de-assertion.       */
	  spiReleaseBus(&SPID2);              				/* Ownership release.               */

	  chThdSleepMilliseconds(25);
	  chMtxUnlock();
  }
  return 0;
}

// Bmp085 Pressure/Temperature Thread
static WORKING_AREA(PollBmp085ThreadWA, 256);
static msg_t PollBmp085Thread(void *arg) {
  chRegSetThreadName("PollBmp085");
  (void)arg;
  while (TRUE) {
    /*chThdSleepMilliseconds(rand() & 31);*/
    chThdSleepMilliseconds(100);
    /* Call reading function */
    chMtxLock(&mtx_bmp);
    bmp085_read_temp();
    bmp085_read_press(BMP_MODE_PR0);
    chMtxUnlock();
  }
  return 0;
}

// LSM303DLH Accelerometer/Magnetometer Thread
static WORKING_AREA(PollLsm303dlhThreadWA, 256);
static msg_t PollLsm303dlhThread(void *arg) {
  chRegSetThreadName("PollLsm303dlh");
  (void)arg;
  while (TRUE) {
    /*chThdSleepMilliseconds(rand() & 31);*/
    chThdSleepMilliseconds(20);
    /* Call reading function */
    chMtxLock(&mtx_lsm);
    lsm303dlh_read_acceleration();
    lsm303dlh_read_magfield();
    chMtxUnlock();
  }
  return 0;
}

// L3G4200D Gyroscope Thread
static WORKING_AREA(PollL3g4200dThreadWA, 256);
static msg_t PollL3g4200dThread(void *arg) {
  chRegSetThreadName("PollL3g4200d");
  (void)arg;
  while (TRUE) {
    /*chThdSleepMilliseconds(rand() & 31);*/
    chThdSleepMilliseconds(20);
    /* Call reading function */
    chMtxLock(&mtx_l3g);
    l3g4200d_gyro_burst();
    chMtxUnlock();
  }
  return 0;
}


/*
 * Application entry point.
 */
int main(void) {
  Thread *shelltp = NULL;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  i2c_setup();
  bmp085_init();
  lsm303dlh_init();
  l3g4200d_init();

  palSetPadMode(IOPORT2, 13, PAL_MODE_STM32_ALTERNATE_PUSHPULL);     /* SCK. */
  palSetPadMode(IOPORT2, 14, PAL_MODE_STM32_ALTERNATE_PUSHPULL);     /* MISO.*/
  palSetPadMode(IOPORT2, 15, PAL_MODE_STM32_ALTERNATE_PUSHPULL);     /* MOSI.*/
  palSetPadMode(IOPORT2, GPIOB_SPI2NSS, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPad(IOPORT2, GPIOB_SPI2NSS);


  chMtxInit(&mtx_bmp);
  chMtxInit(&mtx_lsm);
  chMtxInit(&mtx_l3g);
  chMtxInit(&mtx_filter);


  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);
  usbConnectBus(serusbcfg.usbp);
  //palClearPad(GPIOC, GPIOC_USB_DISC);

  chThdSleepSeconds(3);


  /*
   * Shell manager initialization.
   */

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
  chThdCreateStatic(waFilterThread, sizeof(waFilterThread), NORMALPRIO, FilterThread, NULL);

  chThdCreateStatic(PollBmp085ThreadWA,
          sizeof(PollBmp085ThreadWA),
          NORMALPRIO,
          PollBmp085Thread,
          NULL);
  
  chThdCreateStatic(PollLsm303dlhThreadWA,
        sizeof(PollLsm303dlhThreadWA),
         NORMALPRIO,
         PollLsm303dlhThread,
         NULL);

  chThdCreateStatic(PollL3g4200dThreadWA,
         sizeof(PollL3g4200dThreadWA),
         NORMALPRIO,
         PollL3g4200dThread,
         NULL);



  
  while (TRUE) {
  
    //chprintf((BaseChannel *)&SDU1,"3\r\n");   
    //chprintf((BaseChannel *)&SDU1, "%f:%f:%f:%f:%f:%f\r\n",x_acc,y_acc,z_acc,x_gyro,y_gyro,z_gyro);
    
    chprintf((BaseChannel *)&SDU1, "S:%d:%d:%d:%d:%d:%d:%d:%d:%d:E\r\n",gyroX,gyroY,gyroZ,accelX,accelY,accelZ,magnX,magnY,magnZ);

    chThdSleepMilliseconds(100);
  }
}