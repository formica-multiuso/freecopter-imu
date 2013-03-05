#include <ch.h>
#include <hal.h>


#include "i2c_setup.h"


static const I2CConfig i2cfg1 = {
    OPMODE_I2C,
    100000,
    STD_DUTY_CYCLE,
};



void i2c_setup(void){
  i2cInit();

  i2cStart(&I2CD1, &i2cfg1);

  /* tune ports for I2C1*/
  palSetPadMode(IOPORT2, 6, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
  palSetPadMode(IOPORT2, 7, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);

  /* startups. Pauses added just to be safe */
  chThdSleepMilliseconds(100);
}

