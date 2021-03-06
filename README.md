freecopter-imu
==============
Author: Roberto Marino<br>
Email: formica@ieee.org, roberto.marino@comupter.org<br>
Last Update: Mar 22 2012 @ 1:00pm<br>

---------------------------

This is the repository containing the firmware of the IMU (Inertial Measurement Unit) developed for the FreeCopter Project. 
The project was financed from the following universities:<br>
-- University of Messina (Dept. of Physics)<br>
-- University of Genova (DIBRIS)<br>
-- Ecole Centrale de Nantes (IRCCyN)<br> 


The project is based on the ChibiOS RealTime Operating System ~ http://www.chibios.org/

You can download it from the official git repo, in a local directory 'chibios-dir', typing as follow:

git clone https://github.com/mabl/ChibiOS 'chibios-dir'

I personally suggest to download a stable version from  SOURCEFORGE <br>
http://sourceforge.net/projects/chibios/

---------------------------

To compile the IMU firmware against the OS code you need a cross-compiler GNU toolchain.
You can use the script of James Snyder to provide it.

git clone https://github.com/jsnyder/arm-eabi-toolchain arm-cs-tools


NOTE: If you want to have some advanced debugging option for the board, like remote debugging via gdb+openocd, you must re-compile the gdb source code (obtained from th jsnyder script) adding the configure option --with-expat=/usr/local/lib/. So, you need a working copy of 'libexpat' in your system.

SUGGESTED: You can use also the precompiled toolchain available at https://launchpad.net/gcc-arm-embedded that is already compiled for remote debugging 

---------------------------

VREP VISUALIZATION

It is possible to visualize the graph of the physical quantities from the IMU using V-REP (http://www.coppeliarobotics.com).
For more infos please read the vrep/README.vrep file!
See a YOUTUBE video of the FreeCopter-imu working with V-REP:

http://www.youtube.com/watch?v=4m2rMd7Fa-o
