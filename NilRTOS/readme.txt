The base code for NilRTOS was written by Giovanni Di Sirio, the author
of ChibiOS/RT.

http://www.chibios.org/dokuwiki/doku.php

This is an early release of Nil RTOS for AVR Arduinos.

Copy the NilAnalog, NilRTOS, NilTimer1, and TwiMaster folders
to your Arduino/libraries folder.

The data logger examples requires a version of the SdFat
library that is newer than 20130701.  Please use the included
version of SdFat until a newer version is posted on the SdFat
website.

SD logger examples require a quality SD card to avoid overruns.
Adjust the number of ADC channels and the interval between data
points to match the capabilities of your SD card.

Please read NilRTOS.html for more information.