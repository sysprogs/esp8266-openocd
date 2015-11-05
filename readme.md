Sysprogs port of OpenOCD for ESP8266
=================

This repository contains an improved OpenOCD port for ESP8266 based on [this port](https://github.com/projectgus/openocd).
It contains the following improvements over the original port:
* Full support for software breakpoints
* Continuing from breakpoints works correctly
* Stop reasons are reported correctly
* Interrupts can now be disabled during single-stepping via the *xtensa_no_interrupts_during_steps* command
* ESP8266 software watchdog can be auto-fed via the *esp8266_autofeed_watchdog* command
* Stepping over/into functions in FLASH is fixed
* *mon reset halt* command now successfully resets the entire chip using the TRST signal

You can find a detailed tutorial on using this OpenOCD port from Visual Studio [here](http://visualgdb.com/tutorials/esp8266/openocd/)