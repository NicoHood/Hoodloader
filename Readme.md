Arduino Hoodloader BETA
==================

**There might be some problems with uploading on some computers or other errors. Try v1.5 instead!***

This is the source page of the Hoodloader. The Hoodloader is a fully compatible replacement of the
normal 16u2 usbserial Bootloader. It can still work the same but has more functions.

**Functions:**
* Program the Arduino Uno/Mega like you are used to
* Serial interface is still usable!
* Use your Arduino as HID device
* Use the 16u2 as ISP to reprogram your 328 and others
* One Firmware for Uno/Mega

**Tested Devices:**
* Arduino Uno R3
* Arduino Mega R3

**Limitations:**
* HID only works at baud 115200 (Software and speed limitation, see "how it works below")
* All other bauds work 100%. If you still try to use HID with other baud you will get weird output.
* 16u2 as ISP has the same functions/bugs like the normal Arduino as ISP. Use IDE 1.5.7!
* 8u2 has no ISP function (use Lite version)
* No more Midi/other stuff is possible due to DRAM, RAM and endpoint limitation

**Things that might change in the future:**
* HID deactivation function (re)move to other pin?
* SS Pin for ISP
* Programming baud for ISP

See http://nicohood.wordpress.com/ for more tutorials and projects

Installation on Arduino Uno/Mega R3
===================================
**For the whole Project IDE 1.5.7 is recommended!**

**This tutorial is for Windows and R3 versions only.**
This method is called “Device Firmware Upgrade” and also works on Linux/Mac and older versions but its not explained in this article.
Maybe [this page](http://arduino.cc/en/Hacking/Upgrading16U2Due) can help you.
If you have a 8u2 see instructions below the DFU tutorial.

**What you need: Arduino Uno/Mega, USB Cable, a normal wire, [Flip with Java](http://www.atmel.com/tools/flip.aspx)**

Flashing new Firmware with DFU and Flip
---------------------------------------

To **install the new bootloader** connect your Arduino to your PC and put it into DFU mode.
**You can always switch back to the original firmware, nothing to break.**

Install [Flip with Java](http://www.atmel.com/tools/flip.aspx)** first.
This will also install the needed DFU drivers to flash the new firmware. Start Flip.

**_Briefly_ short these two pins** of the 16u2 with a Wire **(the jumper is only to show the connection)**.
If you have an older Version than the R3 please google how to get in DFU mode.
Today everybody should have an R3 so no resistor and complicated stuff is needed.
![pictures/DFU_Bridge.jpg](pictures/DFU_Bridge.jpg)

The device should now show up as Atmega16u2 in the device manager.
If not or if you get Error: **“AtLibUsbDfu.dll not found”** install the drivers manually from the device manager.
Right click the unknown device and select the Flip installation path to search the drivers.
C:\Program Files (x86)\Atmel\Flip 3.4.7\usb rightclick the .inf file and hit install!

Click the IC Button an select **Atmega16u2. (same for Uno/Mega).**

![pictures/Flip_1.png](pictures/Flip_1.png)

**Click File->Load Hex File** and select the Firmware in Firmwares/Hoodloader1_x.hex (choose newest version).
**Its the same file for Uno and Mega.**

![pictures/Flip_2.png](pictures/Flip_2.png)

Click the USB Cable and **click open.**

![pictures/Flip_2.png](pictures/Flip_2.png)

**Click run** to upload the firmware. Uncheck Reset and click Start Application to restart your Arduino. Or just **replug the cable.**
Read further on how to install the drivers for Windows.

Installation on a 8u2
---------------------
The 8u2 has less flash so you are not able to flash the normal Hoodloader.
Instead there is a Lite Version in Firmwares/LegacyVersions/
You need to flash this with an ISP because it will overwrite DFU.
The DFU Bootloader takes 2 or 4kb (not sure) and the hex file is >6kb anyways.
This is only for advanced useres with older chips.
HID is fully functional, ISP wont work.

CDC Driver installation
-----------------------
**You need to install new drivers for the new device on Windows (Linux, Mac not needed).** Actually they are not new, its just an .inf file that tells
Windows to use its built in CDC Serial driver. Ironically Microsoft never signed its own driver.
The drivers are located in Firmwares/Hoodloader.inf.
Also see [this tutorial](http://arduino.cc/en/guide/windows) on how to install the drivers (rightclick the .inf file and hit install).
[How to install drivers for Windows 8/8.1](https://learn.sparkfun.com/tutorials/disabling-driver-signature-on-windows-8/disabling-signed-driver-enforcement-on-windows-8).
If you want it to be recognized as Uno/Mega edit the makefile and recompile. I dont recommend this to know what
Bootloader currently is on your Board.

You are done! Have fun with your new Arduino Firmware.

HID usage
=========
See the [HID project](https://github.com/NicoHood/HID)
for changelog, bugs, installing instructions and more information of the main feature.

Deactivate HID function
=======================
**This feature might be (re)moved in further releases due to the ISP usage.**

Its possible to deactivate HID if you messed up something in the code and cannot return easily.

**Do NOT use this shorting if you use the 16u2 as ISP! This will damage your 16u2.
You need to replug/reprogram/open the Serial to use the bridge again.
16u2 as ISP sets the pin to OUTPUT which will kill the pin if shorted to ground!**

Just **short these two pins permanently** until you have uploaded your new, working sketch:

![pictures/No_HID_Bridge.jpg](pictures/No_HID_Bridge.jpg)

16u2 as ISP usage
=================
The Hoodloader includes a port of the Arduino as ISP sketch. Some minor code style changes were made but overall it works the same.
No need to flash the main processor.

**This is still under development!**

Copy the folder (16u2asISP) inside hardware/ into sketchbook/hardware/ like this:
```
Arduino/sketchbook/hardware/16u2asISP/avr/boards.txt
Arduino/sketchbook/hardware/16u2asISP/avr/programmers.txt
```

**SS pin is currently on the 4 pin header that normally isnt soldered. It is the pin on the bottom left (closest to the TX led).**
Pins we could use for SS are: RESET, 0, 1, all 4 pins of the extra header. I would not recommend to use pin 0 or 1 to not burn or interfere with the main chip.
Reset would also reset the main chip, the 4 pinheader needs to be soldered. I could also create two versions with two different bauds.
![pictures/ISP_Pins.jpg](pictures/ISP_Pins.jpg)

At the moment I use baud 1 to upload sketches. This is to not interfere with any other baud.
This gives us most compatibility. Under Windows this works, tell me about other systems.
Otherwise I have to use 300 or something similar.

**Use IDE 1.5.7** for 16u2 as ISP. It fixes a verification bug for burning the Mega bootloader.

**Uploading Mega2560 sketches is broken** due to fuse settings. Contact me for any help. See this thread:
http://forum.arduino.cc/index.php?topic=126160.0

TX Led is for status, RX Led for errors. **Please report me if the RX Led starts blinking and wont stop!**
Please also report me other errors related to ISP.

**Do NOT use the HID deactivation if you use the 16u2 as ISP! This will damage your 16u2.
You need to replug/reprogram/open the Serial to use the bridge again.
16u2 as ISP sets the pin to OUTPUT which will kill the pin if shorted to ground!
I might remove this feature(or move to the 4 pin header), HID seems stable so far.**

Wiring: Connect all lines together like this:
```
16u2 - Chip being programmed
5V - 5V
GND - GND
MOSI - MOSI
MISO - MISO
SCK - SCK
SS Pin - RESET
```
![pictures/ProgrammingNano.jpg](pictures/ProgrammingNano.jpg)
If you are programming the same board only MOSI, MISO, SCK, SS is needed to connect.
![pictures/ProgrammingMega.jpg](pictures/ProgrammingMega.jpg)

See Google code discussion:
https://groups.google.com/a/arduino.cc/forum/#!msg/developers/V_T-Uvj8hSs/h9xlGyM9cJoJ

How it works
============
To sum it up: Serial information is grabbed by the "man in the middle, 16u2" and you dont have to worry to get any wrong Serial stuff via USB.

For the Uno/Mega you need a special Bootloader. Why? Because the Uno/Mega has 2 chips on board.
The 328/2560 and 16u2 on each. And the only communication between the 16u2 and the main chip is via Serial.
But the Serial is also used to program the chip. So what I do here is to filter out all Serial Data that comes in
via the NicoHoodProtocol (NHP). There is an indicator address 1 which contains the beginning and the Report ID.
If the following Serial information is Address 2 with a valid checksum the report will be created and sent if its
finished successful. If any error occurred within the first 2 Protocol Addresses the information will be sent via Serial.
The Program should forward this information because it could be a normal information. Everything above 2 Addresses that goes
wrong wont be sent and discarded due to a normal wrong HID report. Normally you dont have to worry about getting weird HID
presses. You need to send exactly 6 bytes with the special Numbers and another 6 bytes for the first information with checksum
and complete the full report. You might get weird Serial output if you hit the exact 12 bytes without timeout of a few milliseconds.
And if the reading timed out the Data will also be forwarded. And if you only send Ascii Code the Information is forwarded instantly
because the NHP filters that out instantly (see documentation of the NHP). So filtering should be fine and dont block :)

Just an excerpt (better explanation soon):
```
so There are 3 modes: Default (usb to serial), HID, and ISP
to distinguish between ISP and the others there is a simple way of doing it: The CDC Virtual Serial interface can have different baud rates.
and the MCU know this baud, because it needs to reprogram the chip. So what i did was to use an invalid baud (1) to say "hey, its programming mode"
this will turn off usbtoserial and HID. this will set the ram up for ISP mode. this will deactivate any incoming Serial byte (discard)
to with baud 1 iso mode is called any every received byte is translated as stk500v1 command
thats the ISP part
the HID - DEFAULT part is also easy
HID only works at baud 0 and baud 115200
baud 0 is when the MCU started, so no baud was set with the pc
baud 115200 is set after programming. thatswhy i had to use 115200. luckly its the fastest baud, for faster reports.
so its the best thing that could happen ;)
if its not baud 0, 115200 or the HID is deactivated (which might get removed) it will be in normal mode
it will translater usb to serial and serial to usb byte by byte
HID mode is a bit different. PC->328 for example works the same
but 328->PC is filtered. the 16u2 listens for NHP (Nicohoodprotocol) commands with a special pattern and filters this out.
therefore i buff every possible command. If its valid it will send the HID report. this is very save, its very unlikly
that you will ever produce a valid HID report with random data. also normal ASCII is always filtered out, due to protocol specification
and if a possible valid protocol times out the data is also send to the pc to not keep the last bytes
```

The 16u2 as ISP works like the Arduino sketch. It just uses its won SPI header to program the device
and uses a special baud to not get into conflict with other commonly used bauds.
In ISP mode no data is transferred between Arduino main chip and Pc. This also wouldnt be possible because of shared ram.

This library wouldnt be possible without
========================================

* [Lufa 140302 from Dean Camera](http://www.fourwalledcubicle.com/LUFA.php)
* [Darran's HID Projects] (https://github.com/harlequin-tech/arduino-usb)
* [Connor's Joystick for the Leonardo](http://www.imaginaryindustries.com/blog/?p=80)
* [Stefan Jones Multimedia Keys Example](http://stefanjones.ca/blog/arduino-leonardo-remote-multimedia-keys/)
* [Athanasios Douitsis Multimedia Keys Example](https://github.com/aduitsis/ardumultimedia)
* [The Original Arduino Sources](https://github.com/arduino/Arduino/tree/master/hardware/arduino/firmwares/atmegaxxu2/arduino-usbserial)
* [USBlyzer](http://www.usblyzer.com/)
* [Arduino as ISP](https://github.com/rsbohn/ArduinoISP)
* A lot of searching through the web
* The awesome official Arduino IRC chat!
* [The NicoHood Protocol ^.^](https://github.com/NicoHood/NicoHoodProtocol)
* For donations please contact me on my blog :)

Known Bugs
==========
See the [HID project](https://github.com/NicoHood/HID) for HID related bugs.

**Programming Arduino Mega with ISP doesnt work because of fuses.**
See this for more information: http://forum.arduino.cc/index.php?topic=126160.0

Burning Bootloader error is fixed with IDE 1.5.7 or higher (avrdude bug)!

Using the Hoodloader on a 8u2 you need to use the Lite Version and DFU wont work.

Feel free to open an Issue on Github if you find a bug. Or message me via my [blog](http://nicohood.wordpress.com/)!

Ideas for the future:
=====================
* bug: programmed via ISP, then use HID wont work. -> dont block serial 100%, just dont use a ringbuffer??
* Save if HID report!=0 and release after every restart
* Design new Arduino with I2C and SPI connection
* Pmode timeout!
* general timeout function for hid and other
* Different h/c files for better overview!
* remove older debug/TODO stuff
* explain how it works better
* deactivate HID for the first seconds?
* release keys on reprogramming to not hold any key
* 16u2 as arduino with special bootloader

Version History
===============
```
1.7.3 Beta Release (10.08.2014)
* Fixed HID flush bug (1.6 - 1.7.2)

1.7.2 Beta Release (10.08.2014)
* Added Lite version for 8u2
* Added Versions that show up as Uno/Mega (not recommended)
* Makefile and structure changes

1.7.1 Beta Release (10.08.2014)
* Fixed HID deactivation bug

1.7 Beta Release (10.08.2014)
* Works as ISP now.
* Exceeded 8kb limit. For flashing a 8u2 use Lite Version please!
* Changed Readme text

1.6 Beta Release (09.08.2014)
* Changed HID management (not blocking that much, faster)
* added RawHID in/out (HID to Serial)

1.5 Beta Release (21.07.2014)
* Moved Hoodloader source from [HID project](https://github.com/NicoHood/HID) here
* Overall a lot of ram improvements, now with a big global union of ram
* Removed USBtoUSART buffer (not needed, saved 128/500 bytes)
* Removed Lite version because of better ram usage not needed
* Separated different modes better to not cause any errors in default mode
* Improved the deactivate option
* Integrated NHP directly
* Replaced LightweightRingbuffer with native Lufa Ringbuffer
* Improved writing to CDC Host
* Fixed a bug in checkNHPProtocol: & needs to be a ==
* General structure changes
* Improved stability
* Fixed Arduino as ISP bug (same as in v1.4.2)

1.4.2 Beta Release (10.07.2014)
* Fix for Arduino as ISP

1.4 Beta Release (10.07.2014)
* Added Lite Version with less ram usage
* Changed PIDs, edited driver file
* added Tutorials

1.3 Beta Release (01.07.2014)
* Improved ram usage (you can get even better but that messes up code and increases flash)

1.2 Beta Release (22.06.2014)
* Sometimes HID Devices weren't updating when using more than 1 Device (set forcewrite to true)
* Fast updates crashed the bootloader (too much ram usage, set CDC buffer from 128b to 100b each)
* Minor file structure changes

1.1 Beta Release (05.06.2014)
* Minor fixes

1.0 Beta Release (03.06.2014)
```

For Developers
==============
If you deactivate some reports it can occur that windows will cause problems and recognize it as different device.
While developing I had that much trouble that I had to change the PID. No way to repair the broken windows driver settings.
So be careful if you change the source on your own with important PIDs.
Therefore reinstall the divers for any device or just dont touch the HID reports in the Bootloader.
The Leonardo/Micro version worked fine till now.

See this how to uninstall the drivers (not tested):
https://support.microsoft.com/kb/315539

The Hoodloader was coded with Windows7/8.1 + Visual Studio Express and compiled with a Raspberry Pi.
Lufa version 140302 is included!

```
#Vendor ID from lufa         0x03EB
#Product ID created my own
#HOODLOADER                  0x6E68
#HOODLOADER-Lite             0x4E48

#You can also use the native Arduino VID and PID
#VendorID  Arduino           0x2341
#ProductID Arduino Uno       0x0001, 0x0043 R3
#ProductID Arduino Mega2560  0x0010, 0x0042 R3
#ProductID Arduino Mega ADK  0x003F, 0x0044 R3

#define RAWHID_USAGE_PAGE	0xFFC0 // recommended: 0xFF00 to 0xFFFF
#define RAWHID_USAGE		0x0C00 // recommended: 0x0100 to 0xFFFF
#define RAWHID_TX_SIZE 63 // 1 byte for report ID
#define RAWHID_RX_SIZE 63 // 1 byte for report ID
```

How to compile (on a Raspberry Pi)
==================================
This instruction is for a Raspberry Pi with Debian. I cannot tell you how to compile Lufa with any other device, but Google can.

I recommend creating a Windows share by right clicking your Arduino folder and hit "share".
Then you can edit the files on Windows with your default code editor and connect to your pi via ssh.
``` bash
$ cd Desktop
$ mkdir Arduino

#Test mounting:
sudo mount -t cifs -o username=yourusername,password=yourpass,nounix,noserverino //YOUR-PC-NAME/Arduino Arduino

#run it automated at startup. If not connected this will cause a long timeout.
sudo nano /etc/fstab
//YOUR-PC-NAME/Arduino /home/pi/Desktop/Arduino cifs username=yourusername,password=yourpass,nounix,noserverino 0 0
```

You need to install the gcc-avr toolchain, avr-libc and compile with:
``` bash
$ sudo apt-get install gcc-avr avr-libc
$ cd Desktop/Arduino/Hoodloader
$ sudo make clean
$ sudo make
```

Licence and Copyright
=====================
If you use this library for any cool project let me know!

```
Copyright (c) 2014 NicoHood
See the readme for credit to other people.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
```
