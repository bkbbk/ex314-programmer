# EX-314 Programmer
A Programmer for ICOM EX-314 RAM Board Based on Arduino Uno by BG1TPT and BIE1IH.
Tested on IC-R71E. Should work on ICOM IC-745, IC-751, IC-271, IC-471 and IC-1271.
![image](https://raw.githubusercontent.com/bkbbk/ex314-programmer/main/public/images/EX314_Programmer.JPG)

# Project files
* **adapter** for the gerber files of the adapter PCB.
* **firmware** for the source code of the programmer.

# Usage
1. Connect ICOM EX-314 to the adapter, then plug the adapter on Arduino Uno.
2. Connect serial port on the adapter to computer, if the details of the backup/restore process are needed.
3. Power on the Arduino Uno.
4. Click the button on the adapter to dump the data of EX-314, output to serial port and backup the data to EEPROM simultaneously.
5. Press and hold the button for 1s to restore the data in EEPROM to EX-314.
6. Press and hold the button for 5s to restore the original data in the program to EX-314.
![image](https://raw.githubusercontent.com/bkbbk/ex314-programmer/main/public/images/Screenshot.png)