# Horizon-OBD2-CAN-Module

Horizon OBD2 CAN Programmable Module 
The Horizon OBD-Interface Module is a freely programmable and universal electronic device, which can be used in the automotive field. It e.g. can be directly attached to OBD2 Standard Interface, supporting CAN-Bus communication but a implementation to another CAN-Bus can also be supported with appropriate wiring harness. 



The module can be programmed and flashed over USB-Interface with Micro-USB connector and standard Arduino IDE will be used for this purpose. The used microcontroller is a Teensy 3.6 with an ARM Cortex M4 kernel. In this documentation, you will also learn on how to organize and install needed ressources to successfully get the device running for you specific applications. The module can be extended with second Bluetooth and Bluetooth LE adapters, that directly can be soldered on the PCB. One Bluetooth Module for Bluetooth 4.0 is already assembled. 

TECHNICAL SPECIFICATIONS OF THE DEVICE 

- DEVELOPED IN YEAR 2019 
- QUIESCENT CURRENT CONSUMPTION :0,8¬µA (@13,5V) 
- ACTIVE MODE CURRENT CONSUMPTION:70mA (@13,5V) 
- DIMENSIONS HOUSING: 110mm x 75mm x 25mm 
- NOMINAL POWER SUPPLY RANGE: +8V...+25V 
- SUPPLY VOLTAGE PEAK: 35V 
- STANDBY-MODE: 10mW 
- NORMAL-MODE: max. 1100mW 
- SINGLE PROCESSOR DESIGN (ARM CORTEX M4 W/ 180Mhz) 
- USB DEBUG- AND LOGGING INTERFACE 
- INTEGRATED FLASH STORAGE W/ READ-OUT PROTECTION 
- Externer Flash Speicher >= 4GB 
- BLUETOOTH4.0 AND BLUETOOTH2.0 SIMULTANOUSLY SUPPORTED 
- DSUB9 CONNECTOR FOR VEHICLE CAN AND POWER SUPPLY 
- SEPARATE KL15 CONTROL FOR AUDIO AND SAFETY REASONS 
- CAN-BUS 11-BIT UND 29-BIT UPTO 1MBIT/S 
- ALL COMPONENTS ARE AUTOMOTIVE COMPLIANT 


Software: 
Software can be written and flashed over Arduino IDE. The device will be connected over USB and can also be debugged by serial interface. 
An example software project is available as a showcase and can be freely changed and adapted. The software program can read out RPM, ETH, LOAD and SPEED from OBD2 interface and provides existing data over USB and Bluetooth 4.0 interface 
