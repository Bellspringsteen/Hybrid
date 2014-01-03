# Software Section
## V2 Parrallel Hybrid Supercapacitor Scooter

# Brief
The software in this projects executes on the PIC microcontroller in the hardware-electronics section. See that section for a list of inputs/outputs. The job of the software is to run the algorithms to control the operation of the Internal Combustion Engine ( ICE), Electric Propulsion Motor, and Supercaps.

# Development Setup
The following items are used to compile and load the PIC.

* Windows Computer or VM 
* CCS C Compiler
* CCS In Circuit Debugger (ICD)

Use CCW to compile C project. You can use the CCS IDE called PCW if you want. 
You can also set the #debug flag to use ICD

# Testing 
Currently there is no automated testing. Enabling #TEST will allow you to isolate and control certain inputs/output on the board. But this tests hardware not software.
