# SmartUsbTester
## Intro
## Capabilities & Features
Quickcharge:
- 5V, 9V, 12V - sets the voltage output
- analyzes if the PS uses QC
- to use QC set voltage to 5V and press QC UP/DOWN -> +- 1V 

Can control current\
Current, voltage and power measurements:
- 
can control the current consumtion of the device (in order to know how much the powersource can give) \
Can draw real time graphs \
mA
mW
mV
TC - Temperature Celsius
Can test max parameters of a PS \
stops test when:
- reached max current output
- voltage drop below the threshhold (3.9V)
- the board protection is raised (current above 3.2A)
Can test resistance parameters of a cable \
- To run the test successfully connect a diffirent from main PS and input socket with a reference cable (with known resistance (for now it is 228 mOhm))
- take measures using option 1
- to the same PS connect the cable you want to test
- take measures using option 2
Can test capacity of a power source (for powerbanks)
- connect and wait from 1 hr to 4 hrs
 - output in mW/hr
## User guide
To power the device connect the power source to the USB-C port located opposite of a cooling unit. When plugged in, you will see the menu on the screen.
To naviagete the menu rotate the blue encoder next to the screen. To select the option - click. The menu has two pages with seven options total.
#### QC
#### POWER
#### CURRENT
#### GRAPHS
#### MAX PARAMS
#### RESISTANCE
#### CAPACITY
