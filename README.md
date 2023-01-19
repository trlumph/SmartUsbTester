# SmartUsbTester
## Intro
To power the device connect the power source to the main USB-C port located opposite of the cooling unit.  
When plugged in, you will see the menu on the screen.  
To naviagete the menu rotate the blue encoder next to the screen.  
To select the option - click.  
The menu has two pages with seven options total.  
On the bottom of the device there are two input sockets: Type-C and Micro-USB (inside they are connected to the same pins).  
ATTENTION. DO NOT CONNECT INPUT SOCKETS AND THE MAIN SOCKET TO THE SAME POWER SOURCE (PS).
## Capabilities, Features & User guide
#### QC
Quickcharge:
- analyzes if the PS uses QC and what type of QC it is. ? - not connected; WAIT - the test is conducted
- 5V, 9V, 12V - QC 2.0 modes
- to use QC 3.0 set voltage to 5V and press QC UP/DOWN -> +- 1V (5 steps by 200 mV)  
  
[demo](https://youtu.be/108r3PzsNO0)
#### POWER
Current, voltage and power measurements from input sockets. \
[demo](https://youtu.be/_7GYyKfhbiY)
#### CURRENT
Can control current
- uses PID to set load \
can control the current consumtion of the device (in order to know how much the powersource can give)  
  
[demo](https://youtu.be/06G0fwgMr5I)
#### GRAPHS
Can draw real time graphs
- mA
- mW
- mV
- TC - Temperature Celsius \
To see the values at the moment set cursor to the name of the graph.
User can set the upper/lower displayed values  
  
[demo](https://youtu.be/3VVAKiCTHck)
#### MAX PARAMS
Can test max parameters of a PS
stops test when:
- reached max current output
- voltage drop below the threshhold (3.9V)
- the board protection is raised (current above 3.2A)  
  
[demo](https://youtu.be/ybpfJkCFA_E)
#### RESISTANCE
Can test resistance parameters of a cable
- To run the test successfully connect a diffirent from main PS and input socket with a reference cable (with known resistance (for now it is 228 mOhm))
- take measures using option 1
- to the same PS connect the cable you want to test
- take measures using option 2  
  
[demo](https://www.youtube.com/watch?v=rPBj17imWao)
#### CAPACITY
Can test capacity of a power source (for powerbanks)
- connect and wait from 1 hr to 4 hrs
 - output in mW/hr  
  
[demo](https://youtu.be/tY9qM2LNSQs)









