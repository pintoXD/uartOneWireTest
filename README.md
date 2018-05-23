# usart-driver

## iButton read over UART on STM32F0xx

An attempt to implement a functional firmware, to read the serial number from an iButton, over UART communication. 
The follow firmware is not really functional, but can read some data after the correct request command to iButton ROM. Even I  had follow the officials application notes and datasheets, still can't get the data correctly.
The main issue: After write the Read ROM command to iButton, at the read, lot of useless data come before the desired data. 
Maybe, the follow point,  can help to find the problem and solve it:

* The data sample is too fast: The OneWire is a slow protocol. Maybe, at the read moment, the MCU doesn't give time enough to iButton read, do whatever it does, and send a response back to MCU, before some new bit read request come;
* Wrong clock configuration;
* Wrong resistor;
* [...]
