# TlsrComProg
TLSR826x programmator via COM port

(!) Limitations: Only TLSR8266 and TLSR8269 Chip Used
(modules JDY-10/M, E104-BT05, E104-BT10N/M, ...)

### Telink SWIRE simulation on a COM port + COM Flasher.

Flash programming for TLSR826x chips using only a COM port.

![SCH](https://github.com/pvvx/TlsrComProg/blob/master/Doc/img/schematic.gif)

    Usage: TlsrComProg [-h] [--port PORT] [--tact TACT] [--fldr FLDR]
                       [--baud BAUD]
                       {rf,we,wf,es,ea} ...
    
    TLSR826x Floader version 25.02.20 (beta)
    
    positional arguments:
      {rf,we,wf,es,ea}      Run TlsrComProg {command} -h for additional help
        rf                  Read Flash to binary file
        we                  Write file to Flash with sectors erases
        wf                  Write file to Flash without sectors erases
        es                  Erase Region (sectors) of Flash
        ea                  Erase All Flash
    
    optional arguments:
      -h, --help            show this help message and exit
      --port PORT, -p PORT  Serial port device (default: COM1)
      --tact TACT, -t TACT  Time Activation ms (0-off, default: 600 ms)
      --fldr FLDR, -f FLDR  Filename floader (default: floader.bin)
      --baud BAUD, -b BAUD  UART Baud Rate (default: 230400)
    

------------

#### Samples:
> **Write Flash:** python.exe TlsrComProg.py -p COM3 -t 5000 we 0 8266_jdy_10.bin
```
================================================
TLSR826x Floader version 26.02.20
------------------------------------------------
Open COM3, 230400 baud...
Reset module (RTS low)...
Activate (5000 ms)...
Connection...
Load <floader.bin> to 0x8000...
Bin bytes writen: 1860
CPU go Start...
------------------------------------------------
ChipID: 0x5325 (TLSR8266), Floader ver: 0.1
Flash JEDEC ID: 514013, Size: 512 kbytes
------------------------------------------------
Inputfile: 8266_jdy_10.bin
Write Flash data 0x00000000 to 0x0000a984...
------------------------------------------------
Done!
```
> **Read Flash:** python.exe TlsrComProg.py -p COM3 rf 0 0x80000 ff.bin
```
================================================
TLSR826x Floader version 26.02.20
------------------------------------------------
Open COM3, 230400 baud...
Reset module (RTS low)...
Activate (600 ms)...
Connection...
Load <floader.bin> to 0x8000...
Bin bytes writen: 1860
CPU go Start...
------------------------------------------------
ChipID: 0x5325 (TLSR8266), Floader ver: 0.1
Flash JEDEC ID: 514013, Size: 512 kbytes
------------------------------------------------
Read Flash from 0x000000 to 0x080000...
Outfile: ff.bin
------------------------------------------------
Done!
```
> **Erase All Flash:** python.exe TlsrComProg.py -p COM3 ea
```
================================================
TLSR826x Floader version 26.02.20
------------------------------------------------
Open COM3, 230400 baud...
Reset module (RTS low)...
Activate (600 ms)...
Connection...
Load <floader.bin> to 0x8000...
Bin bytes writen: 1860
CPU go Start...
------------------------------------------------
ChipID: 0x5325 (TLSR8266), Floader ver: 0.1
Flash JEDEC ID: 514013, Size: 512 kbytes
------------------------------------------------
Erase All Flash ...
------------------------------------------------
Done!
```

[Examples of using](https://github.com/pvvx/TlsrComProg/tree/master/Doc)

[UartFloader Source Code](https://github.com/pvvx/TlsrComProg/tree/master/Uartfloader)

