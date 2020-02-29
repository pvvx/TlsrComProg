# TlsrComProg
TLSR826x programmator via COM port

#### Example with Module JDY-10:
![SCH](https://github.com/pvvx/TlsrComProg/blob/master/Doc/img/PrgJDY10.gif)

COM-**RTS** for **RESET** is optional.
If manual reset is used, set the activation duration to a few seconds (** -t 5000 **)
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

#### Example with Module E104-BT05-TB:
![SCH](https://github.com/pvvx/TlsrComProg/blob/master/Doc/img/PgmE104-BT05-TBsm.jpg)
