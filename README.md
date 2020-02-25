# TlsrComProg
TLSR826x programmator via COM port

### Telink SWIRE simulation on a COM port + COM Flasher.

Flash programming for TLSR826x chips using only a COM port.

![SCH](https://github.com/pvvx/TlsrComProg/blob/master/schematic.gif)

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
    
