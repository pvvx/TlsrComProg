@rem Path=E:\Telink\NanosicSDK;E:\Telink\NanosicSDK\jre\bin;E:\Telink\NanosicSDK\opt\tc32\tools;E:\Telink\NanosicSDK\opt\tc32\bin;E:\Telink\NanosicSDK\usr\bin;E:\Telink\NanosicSDK\bin
@del /Q floader.bin
@del /Q floader.lst
make -s clean
make -s -j4