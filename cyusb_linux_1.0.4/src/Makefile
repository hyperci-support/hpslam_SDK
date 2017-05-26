all:
	g++ -o 00_fwload          00_fwload.c          -L ../lib -l cyusb
	g++ -o 01_getdesc         01_getdesc.c         -L ../lib -l cyusb
	g++ -o 03_getconfig       03_getconfig.c       -L ../lib -l cyusb
	g++ -o 04_kerneldriver    04_kerneldriver.c    -L ../lib -l cyusb
	g++ -o 05_claiminterface  05_claiminterface.c  -L ../lib -l cyusb
	g++ -o 06_setalternate    06_setalternate.c    -L ../lib -l cyusb
	g++ -o 08_cybulk          08_cybulk.c          -L ../lib -l cyusb -l pthread
	g++ -o config_parser      config_parser.c      -L ../lib -l cyusb
	g++ -o cyusbd             cyusbd.c             -L ../lib -l cyusb
	g++ -o getconfig	  getconfig.c          -L ../lib -l cyusb
	g++ -o download_fx2       download_fx2.c       -L ../lib -l cyusb
	g++ -o download_fx3       download_fx3.c       -L ../lib -l cyusb
clean:
	rm -f 00_fwload 01_getdesc 03_getconfig 04_kerneldriver 05_claiminterface 06_setalternate 07_bulkreader 07_bulkwriter
	rm -f 08_cybulk config_parser cyusbd getconfig download_fx2 download_fx3

help:
	@echo	'make		would compile all source programs in this directory
	@echo	'make clean	would remove all object code in this directory
