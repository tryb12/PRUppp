PRU_SUPPORT:=/usr/lib/ti/pru-software-support-package

INCLUDE=--include_path=$(PRU_SUPPORT)/include --include_path=$(PRU_SUPPORT)/include/am335x

pru_options 		= --silicon_version=2 --hardware_mac=on -i/usr/include/arm-linux-gnueabihf/include -i/usr/include/arm-linux-gnueabihf/lib $(INCLUDE)
pru_compiler 		= /usr/bin/clpru
pru_hex_converter 	= /usr/bin/hexpru


all: exports arm_host

arm_host: pru_data.bin pru_code.bin arm_host.c
	gcc -std=gnu11 arm_host.c -o arm_host -lprussdrv -lrt

exports:
	@export PRU_SDK_DIR=/usr
	@export PRU_CGT_DIR=/usr/include/arm-linux-gnueabihf

PRUppp.obj: PRUppp.c
	$(pru_compiler) $(pru_options) --opt_level=off -c PRUppp.c
			
PRUppp.elf: PRUppp.obj 
	$(pru_compiler) $(pru_options) -z PRUppp.obj -llibc.a -m PRUppp.map -o PRUppp.elf AM335x_PRU.cmd --quiet 

pru_code.bin pru_data.bin: bin.cmd PRUppp.elf
	$(pru_hex_converter) bin.cmd ./PRUppp.elf --quiet

clean:
	rm PRUppp.obj
	rm PRUppp.elf
	rm PRUppp.map
