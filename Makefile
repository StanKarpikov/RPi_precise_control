ccflags-y := -std=gnu99 -Wno-declaration-after-statement
obj-m += ptime_control.o
ptime_control-objs := ptime_control_main.o rpi_pwm_control.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
 
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
