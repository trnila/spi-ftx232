ifneq ($(KERNELRELEASE),)
	obj-m  := spi-ftx232.o

else
	KDIR ?= /lib/modules/`uname -r`/build

default:
		$(MAKE) -C $(KDIR) M=$$PWD

clean:
		$(MAKE) -C $(KDIR) M=$$PWD clean
endif
