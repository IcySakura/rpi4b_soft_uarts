obj-m += soft_uart.o

soft_uart-objs := module.o raspberry_soft_uart.o queue.o

ccflags-y := -Wno-incompatible-pointer-types
ccflags-y += -O3 -march=native -mtune=native -pipe

RELEASE = $(shell uname -r)
LINUX = /usr/src/linux-headers-$(RELEASE)

all:
	$(MAKE) -C $(LINUX) M=$(PWD) modules

clean:
	$(MAKE) -C $(LINUX) M=$(PWD) clean

install:
	sudo install -m 644 -c soft_uart.ko /lib/modules/$(RELEASE)
	sudo depmod

