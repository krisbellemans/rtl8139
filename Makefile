obj-m := rtl8139.o

KDIR := /usr/src/linux
PWD := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
