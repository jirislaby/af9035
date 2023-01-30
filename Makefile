KDIR=/lib/modules/$(shell uname -r)/build
KBUILD=$(MAKE) -C $(KDIR) M=$(PWD)

obj-m := af9035.o

default: # us
	$(KBUILD) modules

us: us.o

firmware.bin: firmware.o
	objcopy -O binary -j .text firmware.o firmware.bin

%.s: %.c
	$(KBUILD) $@

clean:
	$(KBUILD) clean

.PHONY: default clean
