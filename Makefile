src-y = cpu/stm32f103/start.S init/main.c
obj-y = cpu/stm32f103/start.o init/main.o
LDSCRIPT = cpu/stm32f103/usboot.lds
CFLAGS = -c -I$(CURDIR)/include

CROSS_COMPILE = arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump

usboot.bin: usboot.elf
	@echo "OBJCOPY   $@"
	@$(OBJCOPY) -O binary -S $^ $@

usboot.dis: usboot.elf
	@echo "OBJDUMP   $@"
	@$(OBJDUMP) -D -m arm $< > $@


usboot.elf: $(obj-y)
	@echo "LD        $@"
	@$(LD) $(obj-y) -T $(LDSCRIPT) -o $@ 

%.o: %.c
	@echo "CC        $@"
	@$(CC) $(CFLAGS) $< -o $@

%.o: %.S
	@echo "CC        $@"
	@$(CC) $(CFLAGS) $< -o $@

clean:
	@rm -f usboot.*
	@rm -f $(obj-y)
