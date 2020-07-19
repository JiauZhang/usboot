# Do not use make's built-in rules and variables
# Do not print "Entering directory ..."
MAKEFLAGS += -rR --no-print-directory

# Use 'make V=1' to see the full commands
ifeq ("$(origin V)", "command line")
	BUILD_VERBOSE = $(V)
endif
ifndef BUILD_VERBOSE
	BUILD_VERBOSE = 0
endif

ifeq ($(BUILD_VERBOSE),1)
	quiet = 
	Q = 
else
	quiet = quiet_
	Q = @
endif

export quiet Q BUILD_VERBOSE

srctree := $(CURDIR)
objtree := $(CURDIR)
src := $(srctree)
obj := $(objtree)

export srctree objtree

src-y = $(srctree)/cpu/stm32f103/start.S init/main.c
src-y += $(srctree)/board/bluepill/board.c

obj-y = $(srctree)/cpu/stm32f103/start.o init/main.o
obj-y += $(srctree)/board/bluepill/board.o

LDSCRIPT = $(srctree)/cpu/stm32f103/usboot.lds
CFLAGS = -c -I$(srctree)/include
LDFLAGS = -O binary -S

CROSS_COMPILE = arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump

usboot.bin: usboot.elf
	$(Q)echo "OBJCOPY   $@"
	$(Q)$(OBJCOPY) $(LDFLAGS) $^ $@

usboot.dis: usboot.elf
	$(Q)echo "OBJDUMP   $@"
	$(Q)$(OBJDUMP) -D -m arm $< > $@


usboot.elf: $(obj-y)
	$(Q)echo "LD        $@"
	$(Q)$(LD) $(obj-y) -T $(LDSCRIPT) -o $@ 

%.o: %.c
	$(Q)echo "CC        $@"
	$(Q)$(CC) $(CFLAGS) $< -o $@

%.o: %.S
	$(Q)echo "CC        $@"
	$(Q)$(CC) $(CFLAGS) $< -o $@

%.o: %.s
	$(Q)echo "CC        $@"
	$(Q)$(CC) $(CFLAGS) $< -o $@

clean:
	$(Q)rm -f usboot.*
	$(Q)rm -f $(obj-y)
