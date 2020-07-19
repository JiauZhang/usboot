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

# Look for make include files relative to root of usboot src
MAKEFLAGS += --include-dir=$(srctree)

# src-y = $(srctree)/cpu/stm32f103/start.S init/main.c
# src-y += $(srctree)/board/bluepill/board.c
src-y :=

# obj-y = $(srctree)/cpu/stm32f103/start.o init/main.o
# obj-y += $(srctree)/board/bluepill/board.o

CPU = stm32f103
BOARD = bluepill
CROSS_COMPILE = arm-none-eabi-

export CPU bluepill

CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump

export CC LD OBJCOPY OBJDUMP

LDSCRIPT = $(srctree)/cpu/$(CPU)/usboot.lds
CFLAGS = -c -I$(srctree)/include
OBJCOPYFLAGS = -O binary -R .comment -S

# Some generic definitions
include $(srctree)/scripts/Makefile.include

# CPU specific files
include $(srctree)/cpu/Makefile
include $(srctree)/init/Makefile
include $(srctree)/board/Makefile

# obj-y
obj-y := $(patsubst %.c,%.o,$(src-y))
obj-y := $(patsubst %.S,%.o,$(obj-y))
obj-y := $(patsubst $.s,%.o,$(obj-y))

usboot.bin: usboot.elf
	$(Q)echo "OBJCOPY   $@"
	$(Q)$(OBJCOPY) $(OBJCOPYFLAGS) $^ $@

usboot.dis: usboot.elf
	$(Q)echo "OBJDUMP   $@"
	$(Q)$(OBJDUMP) -D -m arm $< > $@

usboot.elf: $(obj-y)
	$(Q)echo "LD        $@"
	$(Q)$(LD) $(obj-y) -T $(LDSCRIPT)  -o $@ 

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
