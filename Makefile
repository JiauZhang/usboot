VERSION = 0
PATCHLEVEL = 0
SUBLEVEL = 1
EXTRAVERSION = -rc2

# Do not use make's built-in rules and variables
# Do not print "Entering directory ..."
MAKEFLAGS += -rR --no-print-directory

# Avoid funny character set dependencies
unexport LC_ALL
LC_COLLATE=C
LC_NUMERIC=C
export LC_COLLATE LC_NUMERIC

# Use 'make V=1' to see the full commands
ifeq ("$(origin V)", "command line")
	BUILD_VERBOSE = $(V)
endif
ifndef BUILD_VERBOSE
	BUILD_VERBOSE = 0
endif

# That's our default target when none is given on the command line
PHONY := all
all: usboot

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

ARCH = arm
CPU = cortex-m3
BOARD = bluepill
CROSS_COMPILE = arm-none-eabi-

export ARCH CPU BOARD

AS = $(CROSS_COMPILE)as
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
AR = $(CROSS_COMPILE)ar
NM = $(CROSS_COMPILE)nm
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump

export AS CC LD AR NM OBJCOPY OBJDUMP

LDSCRIPT = $(srctree)/arch/$(ARCH)/$(CPU)/usboot.lds

LIBC_ := $(shell $(CC) -print-file-name=libc.a)
LIBSDIR := $(dir $(LIBC_))
LDFLAGS := 
LIBS := 

CFLAGS = -c -I$(srctree)/include -I$(srctree)/cpu/$(CPU)/include \
	-mthumb -mcpu=cortex-m3

ifeq ($(CPU),cortex-m3)
  CFLAGS += -I$(srctree)/arch/arm/lib/libopencm3/include
  CFLAGS += -DSTM32F1
  LDFLAGS += -L arch/arm/lib/libopencm3/lib
  LIBS += -lopencm3_stm32f1
endif

OBJCOPYFLAGS = -O binary -R .comment -S

export LDFLAGS CFLAGS OBJCOPYFLAGS

# Some generic definitions
include $(srctree)/scripts/Makefile.include

# include user's configuration if it exists
-include include/config/auto.conf

init-y := init/
core-y := board/
libs-y := lib/

# It must be include after core-y
include arch/$(ARCH)/Makefile

usboot-dirs	:= $(patsubst %/,%,$(filter %/, $(init-y) $(core-y) $(libs-y)))

init-y  := $(patsubst %/, %/built-in.o, $(init-y))
core-y  := $(patsubst %/, %/built-in.o, $(core-y))
libs-y1 := $(patsubst %/, %/lib.a, $(libs-y))
libs-y2 := $(patsubst %/, %/built-in.o, $(libs-y))
libs-y  := $(libs-y1) $(libs-y2)

usboot-init := $(head-y) $(init-y)
usboot-main := $(core-y) $(libs-y)
usboot-all := $(usboot-init) $(usboot-main)
usboot-lds := $(LDSCRIPT)

usboot: $(usboot-lds) $(usboot-init) $(usboot-main)
	$(Q)echo 'LD        $@'
	$(Q)$(LD) -o $@ -T $(LDSCRIPT) $(LDFLAGS) \
	$(usboot-init) --start-group $(usboot-main) $(LIBS) --end-group
	@echo "\033[31mUSBOOT:\033[0m $@ is ready"

# The actual objects are generated when descending, 
# make sure no implicit rule kicks in
$(sort $(usboot-init) $(usboot-main)) $(usboot-lds): $(usboot-dirs) ;

PHONY += $(usboot-dirs)
$(usboot-dirs):
	$(Q)$(MAKE) $(build)=$@

usboot.bin: usboot
	$(Q)echo "OBJCOPY   $@"
	$(Q)$(OBJCOPY) $(OBJCOPYFLAGS) $^ $@
	@echo "\033[31mUSBOOT:\033[0m $@ is ready"

usboot.dis: usboot
	$(Q)echo "OBJDUMP   $@"
	$(Q)$(OBJDUMP) -D -m arm $< > $@
	@echo "\033[31mUSBOOT:\033[0m $@ is ready"

SYSTEM_MAP = \
		$(NM) $1 | \
		grep -v '\(compiled\)\|\(\.o$$\)\|\( [aUw] \)\|\(\.\.ng$$\)\|\(LASH[RL]DI\)' | \
		LC_ALL=C sort
System.map:	usboot
	$(Q)echo "SYSMAP    $@"
	@$(call SYSTEM_MAP,$<) > $@
	@echo "\033[31mUSBOOT:\033[0m $@ is ready"

clean:
	@echo "\033[31mNot Implemented\033[0m"

PHONY += FORCE
FORCE:

.PHONY: $(PHONY)
