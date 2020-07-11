src-y = cpu/stm32f103/start.S init/main.c
LDSCRIPT = cpu/stm32f103/usboot.lds

all: $(src-y)
	arm-none-eabi-gcc $(src-y) -o usboot.elf -T $(LDSCRIPT) -nostartfiles
	arm-none-eabi-objcopy -O binary -S usboot.elf usboot.bin
	arm-none-eabi-objdump -D -m arm usboot.elf > usboot.dis