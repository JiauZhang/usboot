all:
	arm-none-eabi-gcc start.S -o start.elf -T usboot.lds -nostartfiles
	arm-none-eabi-objcopy -O binary -S start.elf start.bin
	arm-none-eabi-objdump -D -m arm start.elf > start.dis