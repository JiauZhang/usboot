/*
 * stm32f103 usboot.lds
 */

ENTRY(_vectors)

MEMORY {
	sram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
	flash (rx) : ORIGIN = 0x8000000, LENGTH = 64K
}

SECTIONS
{
	. = 0x8000000;

	. = ALIGN(4);
	.vectors : {
		_text_start = .;
		KEEP(*(.vectors))
	} >flash

	. = ALIGN(4);
	.text : {
		*(.text*)
		*(.glue_7t)
		*(.glue_7)
	} >flash

	. = ALIGN(4);
	.rodata : {
		*(.rodata*)
		. = ALIGN(4);
		_text_end = .;
	} >flash

	. = ALIGN(4);
	.data : {
		_data_start = .;
		*(.data*)
		_data_end = .;
	} >sram AT> flash

	. = ALIGN(4);
	.bss : {
		_bss_start = .;
		*(.bss*)
	} >sram
	
	. = ALIGN(4);
	_bss_end = .;
}