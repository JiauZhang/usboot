/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2015 Karl Palsson <karlp@tweak.net.au>
 *
 * SPDX-License-Identifier: GPL-3.0+
 */

/* Linker script for STM32F103x8, 64k flash, 20k RAM.
 * This script also works for the STM32F103xB, as the bootloader uses only the
 * first 8kB of flash. */

/* Define memory regions. */
MEMORY
{
	vectors (rx) : ORIGIN = 0x08000000, LENGTH = 0x150
	rom (rx)  : ORIGIN = 0x08000150, LENGTH = 0x1EB0
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

/* Include the common ld script. */
INCLUDE arch/arm/cortex-m3/stm32f1.lds
