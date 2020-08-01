/*
 * usboot/init/main.c
 */

#include <common.h>

int bss_test;
extern unsigned int _bss_start;

void _flash_irq()
{
	int i;
	for (i=0; i<10; i++);
}

int main(void)
{
	clock_init();
	bss_test = _bss_start;
	board_init();

	return 0;
}
