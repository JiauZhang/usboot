/*
 * usboot/init/main.c
 */

// #include <main.h>

int bss_test;
extern unsigned int _bss_start;

void _flash_irq()
{
	int i;
	for (i=0; i<10; i++);
}

int main(void)
{
	// clock_init();
	// ram_init();
	bss_test = _bss_start;

	return 0;
}
