#include <common.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

void clock_init()
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_USART1);
}

void usart_init()
{
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, \
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_enable(USART1);
}

void gpio_setup(void)
{
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, \
		GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

void board_init()
{
	int i = 0;
	
	clock_init();
	gpio_setup();
	usart_init();
	
	while (1) {
		gpio_toggle(GPIOC, GPIO13);
		usart_send_blocking(USART1, 'T');
		
		for (i=0; i<800000; i++)
			__asm__("nop");
	}
}

void delay(unsigned int time) {}

void printk(const char *str) {}