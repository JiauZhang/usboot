/* 
 * stm32f103 start.S
 */

.syntax unified
.cpu cortex-m3
.fpu softvfp
.thumb

.global _reset
.global _vectors
.global _data_start
.global _data_end
.global _bss_start
.global _bss_end

.section .vectors, "ax"

.equ _istack, 0x20005000

_vectors:
	.word _istack
	.word _reset
	.word _nmi
	.word _hard_fault
	.word _mem_mange_fault
	.word _bus_fault
	.word _usage_fault
	.rept 4
		.word 0
	.endr
	.word _svc
	.rept 2
		.word 0
	.endr
	.word _pendsv
	.word _systick
	.word _wdg
	.word _pvd
	.word _tamper
	.word _rtc
	.word _flash
	.word _rcc
	.word _exti0
	.word _exit1
	.word _exit2
	.word _exit3
	.word _exit4
	.word _dma1_ch1
	.word _dma1_ch2
	.word _dma1_ch3
	.word _dma1_ch4
	.word _dma1_ch5
	.word _dma1_ch6
	.word _dma1_ch7
	.word _adc1_2
	.word _usb_hp_can_tx
	.word _usb_lp_can_rx0
	.word _can_rx1
	.word _can_sce
	.word _exti9_5
	.word _tim1_brk
	.word _tim1_up
	.word _tim1_trg_com
	.word _tim1_cc
	.word _tim2
	.word _tim3
	.word _tim4
	.word _i2c1_ev
	.word _i2c1_er
	.word _i2c2_ev
	.word _i2c2_er
	.word _spi1
	.word _spi2
	.word _usart1
	.word _usart2
	.word _usart3
	.word _exti15_10
	.word _rtc_alarm
	.word _usb_wakeup
	.rept 8
		.word 0
	.endr

.section .text._reset, "ax"

_reset:
	b main
	@ never be here!
	b _reset

_loop:
	b _loop

.weak _nmi
.thumb_set _nmi, _loop
.weak _hard_fault
.thumb_set _hard_fault, _loop
.weak _mem_mange_fault
.thumb_set _mem_mange_fault, _loop
.weak _bus_fault
.thumb_set _bus_fault, _loop
.weak _usage_fault
.thumb_set _usage_fault, _loop
.weak _svc
.thumb_set _svc, _loop
.weak _pendsv
.thumb_set _pendsv, _loop
.weak _systick
.thumb_set _systick, _loop
.weak _wdg
.thumb_set _wdg, _loop
.weak _pvd
.thumb_set _pvd, _loop
.weak _tamper
.thumb_set _tamper, _loop
.weak _rtc
.thumb_set _rtc, _loop
.weak _flash
.thumb_set _flash, _loop
.weak _rcc
.thumb_set _rcc, _loop
.weak _exti0
.thumb_set _exti0, _loop
.weak _exit1
.thumb_set _exit1, _loop
.weak _exit2
.thumb_set _exit2, _loop
.weak _exit3
.thumb_set _exit3, _loop
.weak _exit4
.thumb_set _exit4, _loop
.weak _dma1_ch1
.thumb_set _dma1_ch1, _loop
.weak _dma1_ch2
.thumb_set _dma1_ch2, _loop
.weak _dma1_ch3
.thumb_set _dma1_ch3, _loop
.weak _dma1_ch4
.thumb_set _dma1_ch4, _loop
.weak _dma1_ch5
.thumb_set _dma1_ch5, _loop
.weak _dma1_ch6
.thumb_set _dma1_ch6, _loop
.weak _dma1_ch7
.thumb_set _dma1_ch7, _loop
.weak _adc1_2
.thumb_set _adc1_2, _loop
.weak _usb_hp_can_tx
.thumb_set _usb_hp_can_tx, _loop
.weak _usb_lp_can_rx0
.thumb_set _usb_lp_can_rx0, _loop
.weak _can_rx1
.thumb_set _can_rx1, _loop
.weak _can_sce
.thumb_set _can_sce, _loop
.weak _exti9_5
.thumb_set _exti9_5, _loop
.weak _tim1_brk
.thumb_set _tim1_brk, _loop
.weak _tim1_up
.thumb_set _tim1_up, _loop
.weak _tim1_trg_com
.thumb_set _tim1_trg_com, _loop
.weak _tim1_cc
.thumb_set _tim1_cc, _loop
.weak _tim2
.thumb_set _tim2, _loop
.weak _tim3
.thumb_set _tim3, _loop
.weak _tim4
.thumb_set _tim4, _loop
.weak _i2c1_ev
.thumb_set _i2c1_ev, _loop
.weak _i2c1_er
.thumb_set _i2c1_er, _loop
.weak _i2c2_ev
.thumb_set _i2c2_ev, _loop
.weak _i2c2_er
.thumb_set _i2c2_er, _loop
.weak _spi1
.thumb_set _spi1, _loop
.weak _spi2
.thumb_set _spi2, _loop
.weak _usart1
.thumb_set _usart1, _loop
.weak _usart2
.thumb_set _usart2, _loop
.weak _usart3
.thumb_set _usart3, _loop
.weak _exti15_10
.thumb_set _exti15_10, _loop
.weak _rtc_alarm
.thumb_set _rtc_alarm, _loop
.weak _usb_wakeup
.thumb_set _usb_wakeup, _loop

.section .data.start
.word _data_start
.word _data_end
.word _bss_start
.word _bss_end