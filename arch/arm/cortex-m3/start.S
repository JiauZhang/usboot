/* 
 * stm32f103 start.S
 */

.syntax unified
.cpu cortex-m3
.fpu softvfp
.thumb

.global _reset_irq
.global _vectors
.global _tstart
.global _tend
.global _dstart
.global _dend
.global _bstart
.global _bend

.section .vectors, "ax"

.equ _istack, 0x20005000

_vectors:
	.word _istack
	.word _reset_irq
	.word _nmi_irq
	.word _hard_fault_irq
	.word _mem_mange_fault_irq
	.word _bus_fault_irq
	.word _usage_fault_irq
	.rept 4
		.word 0
	.endr
	.word _svc_irq
	.rept 2
		.word 0
	.endr
	.word _pendsv_irq
	.word _systick_irq
	.word _wdg_irq
	.word _pvd_irq
	.word _tamper_irq
	.word _rtc_irq
	.word _flash_irq
	.word _rcc_irq
	.word _exti0_irq
	.word _exit1_irq
	.word _exit2_irq
	.word _exit3_irq
	.word _exit4_irq
	.word _dma1_ch1_irq
	.word _dma1_ch2_irq
	.word _dma1_ch3_irq
	.word _dma1_ch4_irq
	.word _dma1_ch5_irq
	.word _dma1_ch6_irq
	.word _dma1_ch7_irq
	.word _adc1_2_irq
	.word _usb_hp_can_tx_irq
	.word _usb_lp_can_rx0_irq
	.word _can_rx1_irq
	.word _can_sce_irq
	.word _exti9_5_irq
	.word _tim1_brk_irq
	.word _tim1_up_irq
	.word _tim1_trg_com_irq
	.word _tim1_cc_irq
	.word _tim2_irq
	.word _tim3_irq
	.word _tim4_irq
	.word _i2c1_ev_irq
	.word _i2c1_er_irq
	.word _i2c2_ev_irq
	.word _i2c2_er_irq
	.word _spi1_irq
	.word _spi2_irq
	.word _usart1_irq
	.word _usart2_irq
	.word _usart3_irq
	.word _exti15_10_irq
	.word _rtc_alarm_irq
	.word _usb_wakeup_irq
	.rept 8
		.word 0
	.endr

.section .text._reset, "ax"

_reset_irq:
	@ clear bss
	mov r2, #0x00000000
	/*
	 * invalid usage
	 * ldr r0, _bstart
	 * ldr r1, _bend
	 */
	ldr r0, =_bstart
	ldr r1, =_bend
	str r2, [r0]
	cmp r0, r1
	blo _reset_irq
	b main
	@ never be here!
	b _reset_irq

_loop:
	b _loop

.weak _nmi_irq
.thumb_set _nmi_irq, _loop
.weak _hard_fault_irq
.thumb_set _hard_fault_irq, _loop
.weak _mem_mange_fault_irq
.thumb_set _mem_mange_fault_irq, _loop
.weak _bus_fault_irq
.thumb_set _bus_fault_irq, _loop
.weak _usage_fault_irq
.thumb_set _usage_fault_irq, _loop
.weak _svc_irq
.thumb_set _svc_irq, _loop
.weak _pendsv_irq
.thumb_set _pendsv_irq, _loop
.weak _systick_irq
.thumb_set _systick_irq, _loop
.weak _wdg_irq
.thumb_set _wdg_irq, _loop
.weak _pvd_irq
.thumb_set _pvd_irq, _loop
.weak _tamper_irq
.thumb_set _tamper_irq, _loop
.weak _rtc_irq
.thumb_set _rtc_irq, _loop
.weak _flash_irq
.thumb_set _flash_irq, _loop
.weak _rcc_irq
.thumb_set _rcc_irq, _loop
.weak _exti0_irq
.thumb_set _exti0_irq, _loop
.weak _exit1_irq
.thumb_set _exit1_irq, _loop
.weak _exit2_irq
.thumb_set _exit2_irq, _loop
.weak _exit3_irq
.thumb_set _exit3_irq, _loop
.weak _exit4_irq
.thumb_set _exit4_irq, _loop
.weak _dma1_ch1_irq
.thumb_set _dma1_ch1_irq, _loop
.weak _dma1_ch2_irq
.thumb_set _dma1_ch2_irq, _loop
.weak _dma1_ch3_irq
.thumb_set _dma1_ch3_irq, _loop
.weak _dma1_ch4_irq
.thumb_set _dma1_ch4_irq, _loop
.weak _dma1_ch5_irq
.thumb_set _dma1_ch5_irq, _loop
.weak _dma1_ch6_irq
.thumb_set _dma1_ch6_irq, _loop
.weak _dma1_ch7_irq
.thumb_set _dma1_ch7_irq, _loop
.weak _adc1_2_irq
.thumb_set _adc1_2_irq, _loop
.weak _usb_hp_can_tx_irq
.thumb_set _usb_hp_can_tx_irq, _loop
.weak _usb_lp_can_rx0_irq
.thumb_set _usb_lp_can_rx0_irq, _loop
.weak _can_rx1_irq
.thumb_set _can_rx1_irq, _loop
.weak _can_sce_irq
.thumb_set _can_sce_irq, _loop
.weak _exti9_5_irq
.thumb_set _exti9_5_irq, _loop
.weak _tim1_brk_irq
.thumb_set _tim1_brk_irq, _loop
.weak _tim1_up_irq
.thumb_set _tim1_up_irq, _loop
.weak _tim1_trg_com_irq
.thumb_set _tim1_trg_com_irq, _loop
.weak _tim1_cc_irq
.thumb_set _tim1_cc_irq, _loop
.weak _tim2_irq
.thumb_set _tim2_irq, _loop
.weak _tim3_irq
.thumb_set _tim3_irq, _loop
.weak _tim4_irq
.thumb_set _tim4_irq, _loop
.weak _i2c1_ev_irq
.thumb_set _i2c1_ev_irq, _loop
.weak _i2c1_er_irq
.thumb_set _i2c1_er_irq, _loop
.weak _i2c2_ev_irq
.thumb_set _i2c2_ev_irq, _loop
.weak _i2c2_er_irq
.thumb_set _i2c2_er_irq, _loop
.weak _spi1_irq
.thumb_set _spi1_irq, _loop
.weak _spi2_irq
.thumb_set _spi2_irq, _loop
.weak _usart1_irq
.thumb_set _usart1_irq, _loop
.weak _usart2_irq
.thumb_set _usart2_irq, _loop
.weak _usart3_irq
.thumb_set _usart3_irq, _loop
.weak _exti15_10_irq
.thumb_set _exti15_10_irq, _loop
.weak _rtc_alarm_irq
.thumb_set _rtc_alarm_irq, _loop
.weak _usb_wakeup_irq
.thumb_set _usb_wakeup_irq, _loop

.section .data.start
_tstart:
	.word _text_start
_tend:
	.word _text_end
_dstart:
	.word _data_start
_dend:
	.word _data_end
_bstart:
	.word _bss_start
_bend:
	.word _bss_end
