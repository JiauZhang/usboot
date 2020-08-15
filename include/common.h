#ifndef __COMMON_H__
#define __COMMON_H__

// override by board specific file
void delay(unsigned int time);
void printk(const char *str);

// defined in board/xxx/board.c
void clock_init();
void board_init();

#endif
