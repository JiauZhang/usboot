// SPDX-License-Identifier: BSD-3-Clause

#include <common.h>

int main(void)
{
	unsigned char str[] = "test uart...\n";

	while (1)
	{
		printk(str);
		delay(1000);
	}
}