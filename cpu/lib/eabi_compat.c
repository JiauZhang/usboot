#include <common.h>

/*
 * if the target cpu dose not support div operation
 * and you have to use div operation, the compiler will
 * warn " undefined reference to `__aeabi_uidiv' " error
 */
int raise (int signum)
{
	return 0;
}

/* Dummy function to avoid linker complaints */
void __aeabi_unwind_cpp_pr0(void) {}

void __aeabi_unwind_cpp_pr1(void) {}
