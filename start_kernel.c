#include <stdlib.h>
#include <stdint.h>

extern unsigned int _end_text;
extern unsigned int _start_data;
extern unsigned int _end_data;
void start_kernel(void)
{
	void (*kernel)(uint32_t reserved, uint32_t mach, uint32_t dt) = (void (*)(uint32_t, uint32_t, uint32_t))(KERNEL_ADDR | 1);

	/* the DTB is just after the bootloader */
	kernel(0, ~0UL, /*DTB_ADDR*/ (uint32_t)&_end_text + (uint32_t)&_end_data - (uint32_t)&_start_data);
}
