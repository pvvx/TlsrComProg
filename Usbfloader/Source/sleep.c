#include "common.h"
#include "clock.h"

_attribute_ram_code_
void sleep_us (u32 us) {
	u32 t = clock_time();
	while(!clock_time_exceed(t, us));
}
