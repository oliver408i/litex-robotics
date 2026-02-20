#include <stdint.h>

#include <generated/csr.h>
#include <generated/soc.h>

static void delay_ms(uint32_t ms)
{
	uint64_t ticks = ((uint64_t)CONFIG_CLOCK_FREQUENCY / 1000u) * ms;

	if (ticks == 0) {
		ticks = 1;
	}
	if (ticks > 0xffffffffu) {
		ticks = 0xffffffffu;
	}

	timer0_en_write(0);
	timer0_reload_write(0);
	timer0_load_write((uint32_t)ticks);
	timer0_en_write(1);

	while (1) {
		timer0_update_value_write(1);
		if (timer0_value_read() == 0) {
			break;
		}
	}
}

int main(void)
{
	while (1) {
		leds_out_write(1); // LED1 on (user_led[1])
		delay_ms(500);
		leds_out_write(0);
		delay_ms(500);
	}
}
