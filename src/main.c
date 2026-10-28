#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "argus/tasks/eps.h"
#include "argus_version.h"

int main(void)
{
	printk("Argus FSW mainboard rev %s (%s)\n", CONFIG_BOARD_REVISION, ARGUS_GIT_VERSION);
	static struct eps_state state;
	const struct eps_io *io = eps_zephyr_init();

	eps_init(&state);
	printk("EPS 1 Hz; time is monotonic seconds since boot; full record every fifth run\n");
	int64_t next_run = k_uptime_get();

	for (;;) {
		(void)eps_step(&state, io, (int32_t)(k_uptime_get() / 1000));
		next_run += 1000;
		/* Do not accumulate execution-time drift or burst through missed ticks. */
		int64_t now = k_uptime_get();

		if (next_run <= now) {
			next_run += ((now - next_run) / 1000 + 1) * 1000;
		}
		k_sleep(K_TIMEOUT_ABS_MS(next_run));
	}
}
