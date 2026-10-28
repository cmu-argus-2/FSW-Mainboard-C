#include <zephyr/sys/printk.h>

#include "argus/tasks/eps.h"
#include "argus_version.h"

int main(void)
{
	printk("Argus FSW mainboard rev %s (%s)\n", CONFIG_BOARD_REVISION, ARGUS_GIT_VERSION);
	printk("EPS console-only: hardware callbacks are not connected; no peripheral access\n");
	printk("EPS 1 Hz; time is uptime seconds; full record every fifth run\n");
	eps_run(eps_console_only_io());
	return 0;
}
