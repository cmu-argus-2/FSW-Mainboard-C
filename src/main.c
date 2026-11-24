#include <zephyr/sys/printk.h>

#include "argus/tasks/eps_task.h"
#include "argus_version.h"

int main(void)
{
	printk("Argus FSW mainboard rev %s (%s)\n", CONFIG_BOARD_REVISION, ARGUS_GIT_VERSION);
	printk("EPS 1 Hz; time is uptime seconds; full record every fifth run\n");
	eps_start(eps_hardware_io());
	return 0;
}
