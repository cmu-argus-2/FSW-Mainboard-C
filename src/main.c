#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "argus/tasks/adcs.h"
#include "argus/tasks/eps_task.h"
#include "argus_version.h"

#define ADCS_STACK_SIZE 3072
/* Python runs ADCS at priority 1, ahead of EPS (2), which has its own thread. */
#define ADCS_PRIORITY K_PRIO_PREEMPT(1)

K_THREAD_STACK_DEFINE(adcs_stack, ADCS_STACK_SIZE);
static struct k_thread adcs_thread;

static void adcs_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	adcs_run(p1);
}

int main(void)
{
	printk("Argus FSW mainboard rev %s (%s)\n", CONFIG_BOARD_REVISION, ARGUS_GIT_VERSION);
	printk("EPS 1 Hz; time is uptime seconds; full record every fifth run\n");
	printk("ADCS: IMU connected; state fixed at DETUMBLING; no light sensors or coils\n");
	k_thread_create(&adcs_thread, adcs_stack, K_THREAD_STACK_SIZEOF(adcs_stack), adcs_entry,
			(void *)adcs_hardware_io(), NULL, NULL, ADCS_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&adcs_thread, "adcs");
	eps_start(eps_hardware_io());
	return 0;
}
