#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "argus/tasks/adcs.h"
#include "argus/tasks/eps.h"
#include "argus_version.h"

#define ADCS_STACK_SIZE 3072
/* Python runs ADCS at priority 1, ahead of EPS (2). EPS runs on the main
 * thread (priority 0), so ADCS shares that priority until scheduling is ported.
 */
#define ADCS_PRIORITY K_PRIO_PREEMPT(0)

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
	printk("EPS console-only: hardware callbacks are not connected; no peripheral access\n");
	printk("EPS 1 Hz; time is uptime seconds; full record every fifth run\n");
	printk("ADCS console-only: state fixed at DETUMBLING; no sensors or coils connected\n");
	k_thread_create(&adcs_thread, adcs_stack, K_THREAD_STACK_SIZEOF(adcs_stack), adcs_entry,
			(void *)adcs_console_only_io(), NULL, NULL, ADCS_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&adcs_thread, "adcs");
	eps_run(eps_console_only_io());
	return 0;
}
