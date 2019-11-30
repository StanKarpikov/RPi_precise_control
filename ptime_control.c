/*--------------------------------------------------------------
						INCLUDES
---------------------------------------------------------------*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/sched.h>
#include <linux/ktime.h>

/*--------------------------------------------------------------
			PRIVATE FUNCTIONS DECLARATION
---------------------------------------------------------------*/

static enum hrtimer_restart function_timer(struct hrtimer *);

/*--------------------------------------------------------------
					PRIVATE VARIABLES
---------------------------------------------------------------*/

static struct hrtimer timer_next_close_s;
static ktime_t timer_next_close;

/*--------------------------------------------------------------
					PRIVATE FUNCTIONS
---------------------------------------------------------------*/

static enum hrtimer_restart function_timer(struct hrtimer * unused)
{
        //gpio_set_value(GPIO_OUTPUT,1);

        hrtimer_forward_now(& timer_next_close_s, timer_next_close);

        //hrtimer_cancel(& htimer);

        return HRTIMER_RESTART;
}

/*--------------------------------------------------------------
					MODULE FUNCTIONS
---------------------------------------------------------------*/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Stanislav Karpikov");
MODULE_DESCRIPTION("GPIO and PWM control based on the times list");
MODULE_VERSION("0.0.1");

static int __init ptime_control_init(void) {
 printk(KERN_INFO "PTime control started\n");

 timer_next_close = ktime_set(0, 100); /*seconds,nanoseconds*/
 hrtimer_init (& timer_next_close_s, CLOCK_REALTIME, HRTIMER_MODE_REL);
 timer_next_close_s.function = function_timer;
 hrtimer_start(& timer_next_close_s, timer_next_close, HRTIMER_MODE_REL);

 return 0;
}

static void __exit ptime_control_exit(void) {
 printk(KERN_INFO "PTime control stopped\n");
}

module_init(ptime_control_init);
module_exit(ptime_control_exit);

