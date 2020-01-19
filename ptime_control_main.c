/*--------------------------------------------------------------
                      INCLUDES
---------------------------------------------------------------*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/sched.h>
#include <linux/ktime.h>
#include <linux/rbtree.h>
#include <linux/jiffies.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/timer.h>

#include "rpi_pwm_control.h"

/*--------------------------------------------------------------
                      DEFINES
---------------------------------------------------------------*/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Stanislav Karpikov");
MODULE_DESCRIPTION("GPIO and PWM control based on the times list");
MODULE_VERSION("0.0.1");

#define DEVICE_NAME "ptime_control"
#define CLASS_NAME "chardrv"

#define LOG_PREFIX "PTime: "

#define SEC_TO_NSEC  (1000ULL*1000ULL*1000ULL)
#define USEC_TO_NSEC (1000UL)
#define SEC_TO_USEC  (1000UL*1000ULL)

#define CONTROL_CLEAR_STRING "CLEAR"

#define PWM_OFF_TIMER_DURATION (500)
#define PWM_OPEN_STATE_MS (105)
#define PWM_CLOSED_STATE_MS (180)

#define PWM_ON_OFF_GPIO (17) /** GPIO17 to connect power switch */
/*--------------------------------------------------------------
              PRIVATE FUNCTIONS PROTOTYPES
---------------------------------------------------------------*/

static enum hrtimer_restart function_timer_open(struct hrtimer *);
static enum hrtimer_restart function_timer_close(struct hrtimer *);

static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);

static void __exit ptime_control_exit(void);
/*--------------------------------------------------------------
                      PRIVATE TYPES
---------------------------------------------------------------*/

typedef struct{
  uint32_t open_time_s;
  uint32_t open_time_us;
  uint32_t close_time_s;
  uint32_t close_time_us;
} event_element_t;

struct event_list_element_s {
  struct rb_node node;
  event_element_t key;
};

/** @brief Devices are represented as file structure in the kernel. The file_operations structure from
 *  /linux/fs.h lists the callback functions that you wish to associated with your file operations
 *  using a C99 syntax structure. char devices usually implement open, read, write and release calls
 */
static struct file_operations fops =
{
   .open = dev_open,
   .read = dev_read,
   .write = dev_write,
   .release = dev_release,
};
/*--------------------------------------------------------------
                    PRIVATE VARIABLES
---------------------------------------------------------------*/

static int    major_number;                  /**< Device major number */
static struct class*  ptime_class  = NULL;   /**< Device class pointer */
static struct device* ptime_device = NULL;   /**< Device pointer */
 
static struct timer_list pwm_off_timer;
 
struct rb_root event_list_root = RB_ROOT;
static struct hrtimer timer_next_open_event;
static struct hrtimer timer_next_close_event;

static uint64_t current_pending_open_time=0;
static uint64_t current_pending_close_time=0;

static DEFINE_SPINLOCK(tree_lock);

/*--------------------------------------------------------------
                    PRIVATE FUNCTIONS
---------------------------------------------------------------*/

static void pwm_on(void){
    printk(KERN_INFO LOG_PREFIX "Switch PWM on \n");
    gpio_set_mode(PWM_ON_OFF_GPIO, 1);
    gpio_write(PWM_ON_OFF_GPIO, 1);
}

static void pwm_off(struct timer_list *t)
{
    printk(KERN_INFO LOG_PREFIX "Switch PWM off \n");
    gpio_set_mode(PWM_ON_OFF_GPIO, 1);
    gpio_write(PWM_ON_OFF_GPIO, 0);
}

static void schedule_pwm_off(void){
    mod_timer(&pwm_off_timer, jiffies + msecs_to_jiffies(PWM_OFF_TIMER_DURATION));
}

static void open_action(void){
    pwm_on();
    pwm_pulse_width(PWM_CHANNEL_0, PWM_OPEN_STATE_MS, false);
    schedule_pwm_off();
}

static void close_action(void){
    pwm_on();
    pwm_pulse_width(PWM_CHANNEL_0, PWM_CLOSED_STATE_MS, false);
    schedule_pwm_off();
}

/** @brief Insert new event list element into tree
 */
static int event_list_insert(struct rb_root *root, struct event_list_element_s *new_event)
{
  spin_lock(&tree_lock);
  struct event_list_element_s *check_node;
  struct rb_node **link = &(root->rb_node), *parent = NULL;
  uint64_t new_event_time_comp =  new_event->key.open_time_s * SEC_TO_USEC + new_event->key.open_time_us;

  int i =0;
  while (*link && i<1000) 
  {
	  i++;
	  parent = *link;
	  check_node = rb_entry(parent, struct event_list_element_s, node);
	  if(!check_node)break;

	  uint64_t check_time_comp = check_node->key.open_time_s * SEC_TO_USEC + check_node->key.open_time_us;
	  
	  if(check_time_comp > new_event_time_comp)
	  {
		  link = &((*link)->rb_left);
	  }else if(check_time_comp < new_event_time_comp){
		  link = &((*link)->rb_right);
	  }else{
		  return 1;
	  }
  }
  if(i>500) printk(KERN_ERR "Timeout inserting element!");

  /* Add new node and rebalance tree. */
  rb_link_node(&new_event->node, parent, link);
  rb_insert_color(&new_event->node, root);
	
  spin_unlock(&tree_lock);

  return 1;
}

/** @brief Add new event
 */
int event_add(event_element_t new_event)
{
  struct event_list_element_s *new_node;
  new_node = (struct event_list_element_s *)kmalloc(sizeof(struct event_list_element_s), GFP_KERNEL);
  new_node->key = new_event;
  event_list_insert(&event_list_root, new_node);
  printk(KERN_INFO LOG_PREFIX "Add event [%u.%06u]-[%u.%06u]", new_event.open_time_s, new_event.open_time_us,  new_event.close_time_s, new_event.close_time_us);
  
  return 1;
}

/** @brief Remove passed timeouts
 */
static void remove_passed(uint32_t before_time_s, uint32_t before_time_us)
{
	unsigned long flags;
	spin_lock_irqsave(&tree_lock, flags);
	struct event_list_element_s *minimum_time_element = 0;
	uint32_t next_open_timeout_s  = 0;
	uint32_t next_open_timeout_us = 0;
	uint64_t before_time_comp = before_time_s * SEC_TO_USEC + before_time_us;
	bool continue_delete=false;
	printk(KERN_INFO LOG_PREFIX "Remove events before [%u.%06u]", before_time_s, before_time_us);
	int i=0;
	do
	{
		i++;
		continue_delete = false;
		
		struct rb_node *minimum_time_node=rb_first(&event_list_root);
		if(!minimum_time_node)break;
		
		minimum_time_element = rb_entry(minimum_time_node, struct event_list_element_s, node);
		if(!minimum_time_element)break;

		next_open_timeout_s  = minimum_time_element->key.open_time_s;
        next_open_timeout_us = minimum_time_element->key.open_time_us;
        uint64_t next_open_timeout_comp = next_open_timeout_s * SEC_TO_USEC + next_open_timeout_us;
       
		bool test=next_open_timeout_comp <= before_time_comp;
        if(test)
		{
			printk(KERN_INFO LOG_PREFIX "Remove event at [%u.%06u]", next_open_timeout_s, next_open_timeout_us);
			rb_erase(minimum_time_node, &event_list_root);
			kfree(minimum_time_element);
            continue_delete = true;
		}
	}while(continue_delete && i<1000);
	if(i>500) printk(KERN_ERR "Timeout deleting element!\n");
	spin_unlock_irqrestore(&tree_lock, flags);
}

static __attribute__ ((unused)) void print_events_list(struct rb_node* node, bool is_root)
{
	static int i=0;
	if(is_root)i=0;
    struct event_list_element_s *tmp_element;
    
    if (node)
    {
        print_events_list(node->rb_left,false);
        tmp_element = rb_entry(node, struct event_list_element_s, node);
        uint32_t next_open_timeout_s  = tmp_element->key.open_time_s;
		uint32_t next_open_timeout_us = tmp_element->key.open_time_us;
        printk(KERN_INFO LOG_PREFIX "Event [%d]: [%u.%06u] \n", i++, next_open_timeout_s, next_open_timeout_us);
        print_events_list(node->rb_right,false);
    }
}
/** @brief Extract next timeout and restart high resolution timer
 */
static void update_timer(void)
{
	unsigned long flags;
	print_events_list(event_list_root.rb_node, true);
	//printk(KERN_INFO LOG_PREFIX "Try to update timer..");
        
	struct event_list_element_s *next_time_node = 0;
	uint32_t next_open_timeout_s = 0;
	uint32_t next_open_timeout_us = 0;
	uint32_t next_close_timeout_s = 0;
	uint32_t next_close_timeout_us = 0;
	spin_lock_irqsave(&tree_lock, flags);
		
	/* Get next closest time */
	struct rb_node *next_time=rb_first(&event_list_root);
	if(!next_time)
	{
		spin_unlock_irqrestore(&tree_lock, flags);
        printk(KERN_INFO LOG_PREFIX "No next");
		return;
	}
	
	next_time_node = rb_entry(next_time, struct event_list_element_s, node);
	if(!next_time_node)
	{
		spin_unlock_irqrestore(&tree_lock, flags);
        printk(KERN_INFO LOG_PREFIX "No entry");
		return;
	}
	spin_unlock_irqrestore(&tree_lock, flags);
	
	/* Get next timeout in UNIX seconds */
	next_open_timeout_s  = next_time_node->key.open_time_s;
	next_open_timeout_us = next_time_node->key.open_time_us;

	next_close_timeout_s  = next_time_node->key.close_time_s;
	next_close_timeout_us = next_time_node->key.close_time_us;

	/* Get current time in UNIX seconds */
	struct timespec current_time_ts;
	getnstimeofday (&current_time_ts);
	uint32_t current_time_s  = current_time_ts.tv_sec;
	uint32_t current_time_us = current_time_ts.tv_nsec/USEC_TO_NSEC;
	uint64_t current_time_ns = current_time_s*SEC_TO_NSEC + current_time_us*USEC_TO_NSEC;

	/* Get next time in UNIX nano seconds */
	uint64_t next_open_timeout_ns = next_open_timeout_s*SEC_TO_NSEC + next_open_timeout_us*USEC_TO_NSEC;
	uint64_t next_close_timeout_ns = next_close_timeout_s*SEC_TO_NSEC + next_close_timeout_us*USEC_TO_NSEC;

	/* Set open time */
    //printk(KERN_INFO LOG_PREFIX "Check next: [%u.%06u]", next_open_timeout_s, next_open_timeout_us);
    //printk(KERN_INFO LOG_PREFIX "Check current: [%u.%06u]", current_time_s, current_time_us);
    
	if(next_open_timeout_ns > current_time_ns && (next_open_timeout_ns != current_pending_open_time))
	{
		/* Set/update time */
		printk(KERN_INFO LOG_PREFIX "Update open timer to next time [%u.%06u]", next_open_timeout_s, next_open_timeout_us);		
		hrtimer_try_to_cancel(&timer_next_open_event);
		hrtimer_start(&timer_next_open_event, ns_to_ktime(next_open_timeout_ns), HRTIMER_MODE_ABS);

		if(!current_pending_close_time || 
		  (next_close_timeout_ns > current_pending_close_time) ||
		  (next_open_timeout_ns < current_pending_open_time))
		{
			current_pending_close_time = next_close_timeout_ns;
			printk(KERN_INFO LOG_PREFIX "Update close timer to next time [%u.%06u]", next_close_timeout_s, next_close_timeout_us);
		}
		current_pending_open_time  = next_open_timeout_ns;
	}else{
        //printk(KERN_INFO LOG_PREFIX "Timer already set to closer time");
    }
}

static void update_close_timer(void)
{
	/* Set/update time */
	printk(KERN_INFO LOG_PREFIX "Set close timer to next time [%llu]", current_pending_close_time);
	hrtimer_try_to_cancel(&timer_next_close_event);
	hrtimer_start(&timer_next_close_event, ns_to_ktime(current_pending_close_time), HRTIMER_MODE_ABS);
}

/** @brief High resolution timer callback function
 */
static enum hrtimer_restart function_timer_open(struct hrtimer * unused)
{
	current_pending_open_time = 0;
	////gpio_set_value(GPIO_OUTPUT,1);
    struct timespec current_time_ts;
    getnstimeofday (&current_time_ts);
    uint32_t current_time_s  = current_time_ts.tv_sec;
    uint32_t current_time_us = current_time_ts.tv_nsec / USEC_TO_NSEC;

	printk(KERN_INFO LOG_PREFIX "<-- Open interrupt at [%u.%06u]", current_time_s, current_time_us);
	open_action();
    
	update_close_timer();
	remove_passed(current_time_s, current_time_us);
	update_timer();

	return HRTIMER_NORESTART;
}

/** @brief High resolution timer callback function
 */
static enum hrtimer_restart function_timer_close(struct hrtimer * unused)
{
    struct timespec current_time_ts;
    getnstimeofday (&current_time_ts);
    uint32_t current_time_s  = current_time_ts.tv_sec;
    uint32_t current_time_us = current_time_ts.tv_nsec / USEC_TO_NSEC;

	printk(KERN_INFO LOG_PREFIX ">-- Close interrupt at [%u.%06u]", current_time_s, current_time_us);
    close_action();

	return HRTIMER_NORESTART;
}

/** @brief The device open function that is called each time the device is opened
 *  This will only increment the numberOpens counter in this case.
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_open(struct inode *inodep, struct file *filep){
   //printk(KERN_INFO LOG_PREFIX "Device opened\n");
   return 0;
}
 
/** @brief This function is called whenever device is being read from user space i.e. data is
 *  being sent from the device to the user. In this case is uses the copy_to_user() function to
 *  send the buffer string to the user and captures any errors.
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 *  @param buffer The pointer to the buffer to which this function writes the data
 *  @param len The length of the b
 *  @param offset The offset if required
 */
static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset){
  /* Note: for the future use */
  printk(KERN_INFO LOG_PREFIX "Trying to read\n");
  return -EFAULT;
}
 
/** @brief This function is called whenever the device is being written to from user space i.e.
 *  data is sent to the device from the user. The data is copied to the message[] array in this
 *  LKM using the sprintf() function along with the length of the string.
 *  @param filep A pointer to a file object
 *  @param buffer The buffer to that contains the string to write to the device
 *  @param len The length of the array of data that is being passed in the const char buffer
 *  @param offset The offset if required
 */
static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset){
   char str_buf[128]={0};
   strncpy ( str_buf, buffer, len );
   printk(KERN_INFO LOG_PREFIX "Received %zu characters from the user: %s", len, str_buf);
   /*
   struct timespec current_time_ts;
   getnstimeofday (&current_time_ts);
   uint32_t current_time_s  = current_time_ts.tv_sec;
   uint32_t current_time_us = current_time_ts.tv_nsec / USEC_TO_NSEC;
   */
   if(strlen(CONTROL_CLEAR_STRING)==len && strncmp(buffer, CONTROL_CLEAR_STRING, len)==0)
   {
       printk(KERN_INFO LOG_PREFIX "Clear list");
       hrtimer_try_to_cancel(&timer_next_open_event);
       hrtimer_try_to_cancel(&timer_next_close_event);
       remove_passed(0xFFFFFFFF,0xFFFFFFFF);
       //TODO: if(in_open_state)close();
       return len;
   }
   
   event_element_t new_event;
   if(sscanf(buffer,"%u.%06u %u.%06u",
                  &new_event.open_time_s,  &new_event.open_time_us,
                  &new_event.close_time_s, &new_event.close_time_us
      )!=4)
   {
		printk(KERN_WARNING LOG_PREFIX "Wrong arguments\n");
		return -EINVAL;
   }

   printk(KERN_INFO LOG_PREFIX "+ Timeout add [%u.%06u]-[%u.%06u]" , 
					new_event.open_time_s, new_event.open_time_us,
					new_event.close_time_s, new_event.close_time_us
					);

   event_add(new_event);
   update_timer();
   return len;
}
 
/** @brief The device release function that is called whenever the device is closed/released by
 *  the userspace program
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_release(struct inode *inodep, struct file *filep){
   //printk(KERN_INFO LOG_PREFIX "Device successfully closed\n");
   return 0;
}
static char* ptime_devnode(struct device *dev, umode_t *mode)
{
        if (!mode)
                return NULL;
        if (dev->devt == MKDEV(major_number, 0))
                *mode = 0666;
        return NULL;
}
static int __init ptime_control_init(void) {
   printk(KERN_INFO LOG_PREFIX "--- Control started\n");

   /* Register major number */
   major_number = register_chrdev(0, DEVICE_NAME, &fops);
   if (major_number<0){
      printk(KERN_ALERT "PTime failed to register a major number\n");
      return major_number;
   }
   //printk(KERN_INFO LOG_PREFIX "registered correctly with major number %d\n", major_number);
 
   /* Register the device class */
   ptime_class = class_create(THIS_MODULE, CLASS_NAME);
   if (IS_ERR(ptime_class)){
      unregister_chrdev(major_number, DEVICE_NAME);
      printk(KERN_ALERT LOG_PREFIX "Failed to register device class\n");
      return PTR_ERR(ptime_class);
   }
   ptime_class->devnode = ptime_devnode;
   //printk(KERN_INFO LOG_PREFIX "device class registered correctly\n");
 
   /* Register the device driver */
   ptime_device = device_create(ptime_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
   if (IS_ERR(ptime_device)){
      class_destroy(ptime_class);
      unregister_chrdev(major_number, DEVICE_NAME);
      printk(KERN_ALERT LOG_PREFIX "Failed to create the device\n");
      return PTR_ERR(ptime_device);
   }
   //printk(KERN_INFO LOG_PREFIX "device created correctly\n");
 
   /* Init high resolution timer */
   hrtimer_init (& timer_next_open_event, CLOCK_REALTIME, HRTIMER_MODE_ABS);
   timer_next_open_event.function = function_timer_open;
   hrtimer_init (& timer_next_close_event, CLOCK_REALTIME, HRTIMER_MODE_ABS);
   timer_next_close_event.function = function_timer_close;
   
   if(pwm_init(ptime_device)<0){
      printk(KERN_WARNING LOG_PREFIX "Failed init PWM\n");
      ptime_control_exit();
      return -1;
   }
   timer_setup(&pwm_off_timer, pwm_off, 0);
        
   return 0;
}

static void __exit ptime_control_exit(void) {
	device_destroy(ptime_class, MKDEV(major_number, 0));
	class_unregister(ptime_class);
	class_destroy(ptime_class);
	unregister_chrdev(major_number, DEVICE_NAME);
    del_timer(&pwm_off_timer);
	printk(KERN_INFO LOG_PREFIX "--- Control stopped\n");
}

module_init(ptime_control_init);
module_exit(ptime_control_exit);

