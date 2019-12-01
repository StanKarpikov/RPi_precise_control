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

/*--------------------------------------------------------------
			DEFINES
---------------------------------------------------------------*/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Stanislav Karpikov");
MODULE_DESCRIPTION("GPIO and PWM control based on the times list");
MODULE_VERSION("0.0.1");

#define DEVICE_NAME "ptime_control"
#define CLASS_NAME "chardrv"

/*--------------------------------------------------------------
			PRIVATE FUNCTIONS PROTOTYPES
---------------------------------------------------------------*/

static enum hrtimer_restart function_timer(struct hrtimer *);

static int     dev_open(struct inode *, struct file *);
static int     dev_release(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char *, size_t, loff_t *);

/*--------------------------------------------------------------
			PRIVATE TYPES
---------------------------------------------------------------*/

typedef struct{
  uint64_t start_time_ns;
  uint64_t stop_time_ns;
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
 
struct rb_root event_list_root = RB_ROOT;
static struct hrtimer timer_next_event;

/*--------------------------------------------------------------
					PRIVATE FUNCTIONS
---------------------------------------------------------------*/

/** @brief Insert new event list element into tree
 */
int event_list_insert(struct rb_root *root, struct event_list_element_s *new_event)
{
  struct event_list_element_s *check_node;
  struct rb_node **link = &(root->rb_node), *parent = NULL;
	
  /* Figure out where to put new node */
  while (*link) 
  {
	  parent = *link;
	  check_node = rb_entry(parent, struct event_list_element_s, node);
	  
	  if (check_node->key.start_time_ns > new_event->key.start_time_ns)
	  {
		  link = &((*link)->rb_left);
	  }else{
		  link = &((*link)->rb_right);
	  }
  }

  /* Add new node and rebalance tree. */
  rb_link_node(&new_event->node, parent, link);
  rb_insert_color(&new_event->node, root);

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
  printk(KERN_INFO "Add event [%lldns]-[%lldns]", new_event.start_time_ns, new_event.stop_time_ns);
  
  return 1;
}

/** @brief Remove passed timeouts
 */
static void remove_passed(uint64_t before_time_ns)
{
	struct event_list_element_s *minimum_time_element = 0;
	uint64_t next_timeout_ns = 0;
	do
	{
		struct rb_node *minimum_time_node=rb_first(&event_list_root);
		if(!minimum_time_node)break;
		
		minimum_time_element = rb_entry(minimum_time_node, struct event_list_element_s, node);
		if(!minimum_time_element)break;

		next_timeout_ns = minimum_time_element->key.start_time_ns;			
		if(next_timeout_ns < before_time_ns)
		{			
			printk(KERN_INFO "Remove event at [%lldns]", next_timeout_ns);
			rb_erase(minimum_time_node, &event_list_root);
			kfree(minimum_time_element);
		}
	}while(next_timeout_ns < before_time_ns);
}

/** @brief Extract next timeout and restart high resolution timer
 */
static void update_timer(void)
{
	/* Get next closest time */
	struct event_list_element_s *next_time_node = 0;
	uint64_t next_timeout_ns = 0;
	struct rb_node *next_time=rb_first(&event_list_root);
	if(!next_time)return;
	
	next_time_node = rb_entry(next_time, struct event_list_element_s, node);
	if(!next_time_node)return;
	
    next_timeout_ns = next_time_node->key.start_time_ns;

	printk(KERN_INFO "Update timer to [%lldns]", next_timeout_ns);

	hrtimer_cancel(&timer_next_event);
	ktime_t timer_next_close = ns_to_ktime(next_timeout_ns);
	//hrtimer_forward_now(& timer_next_event, timer_next_close);
	hrtimer_start(&timer_next_event, timer_next_close, HRTIMER_MODE_ABS);
}

/** @brief High resolution timer callback function
 */
static enum hrtimer_restart function_timer(struct hrtimer * unused)
{
	ktime_t current_time = ktime_get();
	//gpio_set_value(GPIO_OUTPUT,1);
	printk(KERN_INFO "Timeout interrupt: %ld [%lldns]", jiffies, ktime_to_ns(current_time));
	remove_passed(ktime_to_ns(current_time));
	update_timer();
	//hrtimer_cancel(& htimer);

	return HRTIMER_NORESTART;
}

/** @brief The device open function that is called each time the device is opened
 *  This will only increment the numberOpens counter in this case.
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_open(struct inode *inodep, struct file *filep){
   printk(KERN_INFO "PTime: Device opened\n");
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
  printk(KERN_INFO "PTime: Trying to read\n");
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
   printk(KERN_INFO "PTime: Received %zu characters from the user\n", len);
   
   uint32_t delta=0;
   sscanf(buffer,"%u",&delta);
   
   ktime_t current_time = ktime_get();
   current_time = ktime_add_us(current_time, delta*1000);
   printk(KERN_INFO "Timeout add: [%lldns]", ktime_to_ns(current_time));
   
   event_element_t new_event;
   new_event.start_time_ns = ktime_to_ns(current_time);
   new_event.stop_time_ns = ktime_add_us(current_time, 3000);
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
   printk(KERN_INFO "PTime: Device successfully closed\n");
   return 0;
}

static int __init ptime_control_init(void) {
   printk(KERN_INFO "PTime control started\n");

   /* Register major number */
   major_number = register_chrdev(0, DEVICE_NAME, &fops);
   if (major_number<0){
      printk(KERN_ALERT "PTime failed to register a major number\n");
      return major_number;
   }
   printk(KERN_INFO "PTime: registered correctly with major number %d\n", major_number);
 
   /* Register the device class */
   ptime_class = class_create(THIS_MODULE, CLASS_NAME);
   if (IS_ERR(ptime_class)){
      unregister_chrdev(major_number, DEVICE_NAME);
      printk(KERN_ALERT "PTime: Failed to register device class\n");
      return PTR_ERR(ptime_class);
   }
   printk(KERN_INFO "PTime: device class registered correctly\n");
 
   /* Register the device driver */
   ptime_device = device_create(ptime_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
   if (IS_ERR(ptime_device)){
      class_destroy(ptime_class);
      unregister_chrdev(major_number, DEVICE_NAME);
      printk(KERN_ALERT "PTime: Failed to create the device\n");
      return PTR_ERR(ptime_device);
   }
   printk(KERN_INFO "PTime: device created correctly\n");
 
   /* Init high resolution timer */
   hrtimer_init (& timer_next_event, CLOCK_REALTIME, HRTIMER_MODE_REL);
   timer_next_event.function = function_timer;

   return 0;
}

static void __exit ptime_control_exit(void) {
	device_destroy(ptime_class, MKDEV(major_number, 0));
	class_unregister(ptime_class);
	class_destroy(ptime_class);
	unregister_chrdev(major_number, DEVICE_NAME);
	printk(KERN_INFO "PTime control stopped\n");
}

module_init(ptime_control_init);
module_exit(ptime_control_exit);

