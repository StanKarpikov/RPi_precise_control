/* Original file from the link:
 *
 * https://www.raspberrypi.org/forums/viewtopic.php?t=231994
 *
 * */

/*--------------------------------------------------------------
                      INCLUDES
---------------------------------------------------------------*/

#include <linux/types.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/ioport.h>
#include <asm/uaccess.h>
#include <linux/delay.h>


#include "rpi_pwm_control.h"

/*--------------------------------------------------------------
                      DEFINES
---------------------------------------------------------------*/
/* macros to get at IO space when running virtually */
#define IO_ADDRESS(x)	(((x) & 0x0fffffff) + (((x) >> 4) & 0x0f000000) + 0xf0000000)

#define __io_address(a)     __io(IO_ADDRESS(a))

#define PI_1_PERIPHERAL_BASE    0x20000000
#define PI_2_PERIPHERAL_BASE    0x3F000000

#define GPIO_BASE    0x200000
#define PWM_BASE     0x20C000
#define CLOCK_BASE   0x101000


/* uint32_t pointer offsets to registers in the PWM address map are byte
|  offset / 4.
*/
#define PWM_CTL_REG          (0x0 / 4)
#define CTL_REG_RESET_STATE  0
#define CTL_REG_PWM1_ENABLE  1
#define CTL_REG_MSEN1        0x80
#define CTL_REG_PWM1_MS_MODE (CTL_REG_PWM1_ENABLE | CTL_REG_MSEN1)
#define CTL_REG_PWM2_ENABLE  0x100
#define CTL_REG_MSEN2        0x8000
#define CTL_REG_PWM2_MS_MODE (CTL_REG_PWM2_ENABLE | CTL_REG_MSEN2)

#define PWM_RNG1_REG    (0x10 / 4)
#define PWM_DAT1_REG    (0x14 / 4)
#define PWM_RNG2_REG    (0x20 / 4)
#define PWM_DAT2_REG    (0x24 / 4)

#define PWM_CLOCK_HZ    19200000.0
#define PWM_RESOLUTION  0.000005        // 5 usec resolution
#define PWM_MSEC_TO_COUNT(ms)   ((ms) / PWM_RESOLUTION / 1000.0)

#define PULSE_WIDTH_RESOLUTION  .00001  // .01 msec resolution

/* PWM clock manager registers are not in the BCM2835-ARM-Peripherals pdf,
|  but are defined (as byte addresses CM_PWMDIV & CM_PWMCTL) in the addendum:
|      http://www.scribd.com/doc/127599939/BCM2835-Audio-clocks
|  These REG defines here give uint32_t pointer offsets from clock base.
*/
#define CM_PASSWORD  0x5A000000
#define CM_PWMCTL_REG      (0xa0 / 4)
#define    PWMCTL_BUSY     0x80     // Read only
#define    PWMCTL_KILL     0x20
#define    PWMCTL_ENABLE   0x10
#define    PWMCTL_SRC_OSC  0x1
#define CM_PWMDIV_REG      (0xa4 / 4)
#define    PWMDIV_DIVI(divi) (divi << 12)   // bits 23-12, max 4095

#define GPSET_REG     (0x1c / 4)
#define GPCLR_REG     (0x28 / 4)
#define GPLEV_REG     (0x34 / 4)
#define GPPUD_REG     (0x94 / 4)
#define   PUD_DOWN    1
#define   PUD_UP      2
#define GPPUDCLK_REG  (0x98 / 4)

#define MAX(a,b)    (((a) > (b)) ? (a) : (b))
#define MIN(a,b)    (((a) < (b)) ? (a) : (b))

/*--------------------------------------------------------------
                    PRIVATE VARIABLES
---------------------------------------------------------------*/

/** Pointer to mapped GPIO peripheral register.*/
static volatile uint32_t *gpio_mmap;
/** Pointer to mapped PWM peripheral register.*/
static volatile uint32_t *pwm_mmap;
/** Pointer to mapped Clock peripheral register.*/
static volatile uint32_t *clock_mmap;

/** PWM pin that is used to output */
static int servo_gpio  = PWM0_GPIO_18;

/*
Raspberry Pi PWM map:

PWM0: 12,4(Alt0) 18,2(Alt5) 40,4(Alt0)            52,5(Alt1)
PWM1: 13,4(Alt0) 19,2(Alt5) 41,4(Alt0) 45,4(Alt0) 53,5(Alt1)
*/
/** Alt function codes for PWM GPIO in enum */
static const uint8_t alt_codes[PWM_GPIO_SIZE]={0,5,0,1,0,5,0,0,1};
/** GPIO pin numbers for PWM GPIO in enum */
static const uint8_t gpio_num[PWM_GPIO_SIZE] ={12,18,40,52,13,19,41,45,53};

/** BCM2835 ARM Peripherals pg 91, 10 gpios per gpfsel with mode bits */
static unsigned int gpfsel_mode_table[] =
{
    /* in    out   alt0   alt1   alt2   alt3   alt4   alt5 */
    0b000, 0b001, 0b100, 0b101, 0b110, 0b111, 0b011, 0b010
};
/*--------------------------------------------------------------
                    PRIVATE FUNCTIONS
---------------------------------------------------------------*/

/** @brief BCM2835 ARM Peripherals pg 101 - PUD sequence
 *  @param pin - Pin number
 *  @param pud - Pull Up
 */
static void __attribute__ ((unused)) gpio_set_pud(uint8_t pin, int pud)
{
    int  gp_reg = GPPUDCLK_REG + ((pin > 31) ? 1 : 0);

    if (pud != PUD_DOWN && pud != PUD_UP)
        return;
    *(gpio_mmap + GPPUD_REG) = pud;
    udelay(2);          // min wait of 150 cycles
    *(gpio_mmap + gp_reg) = 1 << (pin & 0x1f);
    udelay(2);
    *(gpio_mmap + GPPUD_REG) = 0;
    *(gpio_mmap + gp_reg) = 0;
}

/** @brief Initialize Peripherals
 *  @param peripheral_base - Address base
 */
static int init_peripherals(int peripheral_base, struct device *dev)
{
    uint32_t divi;
    /*
    #define DEVICE_NAME "ptime_control"

    printk(KERN_INFO "dev: 0x%x", (uint32_t)dev); 
    
    if(devm_request_mem_region(dev, PI_2_PERIPHERAL_BASE + GPIO_BASE, 0x100, DEVICE_NAME ) == NULL )
    {
        printk( KERN_ALERT
            "error:%s: unable to obtain I/O memory address 0x%x\n",
            DEVICE_NAME, PI_2_PERIPHERAL_BASE + GPIO_BASE );

        return -1;
    }*/
  
    gpio_mmap = devm_ioremap_nocache(dev, PI_2_PERIPHERAL_BASE + GPIO_BASE, 0x10000);
    clock_mmap = devm_ioremap_nocache(dev, PI_2_PERIPHERAL_BASE + CLOCK_BASE, 0x10000);
    pwm_mmap = devm_ioremap_nocache(dev, PI_2_PERIPHERAL_BASE + PWM_BASE, 0x10000);
    
    if(!gpio_mmap) { printk(KERN_INFO "gpio_mmap failed" ); return -1;}
    if(!pwm_mmap)  { printk(KERN_INFO "pwm_mmap failed"  ); return -1;}
    if(!clock_mmap){ printk(KERN_INFO "clock_mmap failed"); return -1;}

    gpio_alt_function(gpio_num[servo_gpio], alt_codes[servo_gpio]);

    /* Kill clock (waiting for busy flag does not work)
    */
    *(clock_mmap + CM_PWMCTL_REG) = CM_PASSWORD | PWMCTL_KILL;
    udelay(10);

    /* PWM clock is 19.2MHz. Set the divisor so each count gives the resolution
    |  we want.
    */
    divi = (uint32_t) (PWM_CLOCK_HZ * PWM_RESOLUTION);
    *(clock_mmap + CM_PWMDIV_REG)  = CM_PASSWORD | PWMDIV_DIVI(divi);

    *(clock_mmap + CM_PWMCTL_REG) = CM_PASSWORD | PWMCTL_ENABLE | PWMCTL_SRC_OSC;

    /* Turn off PWM - reset state.
    */
    *(pwm_mmap + PWM_CTL_REG) = CTL_REG_RESET_STATE;
    udelay(50);

    /* E.g. Range of 20 msec -> 4000 counts at 5 usec PWM resolution */
    *(pwm_mmap + PWM_RNG1_REG) = (uint32_t) PWM_MSEC_TO_COUNT(20);
    *(pwm_mmap + PWM_RNG2_REG) = (uint32_t) PWM_MSEC_TO_COUNT(20);

    /* PWM channels run in M/S mode so pulse width is count of data and
    |  period is count of range.
    */
    *(pwm_mmap + PWM_CTL_REG) = CTL_REG_PWM1_MS_MODE | CTL_REG_PWM2_MS_MODE;
    return 1;
}

/*--------------------------------------------------------------
                    PUBLIC FUNCTIONS
---------------------------------------------------------------*/

/** @brief Set hardware PWM pulse width
 *  @param pwm_channel - PWM channels
 *  @param width - Servo pulse width units are .01 msec (E.g. so width = 150 is 1.5 msec)
 *  @param invert - Invert channel
 */
int pwm_pulse_width(pwm_channel_t pwm_channel, int width, bool invert)
{
    uint32_t count;
    int      reg;

    if (pwm_channel == PWM_CHANNEL_0)
        reg = PWM_DAT1_REG;
    else if (pwm_channel == PWM_CHANNEL_1)
        reg = PWM_DAT2_REG;
    else
        return -1;

    if (invert)width = 300 - width;    // 150 msec is center

    count = (uint32_t)(PULSE_WIDTH_RESOLUTION / PWM_RESOLUTION) * width;
    if (count > PWM_MSEC_TO_COUNT(3.0))
        count = PWM_MSEC_TO_COUNT(3.0);
    if (count < PWM_MSEC_TO_COUNT(0.5))
        count = PWM_MSEC_TO_COUNT(0.5);

    *(pwm_mmap + reg) = count;

	return 1;
}

/** @brief Set GPIO alt function
 *  @param pin - GPIO number
 *  @param altfn - Function code
 */
void gpio_alt_function(uint8_t pin, uint8_t altfn)
{
    uint8_t  gpfsel = pin / 10;
    uint8_t  shift = (pin % 10) * 3;

    if (altfn >= 0 && altfn <= 5)
    {
        *(gpio_mmap + gpfsel) = (*(gpio_mmap + gpfsel) & ~(0x7 << shift))
                    | (gpfsel_mode_table[altfn + 2] << shift);
    }
}

/** @brief Set GPIO output mode
 *  @param pin - GPIO number
 *  @param mode - 0:input 1:output
 */
void gpio_set_mode(uint8_t pin, int mode)
{
    int  gpfsel = pin / 10,
         shift = (pin % 10) * 3;

    if (mode == 0 || mode == 1)
        *(gpio_mmap + gpfsel) = (*(gpio_mmap + gpfsel) & ~(0x7 << shift))
                    | (gpfsel_mode_table[mode] << shift);
}

/** @brief Read GPIO pin
 *  @param pin - GPIO number
 *  @return GPIO state
 */
int gpio_read(int pin)
{
    int  gp_reg = GPLEV_REG + ((pin > 31) ? 1 : 0);

    return (*(gpio_mmap + gp_reg) & (1 << (pin & 0x1f)) ? 1 : 0 );
}

/** @brief Write GPIO pin
 *  @param pin - GPIO number
 *  @param level - GPIO state
 */
void gpio_write(uint8_t pin, int level)
{
    int  gp_reg = ((level == 0) ? GPCLR_REG : GPSET_REG) + ((pin > 31) ? 1 : 0);

    *(gpio_mmap + gp_reg) = 1 << (pin & 0x1f);
}

/** @brief Initialization function
 */
int pwm_init(struct device *dev)
{
	int peripheral_base;

    peripheral_base = PI_2_PERIPHERAL_BASE;

	return init_peripherals(peripheral_base, dev);
}

#ifdef PWM_TEST
/** @brief Main function (test)
 */
int main(int argc, char **argv)
{
    char  buf[200];

    printf("uid:%d  euid:%d gid:%d\n", getuid(), geteuid(), getgid());

    if (getuid() != 0)
    {
		/* Restarts via sudo if run as user pi. */
        printf ("Restart as sudo\n");
        snprintf(buf, 200, "sudo %s", argv[0]);
        system(buf);
        exit(0);
    }
    printf("  uid:%d  euid:%d gid:%d\n", getuid(), geteuid(), getgid());

    pwm_init();

    setgid(27);
    setuid(1000);

    printf("  uid:%d  euid:%d gid:%d\n", getuid(), geteuid(), getgid());

    while (1)
    {
        pwm_pulse_width(PWM_CHANNEL_0, 150, servo_invert); /* 1.5 ms */
        sleep(3);
        pwm_pulse_width(PWM_CHANNEL_0, 100, servo_invert); /* 1.0 ms */
        sleep(3);
    }
    return 0;
}
#endif
