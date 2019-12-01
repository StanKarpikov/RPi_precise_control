/* File from the link:
 * 
 * https://www.raspberrypi.org/forums/viewtopic.php?t=231994
 * 
 * */

// cc pwm.c -Wall -o pwm
// ./pwm
// Restarts via sudo if run as user pi.


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>


#define PI_1_PERIPHERAL_BASE    0x20000000
#define PI_2_PERIPHERAL_BASE    0x3F000000

#define GPIO_BASE    0x200000
#define PWM_BASE     0x20C000
#define CLOCK_BASE   0x101000


/* uint32_t pointer offsets to registers in the PWM address map are byte
|  offset / 4.
*/
#define PWM_CTL_REG              (0x0 / 4)
#define     CTL_REG_RESET_STATE  0
#define     CTL_REG_PWM1_ENABLE  1
#define     CTL_REG_MSEN1        0x80
#define     CTL_REG_PWM1_MS_MODE (CTL_REG_PWM1_ENABLE | CTL_REG_MSEN1)
#define     CTL_REG_PWM2_ENABLE  0x100
#define     CTL_REG_MSEN2        0x8000
#define     CTL_REG_PWM2_MS_MODE (CTL_REG_PWM2_ENABLE | CTL_REG_MSEN2)

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


/* Pointers to mapped peripheral registers.
*/
static volatile uint32_t *gpio_mmap;
static volatile uint32_t *pwm_mmap;
static volatile uint32_t *clock_mmap;


int servo_pan_gpio = 18;
int servo_tilt_gpio = 19;

int servo_pan_invert = 0;
int servo_tilt_invert = 0;


/* Servo pulse width units are .01 msec (E.g. so width = 150 is 1.5 msec)
 */
void
servo_pulse_width(int pwm_channel, int width, int invert)
    {
    uint32_t count;
    int      reg;

    if (pwm_channel == 1)
        reg = PWM_DAT1_REG;
    else if (pwm_channel == 2)
        reg = PWM_DAT2_REG;
    else
        return;

    if (invert)
        width = 300 - width;    // 150 msec is center

    count = (uint32_t)(PULSE_WIDTH_RESOLUTION / PWM_RESOLUTION) * width;
    if (count > PWM_MSEC_TO_COUNT(3.0))
        count = PWM_MSEC_TO_COUNT(3.0);
    if (count < PWM_MSEC_TO_COUNT(0.5))
        count = PWM_MSEC_TO_COUNT(0.5);

    *(pwm_mmap + reg) = count;
    }

void
gpio_to_channel(int gpio, int *channel, int *altfn)
    {
    int chan = -1, alt = -1;

    if (gpio == 12 || gpio == 18)
        {
        chan = 1;
        if (gpio == 18)
            alt = 5;
        }
    if (gpio == 13 || gpio == 19)
        {
        chan = 2;
        if (gpio == 19)
            alt = 5;
        }
    if (channel)
        *channel = chan;
    if (altfn)
        *altfn = alt;
    }


void
servo_move(int pan, int tilt, int delay)
    {
    static float pan_cur, tilt_cur;
    float        pan_inc, tilt_inc;
    int          pan_channel, tilt_channel;
    int          pan_delta, tilt_delta, max_delta, i;

    gpio_to_channel(servo_pan_gpio, &pan_channel, NULL);
    gpio_to_channel(servo_tilt_gpio, &tilt_channel, NULL);

    if (pan_cur == 0)
        pan_cur = (float)pan;
    if (tilt_cur == 0)
        tilt_cur = (float)tilt;

    pan_delta = pan - pan_cur;
    tilt_delta = tilt - tilt_cur;
    max_delta = MAX(abs(pan_delta), abs(tilt_delta));
    pan_inc = (float)max_delta / (float)pan_delta;
    tilt_inc = (float)max_delta / (float)tilt_delta;

    for (i = 1; i < max_delta && delay > 0; ++i)
        {
        pan_cur += pan_inc;
        tilt_cur += tilt_inc;
        servo_pulse_width(pan_channel, (int)pan_cur, servo_pan_invert);
        servo_pulse_width(tilt_channel, (int)tilt_cur, servo_tilt_invert);
        usleep(delay * 1000);
        }
    pan_cur = (float)pan;
    tilt_cur = (float)tilt;
    servo_pulse_width(pan_channel, pan, servo_pan_invert);
    servo_pulse_width(tilt_channel, tilt, servo_tilt_invert);
    }

  /* BCM2835 ARM Peripherals pg 91, 10 gpios per gpfsel with mode bits:
  */
static unsigned int gpfsel_mode_table[] =
    {
    /* in    out   alt0   alt1   alt2   alt3   alt4   alt5 */
    0b000, 0b001, 0b100, 0b101, 0b110, 0b111, 0b011, 0b010
    };

void
gpio_alt_function(int pin, int altfn)
    {
    int  gpfsel = pin / 10,
         shift = (pin % 10) * 3;

    if (altfn >= 0 && altfn <= 5)
        *(gpio_mmap + gpfsel) = (*(gpio_mmap + gpfsel) & ~(0x7 << shift))
                    | (gpfsel_mode_table[altfn + 2] << shift);
    }

void
gpio_set_mode(int pin, int mode)    /* mode 0:input 1:output */
    {
    int  gpfsel = pin / 10,
         shift = (pin % 10) * 3;

    if (mode == 0 || mode == 1)
        *(gpio_mmap + gpfsel) = (*(gpio_mmap + gpfsel) & ~(0x7 << shift))
                    | (gpfsel_mode_table[mode] << shift);
    }


  /* BCM2835 ARM Peripherals pg 101 - PUD sequence
  */
void
gpio_set_pud(int pin, int pud)
    {
    int  gp_reg = GPPUDCLK_REG + ((pin > 31) ? 1 : 0);

    if (pud != PUD_DOWN && pud != PUD_UP)
        return;
    *(gpio_mmap + GPPUD_REG) = pud;
    usleep(2);          // min wait of 150 cycles
    *(gpio_mmap + gp_reg) = 1 << (pin & 0x1f);
    usleep(2);
    *(gpio_mmap + GPPUD_REG) = 0;
    *(gpio_mmap + gp_reg) = 0;
    }

int
gpio_read(int pin)
    {
    int  gp_reg = GPLEV_REG + ((pin > 31) ? 1 : 0);

    return (*(gpio_mmap + gp_reg) & (1 << (pin & 0x1f)) ? 1 : 0 ); 
    }

void
gpio_write(int pin, int level)
    {
    int  gp_reg = ((level == 0) ? GPCLR_REG : GPSET_REG) + ((pin > 31) ? 1 : 0);

    *(gpio_mmap + gp_reg) = 1 << (pin & 0x1f);
    }

void
init_peripherals(int peripheral_base)
    {
    int      fd;
    uint32_t divi;

    if ((fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0)
        {
        fprintf(stderr, "/dev/mem open failed: %m\n");
        exit (-1);
        }
    gpio_mmap = (uint32_t *) mmap(NULL, 0x100, PROT_READ|PROT_WRITE, MAP_SHARED,
                        fd, peripheral_base + GPIO_BASE);
    pwm_mmap  = (uint32_t *) mmap(NULL, 0x100, PROT_READ|PROT_WRITE, MAP_SHARED,
                        fd, peripheral_base + PWM_BASE);
    clock_mmap = (uint32_t *) mmap(NULL, 0x100, PROT_READ|PROT_WRITE, MAP_SHARED,
                        fd, peripheral_base + CLOCK_BASE);

    close(fd);

    if (pwm_mmap == MAP_FAILED || clock_mmap == MAP_FAILED)
        exit(-1);

    gpio_alt_function(18, 5);
    gpio_alt_function(19, 5);

    /* Kill clock (waiting for busy flag does not work)
    */
    *(clock_mmap + CM_PWMCTL_REG) = CM_PASSWORD | PWMCTL_KILL;
    usleep(10);  

    /* PWM clock is 19.2MHz. Set the divisor so each count gives the resolution
    |  we want.
    */
    divi = (uint32_t) (PWM_CLOCK_HZ * PWM_RESOLUTION);
    *(clock_mmap + CM_PWMDIV_REG)  = CM_PASSWORD | PWMDIV_DIVI(divi);
    
    *(clock_mmap + CM_PWMCTL_REG) = CM_PASSWORD | PWMCTL_ENABLE | PWMCTL_SRC_OSC;

    /* Turn off PWM - reset state.
    */
    *(pwm_mmap + PWM_CTL_REG) = CTL_REG_RESET_STATE;
    usleep(50);

    /* E.g. Range of 20 msec -> 4000 counts at 5 usec PWM resolution */
    *(pwm_mmap + PWM_RNG1_REG) = (uint32_t) PWM_MSEC_TO_COUNT(20);
    *(pwm_mmap + PWM_RNG2_REG) = (uint32_t) PWM_MSEC_TO_COUNT(20);

    /* PWM channels run in M/S mode so pulse width is count of data and
    |  period is count of range.
    */
    *(pwm_mmap + PWM_CTL_REG) = CTL_REG_PWM1_MS_MODE | CTL_REG_PWM2_MS_MODE;
    }

int
pi_model(void)
    {
    FILE  *f;
    int   model = 1;
    char  buf[200], arm[32];

    if ((f = fopen("/proc/cpuinfo", "r")) == NULL)
        return 0;
    while (fgets(buf, sizeof(buf), f) != NULL)
        {
        if (sscanf(buf, "model name %*s %31s", arm) > 0)
            {
            if (!strcmp(arm, "ARMv7"))
                model = 2;
            break;
            }
        }
    fclose(f);
    return model;
    }

int
main(int argc, char **argv)
    {
    char  buf[200];
    int   model, peripheral_base;

    model = pi_model();
    if (model == 2)
        peripheral_base = PI_2_PERIPHERAL_BASE;
    else
        peripheral_base = PI_1_PERIPHERAL_BASE;

    printf("model: %d\n", model);
    printf("uid:%d  euid:%d gid:%d\n", getuid(), geteuid(), getgid());

    if (getuid() != 0)
        {
        snprintf(buf, 200, "sudo %s", argv[0]);
        system(buf);
        exit(0);
        }
    printf("  uid:%d  euid:%d gid:%d\n", getuid(), geteuid(), getgid());

    init_peripherals(peripheral_base);
    setgid(27);
    setuid(1000);

    printf("  uid:%d  euid:%d gid:%d\n", getuid(), geteuid(), getgid());

    while (1)
        {
        /* servo pulse width units are .01 msec
        */
        servo_move(150, 150, 0);
        sleep(1);
        servo_move(120, 120, 50);
        sleep(1);
        servo_move(180, 180, 10);
        sleep(1);
        servo_move(120, 160, 30);
        sleep(1);
#if 0
        servo_pulse_width(1, 150);
        servo_pulse_width(2, 150);
        sleep(1);
        servo_pulse_width(1, 100);
        servo_pulse_width(2, 200);
        sleep(1);
        for (i = 100; i <= 200; i += 1)
                {
                servo_pulse_width(1, i);
                servo_pulse_width(2, 300 - i);
                usleep(30000);
                }
        sleep(1);
#endif
        }
    return 0;
    }
