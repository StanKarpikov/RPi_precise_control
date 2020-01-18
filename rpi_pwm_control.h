
#ifndef __RPI_PWM_CONTROL
#define __RPI_PWM_CONTROL

/*--------------------------------------------------------------
                      PUBLIC TYPES
---------------------------------------------------------------*/

/** PWM channel codes */
typedef enum{
	PWM_CHANNEL_0 = 0,
	PWM_CHANNEL_1 = 1
}pwm_channel_t;

/** GPIO codes for internal use */
enum{
	PWM0_GPIO_12 = 0,
	PWM0_GPIO_18,
	PWM0_GPIO_40,
	PWM0_GPIO_52,
	PWM1_GPIO_13,
	PWM1_GPIO_19,
	PWM1_GPIO_41,
	PWM1_GPIO_45,
	PWM1_GPIO_53,
	PWM_GPIO_SIZE
};

/*--------------------------------------------------------------
            PUBLIC FUNCTION PROTOTYPES
---------------------------------------------------------------*/

/** @brief Set hardware PWM pulse width
 *  @param pwm_channel - PWM channels
 *  @param width - Servo pulse width units are .01 msec (E.g. so width = 150 is 1.5 msec)
 *  @param invert - Invert channel
 */
int pwm_pulse_width(pwm_channel_t pwm_channel, int width, bool invert);

/** @brief Set GPIO alt function
 *  @param pin - GPIO number
 *  @param altfn - Function code
 */
void gpio_alt_function(uint8_t pin, uint8_t altfn);

/** @brief Set GPIO output mode
 *  @param pin - GPIO number
 *  @param mode - 0:input 1:output
 */
void gpio_set_mode(uint8_t pin, int mode);

/** @brief Read GPIO pin
 *  @param pin - GPIO number
 *  @return GPIO state
 */
int gpio_read(int pin);

/** @brief Write GPIO pin
 *  @param pin - GPIO number
 *  @param level - GPIO state
 */
void gpio_write(uint8_t pin, int level);

/** @brief Initialization function
 */
int pwm_init(struct device *dev);

#endif /* __RPI_PWM_CONTROL */
