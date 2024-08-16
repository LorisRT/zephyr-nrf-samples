/**
 ******************************************************************************
 * @file           : main.c
 * @author         : LorisRT
 * @brief          : driver and sample developpement for nRF52840 with zephir
 ******************************************************************************
 * @attention
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/*
 Doc. for identifier: 
 https://stackoverflow.com/questions/75908381/cannot-get-device-binding-in-zephyr

 Doc for phandle (e.i.: property of node):
 https://docs.zephyrproject.org/latest/build/dts/phandles.html

 Doc. for function call:
 https://stackoverflow.com/questions/35186290/c-struct-object-stack-function-call-is-not-allowed-in-constant-expression-err

 Doc. for GPIO with devicetree:
 https://docs.nordicsemi.com/bundle/ncs-latest/page/zephyr/hardware/peripherals/gpio.html#structgpio__dt__spec

 Doc. for I2C example from Nordic artile:
 https://devzone.nordicsemi.com/guides/nrf-connect-sdk-guides/b/peripherals/posts/twi-ic2-implementation-with-nrfx-twis-driver
*/

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <hal/nrf_gpio.h>
#include <zephyr/drivers/i2c.h>

#include "app_mpu6050.h"

#define SUCCESS 0


/* CODE EXECUTION DEFINE VARIABLERS */
#define LAUNCH_CODE_DEVACADEMY_EXAMPLE_REQUIRED false
#define LAUNCH_CODE_WSN_0_EXAMPLE_REQUIRED false
#define LAUNCH_CODE_WSN_1_EXAMPLE_REQUIRED false
#define LAUNCH_CODE_WSN_2_EXAMPLE_REQUIRED false
#define LAUNCH_CODE_WSN_I2C_EXAMPLE_REQUIRED false
#define LAUNCH_CODE_I2C_FOR_MPU6050_REQUIRED true

#define END_MESSAGE_FOR_EXAMPLES "End of code reach successfully\n"
#define END_MESSAGE_UNEXPECTED_OUTPUT "Reached end return, unexpected error in code"


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000
#define LIMIT_COUNT_BLINKING 1000



/* MPU6050 REGISTER DEFINITION */
#define MPU6050_WHO_AM_I 0x75


/* Define for LAUNCH_CODE_WDN_0_EXAMPLE_REQUIRED */
#define P0_BASE_ADDRESS 0x50000000
#define GPIO_OUT_OFFSET_REG 0x504
#define GPIO_OUTSET_OFFSET_REG 0x508
#define GPIO_DIR_OFFSET_REG 0x514
#define GPIO_DIRSET_OFFSET_REG 0x518
#define CLOCK_BASE_ADDRESS 0x40000000
#define CLOCK_HFCLKSTAT_OFFSET_REG 0x40c
#define CLOCK_LFCLKSTAT_OFFSET_REG 0x418
volatile uint32_t *p_gpio_out_reg = (volatile uint32_t *) (P0_BASE_ADDRESS + GPIO_OUT_OFFSET_REG);
volatile uint32_t *p_gpio_outset_reg = (volatile uint32_t *) (P0_BASE_ADDRESS + GPIO_OUTSET_OFFSET_REG);
volatile uint32_t *p_gpio_dir_reg = (volatile uint32_t *) (P0_BASE_ADDRESS + GPIO_DIR_OFFSET_REG);
volatile uint32_t *p_gpio_dirset_reg = (volatile uint32_t *) (P0_BASE_ADDRESS + GPIO_DIRSET_OFFSET_REG);
volatile uint32_t *p_clk_hfclkstat = (volatile uint32_t *) (CLOCK_BASE_ADDRESS + CLOCK_HFCLKSTAT_OFFSET_REG);
volatile uint32_t *p_clk_lfclkstat = (volatile uint32_t *) (CLOCK_BASE_ADDRESS + CLOCK_LFCLKSTAT_OFFSET_REG);
#define GPIO_PIN_13_LED1 13



/* Define for LAUNCH_CODE_WSN_I2C_EXAMPLE_REQUIRED */
#define ERROR_MESSAGE_WRITE_FAIL_IN_I2C "Could not perform a I2C write for the LAUNCH_CODE_WSN_I2C_EXAMPLE"
#define I2C0_NODE DT_NODELABEL(mpu6050)
static const struct i2c_dt_spec i2c_device_mpu6050 = I2C_DT_SPEC_GET(I2C0_NODE);



/* Define for LAUNCH_CODE_WSN_2_EXAMPLE_REQUIRED */
// Doc: https://docs.nordicsemi.com/bundle/ncs-latest/page/zephyr/hardware/peripherals/gpio.html#group__gpio__interface_1ga2fa6bb5880f46984f9fc29c70f7d503e
#define LED0_DK DT_ALIAS(led0)
// - GPIO_DT_SPEC_GET is a static initialiser for a struct gpio_dt_spec:
// requires: node_id and property name (see dts file for gpio led0 node)
// - gpio_dt_spec is a container (struct in C) for GPIO pin information speficied in dt
static const struct gpio_dt_spec ledFromDK = GPIO_DT_SPEC_GET(LED0_DK, gpios);
#define LED_ON_DT 1



/* Define for LAUNCH_CODE_WSN_1_EXAMPLE_REQUIRED */
#define ERROR_MESSAGE_GPIO_CONFIG "Could not configure GPIO in LAUNCH_CODE_WSN1_EXAMPLE\n"
#define ERROR_MESSAGE_GPIO_SET "Could not set GPIO in LAUNCH_CODE_WSN1_EXAMPLE\n"
#define GPIO_NODE_ID DT_NODELABEL(gpio0)
#define GPIO_NAME DEVICE_DT_NAME(GPIO_NODE_ID)
#define LED1_ON 0
#define LED1_OFF 1
const struct device *gpio_led_wsn_1;



/* Define for LAUNCH_CODE_DEVACADEMY_EXAMPLE_REQUIRED */
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);



int main(void)
{
	/* Extract MPU6050 inertial data with i2c protocol */
	if (LAUNCH_CODE_I2C_FOR_MPU6050_REQUIRED){
		
	}

	/* First example of i2c use with nrf and zephyr functions */
	if (LAUNCH_CODE_WSN_I2C_EXAMPLE_REQUIRED){
		if (false == device_is_ready(i2c_device_mpu6050.bus)){
			printk("I2C device not ready\n");
			return 0;
		}

		uint8_t config[1] = {MPU6050_WHO_AM_I};
		
		if (SUCCESS != i2c_write_dt(&i2c_device_mpu6050, config, sizeof(config))){
			printk(ERROR_MESSAGE_WRITE_FAIL_IN_I2C);
			return 0;
		}

		printk(END_MESSAGE_FOR_EXAMPLES);
		while(true);
	}

	/* LED1 from nRF52840 DK blinking example */
	if (LAUNCH_CODE_WSN_2_EXAMPLE_REQUIRED){
		if (false == gpio_is_ready_dt(&ledFromDK)){
			return 0;
		}
		if (SUCCESS != gpio_pin_configure_dt(&ledFromDK, GPIO_OUTPUT)){
			return 0;
		}
		// WARNING: LED ON from dt functions is set with 1 (not 0 as opposed to other example)
		if (SUCCESS != gpio_pin_set_dt(&ledFromDK, LED_ON_DT)){
			return 0;
		}
		printk(END_MESSAGE_FOR_EXAMPLES);
		while(true);
	}

	/* Original blinking LED example for nRF52840 DK */
	if (LAUNCH_CODE_DEVACADEMY_EXAMPLE_REQUIRED){
		int ret;
		int count = 0;

		if (!gpio_is_ready_dt(&led)) {
			return 0;
		}

		ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			return 0;
		}

		while (1) {
			ret = gpio_pin_toggle_dt(&led);
			if (ret < 0) {
				return 0;
			}
			k_msleep(SLEEP_TIME_MS);
			printk("Blink n°%d\n", ++count);
			if (count > LIMIT_COUNT_BLINKING){
				count = 0;
			}
		}
	}    

	/* LED1 from nRF52840 DK blinking example */
	if (LAUNCH_CODE_WSN_1_EXAMPLE_REQUIRED){
		//printk(GPIO_NAME);
		gpio_led_wsn_1 = device_get_binding(GPIO_NAME);
		if (SUCCESS != gpio_pin_configure(gpio_led_wsn_1, GPIO_PIN_13_LED1, GPIO_OUTPUT)){
			printk(ERROR_MESSAGE_GPIO_CONFIG);
			return 0;
		}
		if (SUCCESS != gpio_pin_set(gpio_led_wsn_1, GPIO_PIN_13_LED1, LED1_ON)){
			printk(ERROR_MESSAGE_GPIO_SET);
			return 0;
		}
		printk(END_MESSAGE_FOR_EXAMPLES);
		while(true);
	}

	/* Baremetal programming of LED1 blinking example */
	if (LAUNCH_CODE_WSN_0_EXAMPLE_REQUIRED){

		printk("Enters WSN_0 condition\n");

		/* Check CLK status */
		printk("HFCLKSTAT reg: 0x%08x\n", *p_clk_hfclkstat);
		printk("LFCLKSTAT reg: 0x%08x\n", *p_clk_lfclkstat);

		/* Configure GPIO */
		*p_gpio_dir_reg |= (1 << GPIO_PIN_13_LED1);
		*p_gpio_out_reg |= (0 << GPIO_PIN_13_LED1);
		printk("register gpio_dir_reg configured as: 0x%08x\n", *p_gpio_dir_reg);
		printk("register gpio_out_reg configured as: 0x%08x\n", *p_gpio_out_reg);

		while(true);
	}

	/* Should never reach here */
	printk(END_MESSAGE_UNEXPECTED_OUTPUT);
	return 0;
}
