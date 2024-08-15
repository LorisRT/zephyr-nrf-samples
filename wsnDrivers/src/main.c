/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 Doc. for identifier: 
 https://stackoverflow.com/questions/75908381/cannot-get-device-binding-in-zephyr

 Doc for phandle (e.i.: property of node):
 https://docs.zephyrproject.org/latest/build/dts/phandles.html

 Doc. for function call:
 https://stackoverflow.com/questions/35186290/c-struct-object-stack-function-call-is-not-allowed-in-constant-expression-err
*/

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <hal/nrf_gpio.h>

#define SUCCESS 0


/* CODE EXECUTION DEFINE VARIABLERS */
#define LAUNCH_CODE_DEVACADEMY_EXAMPLE_REQUIRED false
#define LAUNCH_CODE_WSN_0_EXAMPLE_REQUIRED true
#define LAUNCH_CODE_WSN_1_EXAMPLE_REQUIRED false



/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000
#define LIMIT_COUNT_BLINKING 1000



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



/* Define for LAUNCH_CODE_WSN_1_EXAMPLE_REQUIRED */
#define ERROR_MESSAGE_GPIO_CONFIG "Could not configure GPIO in LAUNCH_CODE_WSN1_EXAMPLE"
#define GPIO_NODE_ID DT_NODELABEL(gpio0)
#define GPIO_NAME DEVICE_DT_NAME(GPIO_NODE_ID)

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

	/*  */
	if (LAUNCH_CODE_WSN_1_EXAMPLE_REQUIRED){
		printk(GPIO_NAME);
		while(true);
		gpio_led_wsn_1 = device_get_binding(GPIO_NAME);
		// if (SUCCESS != (gpio_led_wsn_1, GPIO_PIN_13_LED1, GPIO_OUTPUT)){ //WARNINGGGGGGGGGGGGGG
		// 	printk(ERROR_MESSAGE_GPIO_CONFIG);
		// 	return 0;
		// }

		gpio_pin_set(gpio_led_wsn_1, GPIO_PIN_13_LED1, 1);

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

	/* Shoudl never reach here */
	return 0;
}
