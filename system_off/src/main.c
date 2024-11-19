/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "retained.h"

#include <inttypes.h>
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/util.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   100
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
/* The devicetree node identifier for the "led0" alias. */
#define LED1_NODE DT_ALIAS(led1)

static const struct gpio_dt_spec led_blue = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led_red = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
// static const struct gpio_dt_spec sw0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

int main(void)
{
	int ret;

	if (!gpio_is_ready_dt(&led_blue)) {return 0;}
	if (!gpio_is_ready_dt(&led_red)) {return 0;}

	ret = gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {return 0;}

	ret = gpio_pin_configure_dt(&led_red, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {return 0;}

	ret = gpio_pin_set_dt(&led_red, 1);
	ret = gpio_pin_set_dt(&led_blue, 1);

	

	// int rc;
	// const struct device *const cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

	// if (!device_is_ready(cons)) {
	// 	printf("%s: device not ready.\n", cons->name);
	// 	return 0;
	// }

	//printf("\n%s Turning system off Demo\n", CONFIG_BOARD);

	if (IS_ENABLED(CONFIG_APP_RETENTION)) {
		bool retained_ok = retained_validate();
		ret = gpio_pin_set_dt(&led_blue, 0);
		ret = gpio_pin_set_dt(&led_red, 0);

		/* Increment for this boot attempt and update. */
		retained.boots += 1;
		retained_update();

		// printf("Retained data: %s\n", retained_ok ? "valid" : "INVALID");
		// printf("Boot count: %u\n", retained.boots);
		// printf("Off count: %u\n", retained.off_count);
		// printf("Active Ticks: %" PRIu64 "\n", retained.uptime_sum);
	} else {
		// printf("Retained data not supported\n");
		ret = gpio_pin_set_dt(&led_red, 1);
	}

	// /* configure sw0 as input, interrupt as level active to allow wake-up */
	// rc = gpio_pin_configure_dt(&sw0, GPIO_INPUT);
	// if (rc < 0) {
	// 	printf("Could not configure sw0 GPIO (%d)\n", rc);
	// 	return 0;
	// }

	// rc = gpio_pin_interrupt_configure_dt(&sw0, GPIO_INT_LEVEL_ACTIVE);
	// if (rc < 0) {
	// 	printf("Could not configure sw0 GPIO interrupt (%d)\n", rc);
	// 	return 0;
	// }
	//printf("Testing LEDs ...\n");

	for (size_t i = 0; i < 5; i++)
	{
		ret = gpio_pin_toggle_dt(&led_blue);		
		if (ret < 0) {
			return 0;
		}
		ret = gpio_pin_toggle_dt(&led_red);		
		if (ret < 0) {
			return 0;
		}

		k_msleep(SLEEP_TIME_MS);
	}
	

	//printf("Entering system off\n");

	// rc = pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);
	// if (rc < 0) {
	// 	printf("Could not suspend console (%d)\n", rc);
	// 	return 0;
	// }

	

	if (IS_ENABLED(CONFIG_APP_RETENTION)) {
		/* Update the retained state */
		retained.off_count += 1;
		retained_update();
	}

	ret = gpio_pin_set_dt(&led_blue, 0);
	ret = gpio_pin_set_dt(&led_red, 0);

	sys_poweroff();

	return 0;
}
