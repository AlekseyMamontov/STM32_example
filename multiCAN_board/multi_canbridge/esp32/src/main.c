/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Empty starting point: blinks the status LED (alias led0).
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include <bus_proto.h>

#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

	while (1) {
		gpio_pin_toggle_dt(&led);
		k_msleep(500);
	}

	return 0;
}
