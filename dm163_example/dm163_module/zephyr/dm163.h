#ifndef DM163_H
#define DM163_H
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

void dm163_turn_off_row(const struct device *dev, const struct gpio_dt_spec *rows,int row);

#endif