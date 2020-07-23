/*
 * GPIO driver for the Xilinx AXI GPIO v2.0 LogiCORE IP Core
 * Copyright (c) 2020 Weidmueller Interface GmbH & Co. KG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * IP core documentation used:
 * Xilinx AXI GPIO v2.0 LogiCORE IP Product Guide PG144 dated October 5, 2016.
 * NOTICE: this driver only supports the AXI GPIO IP core in single channel
 * operation mode. While pin access in index-based and could therefore handle
 * the dual-channel mode range [0..63], the bit masks of the callback API
 * are limited to 32 bits.
 */

#include <kernel.h>
#include <device.h>
#include <errno.h>

#include <drivers/gpio.h>
#include "gpio_utils.h"

#define DT_DRV_COMPAT xlnx_axi_gpio

#define LOG_MODULE_NAME gpio_xlnx_axi
#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define GPIO_XLNX_AXI_PINS_PER_CHANNEL		32

#define GPIO_XLNX_AXI_GPIO_DATA_REG_OFFSET	0x0000
#define GPIO_XLNX_AXI_GPIO_TRI_REG_OFFSET	0x0004
#define GPIO_XLNX_AXI_GPIO_GIER_REG_OFFSET	0x011C
#define GPIO_XLNX_AXI_GPIO_IP_IER_REG_OFFSET	0x0128
#define GPIO_XLNX_AXI_GPIO_IP_ISR_REG_OFFSET	0x0120

#define GPIO_XLNX_AXI_GLOBAL_INT_ENABLE		(1 << 31)
#define GPIO_XLNX_AXI_CH1_INT_ENABLE		(1 <<  0)
#define GPIO_XLNX_AXI_CH1_INT_DISABLE		(0 <<  0)
#define GPIO_XLNX_AXI_CH1_INT_PENDING		(1 <<  0)

#define LOG_LEVEL CONFIG_GPIO_LOG_LEVEL

/* Device configuration / run-time data resolver macros */
#define DEV_CFG(dev) \
	((struct gpio_xlnx_axi_dev_cfg*)(dev)->config_info)
#define DEV_DATA(dev) \
	((struct gpio_xlnx_axi_dev_data*)(dev)->driver_data)

/* Static driver instance configuration data */
typedef void (*gpio_xlnx_axi_config_irq_t)(struct device*);
struct gpio_xlnx_axi_dev_cfg {
	uint32_t			base_addr;
	uint32_t			data_reg_offset;
	uint32_t			tri_reg_offset;
	uint32_t			gier_reg_offset;
	uint32_t			ip_ier_reg_offset;
	uint32_t			ip_isr_reg_offset;

	uint8_t				supp_interrupt;
	gpio_xlnx_axi_config_irq_t	config_func;
};

/* Driver instance run-time data */
struct gpio_xlnx_axi_dev_data {
	struct gpio_driver_data		base_data;
	sys_slist_t 			callbacks;

	uint32_t			last_data;
	uint32_t			pin_dir;
	uint32_t			int_mask;
	uint64_t			int_trig;

	uint8_t				use_interrupt;
};

/* ISR */
static void gpio_xlnx_axi_isr (void *arg)
{
	struct device			*dev         = (struct device*)arg;
	struct gpio_xlnx_axi_dev_cfg	*dev_conf    = DEV_CFG(dev);
	struct gpio_xlnx_axi_dev_data	*dev_data    = DEV_DATA(dev);
	uint32_t			isr_reg_val  = 0;
	uint32_t			curr_in_data = 0;
	uint32_t			last_in_data = 0;
	uint32_t			pin_mask     = 0;

	isr_reg_val = sys_read32(dev_conf->base_addr
		+ dev_conf->ip_isr_reg_offset);

	if (isr_reg_val & GPIO_XLNX_AXI_CH1_INT_PENDING) {
		/* 
		 * A data change interrupt is pending for channel 1.
		 * -> Read the current contents of the data register.
		 *    (For all pins configured as output: RAZ)
		 * -> Apply the invert mask to the acquired data.
		 * -> XOR the resulting data word with the reference data
		 *    word obtained during either the last read/set/
		 *    clear/toggle call or the last execution of the ISR,
		 *    thus creating a bitmask of all pins whose level has
		 *    changed. If the bitmask is non-zero, hand it over
		 *    to gpio_fire_callbacks.
		 */
		curr_in_data = sys_read32(dev_conf->base_addr
			+ dev_conf->data_reg_offset);
		last_in_data = dev_data->last_data & (~dev_data->pin_dir);
		pin_mask     = curr_in_data ^ last_in_data;

		gpio_fire_callbacks(&dev_data->callbacks, dev, pin_mask);

		/* Clear the interrupt pending bit */
		sys_write32(GPIO_XLNX_AXI_CH1_INT_PENDING, 
			dev_conf->base_addr + dev_conf->ip_isr_reg_offset);

		/* Store the current data word contents as reference for
		 * future level change detection. Retain the output data,
		 * replace the input data with the current data. */
		dev_data->last_data = (dev_data->last_data 
			& dev_data->pin_dir) | curr_in_data;
	}
}

static int gpio_xlnx_axi_pin_config (
	struct device *dev, 
	gpio_pin_t    pin, 
	gpio_flags_t  flags)
{
	struct gpio_xlnx_axi_dev_cfg	*dev_conf = DEV_CFG(dev);
	struct gpio_xlnx_axi_dev_data	*dev_data = DEV_DATA(dev);

	if (pin >= GPIO_XLNX_AXI_PINS_PER_CHANNEL) {
		/* Pin index exceeds the valid range. */
		return -EINVAL;
	}

	if (((flags & GPIO_INPUT) == 0) && ((flags & GPIO_OUTPUT) == 0)) {
		/* No direction specified for the respective pin. */
		return -ENOTSUP;
	}

	if (((flags & GPIO_INPUT) != 0) && ((flags & GPIO_OUTPUT) != 0)) {
		/* Bi-directional GPIO pins are not supported by the AXI 
		 * GPIO IP Core. */
		return -ENOTSUP;
	}

	if ((flags & GPIO_PULL_UP) || (flags & GPIO_PULL_DOWN)) {
		/* No such option for the AXI GPIO IP Core. */
		return -ENOTSUP;
	}

	/* Store the respective pin's configuration in the current device's
	 * run-time data section, update AXI GPIO IP Core register contents
	 * accordingly. */
	if (flags & GPIO_OUTPUT) {
		dev_data->pin_dir |= BIT(pin);
	} else  if (flags & GPIO_INPUT) {
		dev_data->pin_dir &= ~BIT(pin);
	}

	/* Update the pin direction register */
	sys_write32(~(dev_data->pin_dir), dev_conf->base_addr
		+ dev_conf->tri_reg_offset);

	/* Read the current data register contents as reference for pin level
	 * change detection in the ISR.
	 * -> retain the current output data, replace the input data. */
	dev_data->last_data &= dev_data->pin_dir;
	dev_data->last_data |= sys_read32(dev_conf->base_addr + dev_conf->data_reg_offset);

	/* If an initial value was specified for the pin, adjust the data register
	 * accordingly and store the initial value locally. */
	if ((flags & GPIO_OUTPUT) 
		&& (flags & (GPIO_OUTPUT_INIT_LOW | GPIO_OUTPUT_INIT_HIGH))) {

		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			dev_data->last_data |= BIT(pin);
		} else if (flags & GPIO_OUTPUT_INIT_LOW) {
			dev_data->last_data &= ~BIT(pin);
		}

		sys_write32(dev_data->last_data, dev_conf->base_addr + dev_conf->data_reg_offset);
	}

	return 0;
}

static int gpio_xlnx_axi_port_get_raw (
	struct device     *dev, 
	gpio_port_value_t *value)
{
	struct gpio_xlnx_axi_dev_cfg	*dev_conf = DEV_CFG(dev);
	struct gpio_xlnx_axi_dev_data	*dev_data = DEV_DATA(dev);
	uint32_t			reg_val   = 0;

	/* Read the entire data word. Pins configured as output are RAZ. */
	reg_val = sys_read32(dev_conf->base_addr + dev_conf->data_reg_offset);

	/* Store the register contents just read as reference for pin 
	 * level change detection -> retain the output data, replace the
	 * input data. */
	dev_data->last_data &= dev_data->pin_dir;
	dev_data->last_data |= reg_val;

	*value = (gpio_port_value_t)reg_val;
	return 0;
}

static int gpio_xlnx_axi_port_set_masked_raw (
	struct device     *dev, 
	gpio_port_pins_t  mask,
	gpio_port_value_t value)
{
	struct gpio_xlnx_axi_dev_cfg	*dev_conf = DEV_CFG(dev);
	struct gpio_xlnx_axi_dev_data	*dev_data = DEV_DATA(dev);
	uint32_t			reg_val   = 0;
	uint32_t			reg_msk   = 0;

	/* Write the entire data word with an additional mask
	 * provided by the caller. Although writing to pins 
	 * configured as input has no effect, mask out any bits
	 * that belong to pins configured as input. Apply the 
	 * invert mask to the data to be written. Apply the mask
	 * provided by the caller afterwards. */
	reg_val = (uint32_t)value & dev_data->pin_dir;
	reg_msk = (uint32_t)mask  & dev_data->pin_dir;
	reg_val &= reg_msk;

	/* Store the register contents about to be written as 
	 * reference for pin level change detection -> retain the 
	 * input data, replace the output data. */
	dev_data->last_data &= ~dev_data->pin_dir;
	dev_data->last_data |= reg_val;
	
	sys_write32(dev_data->last_data, dev_conf->base_addr
		+ dev_conf->data_reg_offset);
	return 0;
}

static int gpio_xlnx_axi_port_set_bits_raw (
	struct device    *dev, 
	gpio_port_pins_t pins)
{
	struct gpio_xlnx_axi_dev_cfg	*dev_conf = DEV_CFG(dev);
	struct gpio_xlnx_axi_dev_data	*dev_data = DEV_DATA(dev);
	uint32_t			reg_val   = 0;

	/* Set individual output pins based on the bitmask
	 * provided by the caller, then write the entire data 
	 * word. Although writing to pins configured as input 
	 * has no effect, mask out any bits that belong to pins
	 * configured as input. Apply the invert mask to the 
	 * data to be written. */
	reg_val  = (uint32_t)pins & dev_data->pin_dir;

	/* Store the register contents about to be written as 
	 * reference for pin level change detection -> retain the 
	 * input data, replace the output data. */
	dev_data->last_data &= ~dev_data->pin_dir;
	dev_data->last_data |= reg_val;

	sys_write32(dev_data->last_data, dev_conf->base_addr
		+ dev_conf->data_reg_offset);
	return 0;
}

static int gpio_xlnx_axi_port_clear_bits_raw (
	struct device    *dev, 
	gpio_port_pins_t pins)
{
	struct gpio_xlnx_axi_dev_cfg	*dev_conf = DEV_CFG(dev);
	struct gpio_xlnx_axi_dev_data	*dev_data = DEV_DATA(dev);
	uint32_t			reg_val   = 0;

	/* Clear individual output pins based on the bitmask
	 * provided by the caller, then write the entire data 
	 * word. Although writing to pins configured as input 
	 * has no effect, mask out any bits that belong to pins
	 * configured as input. */
	reg_val = ~((uint32_t)pins & dev_data->pin_dir);

	/* Store the register contents about to be written as 
	 * reference for pin level change detection -> apply
	 * the calculated bitmask masking out the output pins
	 * specified by the caller. */
	dev_data->last_data &= reg_val;

	sys_write32(dev_data->last_data, dev_conf->base_addr
		+ dev_conf->data_reg_offset);
	return 0;
}

static int gpio_xlnx_axi_port_toggle_bits (
	struct device    *dev, 
	gpio_port_pins_t pins)
{
	struct gpio_xlnx_axi_dev_cfg	*dev_conf = DEV_CFG(dev);
	struct gpio_xlnx_axi_dev_data	*dev_data = DEV_DATA(dev);
	uint32_t			reg_val   = 0;

	/* Toggle individual output pins based on the bitmask
	 * provided by the caller, then write the entire data 
	 * word. Although writing to pins configured as input 
	 * has no effect, mask out any bits that belong to pins
	 * configured as input. */
	reg_val = (uint32_t)pins & dev_data->pin_dir;

	/* Store the register contents about to be written as 
	 * reference for pin level change detection -> apply
	 * the calculated bitmask using XOR in order to toggle
	 * the pins specified by the caller. */
	dev_data->last_data ^= reg_val;

	sys_write32(dev_data->last_data, dev_conf->base_addr
		+ dev_conf->data_reg_offset);
	return 0;
}

static int gpio_xlnx_axi_pin_interrupt_configure (
	struct device      *dev, 
	gpio_pin_t         pin,
	enum gpio_int_mode mode, 
	enum gpio_int_trig trig)
{
	struct gpio_xlnx_axi_dev_cfg	*dev_conf = DEV_CFG(dev);
	struct gpio_xlnx_axi_dev_data	*dev_data = DEV_DATA(dev);
	uint32_t			mask_pre  = dev_data->int_mask;

	if (pin >= GPIO_XLNX_AXI_PINS_PER_CHANNEL) {
		/* Pin index exceeds the valid range. */
		return -EINVAL;
	}

	if (dev_conf->supp_interrupt == 0) {
		/* Interrupt not supported by the current instance. */
		return -ENOTSUP;
	}

	if (mode == GPIO_INT_MODE_DISABLED) {
		dev_data->int_mask &= ~BIT(pin);

		if (dev_data->int_mask == 0) {
			dev_data->use_interrupt = 0;

			/* Not a single pin is configured to trigger an
			 * interrupt -> clear the interrupt enable bit
			 * in the IP core. */
			sys_write32(GPIO_XLNX_AXI_CH1_INT_DISABLE,
				(dev_conf->base_addr
				+ dev_conf->ip_ier_reg_offset));
		}
	} else if (mode == GPIO_INT_MODE_EDGE) {
		dev_data->int_mask	|= BIT(pin);
		dev_data->use_interrupt	 = 1;

		if (mask_pre == 0) {
			/* The first pin is configured to trigger an
			 * interrupt -> set the interrupt enable bit
			 * in the IP core. */
			sys_write32(GPIO_XLNX_AXI_CH1_INT_ENABLE,
				(dev_conf->base_addr
				+ dev_conf->ip_ier_reg_offset));
		}
	} else {
		/* The interrupt of the AXI GPIO IP core is triggered whenever
		 * a level change of *any* pin is detected. It is only during
		 * the handling of such an interrupt that the current value
		 * of the input pins can be compared against the last known
		 * value. Therefore, the AXI GPIO IP core implements edge-
		 * triggered interrupt behaviour, level triggered interrupts
		 * are unsupported. */
		return -ENOTSUP;
	}

	return 0;
}

static int gpio_xlnx_axi_manage_callback (
	struct device        *dev,
	struct gpio_callback *callback,
	bool                 set) 
{
	struct gpio_xlnx_axi_dev_cfg	*dev_conf = DEV_CFG(dev);
	struct gpio_xlnx_axi_dev_data	*dev_data = DEV_DATA(dev);

	if (dev_conf->supp_interrupt == 0) {
		/* Interrupt not supported by the current instance. */
		return -ENOTSUP;
	}

	return gpio_manage_callback(&dev_data->callbacks, callback, set);
}

static uint32_t gpio_xlnx_axi_get_pending_int (struct device *dev)
{
	struct gpio_xlnx_axi_dev_cfg 	*dev_conf = DEV_CFG(dev);
	struct gpio_xlnx_axi_dev_data	*dev_data = DEV_DATA(dev);
	uint32_t			reg_val   = 0;

	if (dev_conf->supp_interrupt == 0
		|| dev_data->use_interrupt == 0)
	{
		/* No interrupt specified in device tree for this instance or
		 * not a single pin managed by this instance is configured as
		 * an interrupt source -> no pending interrupt. */
		return 0;
	}

	reg_val = sys_read32(dev_conf->base_addr
		+ dev_conf->ip_isr_reg_offset);
	return (reg_val & GPIO_XLNX_AXI_CH1_INT_PENDING) ? 1 : 0;
}

/* GPIO API function pointers for this driver */
static const struct gpio_driver_api gpio_xlnx_axi_driver_api = {
	.pin_configure           = gpio_xlnx_axi_pin_config,
	.port_get_raw            = gpio_xlnx_axi_port_get_raw,
	.port_set_masked_raw     = gpio_xlnx_axi_port_set_masked_raw,
	.port_set_bits_raw       = gpio_xlnx_axi_port_set_bits_raw,
	.port_clear_bits_raw     = gpio_xlnx_axi_port_clear_bits_raw,
	.port_toggle_bits        = gpio_xlnx_axi_port_toggle_bits,
	.pin_interrupt_configure = gpio_xlnx_axi_pin_interrupt_configure,
	.manage_callback         = gpio_xlnx_axi_manage_callback,
	.get_pending_int         = gpio_xlnx_axi_get_pending_int
};

/* Per-instance run-time data initialization function */
static int gpio_xlnx_axi_init(struct device *dev) 
{
	struct gpio_xlnx_axi_dev_cfg	*dev_conf = DEV_CFG(dev);
	struct gpio_xlnx_axi_dev_data	*dev_data = DEV_DATA(dev);

	dev_data->int_mask	= 0;
	dev_data->int_trig	= 0;
	dev_data->last_data	= 0;
	dev_data->pin_dir	= 0;
	dev_data->use_interrupt	= 0;

	if (dev_conf->supp_interrupt == 1 && dev_conf->config_func != NULL) {
		dev_conf->config_func(dev);
	}

	printk("Init AXI GPIO @0x%08X: %s\n", dev_conf->base_addr, dev_conf->supp_interrupt != 0 ? "IRQ" : "polling");
	return 0;
}

#define GPIO_XLNX_AXI_NODEID(inst) DT_INST(inst, xlnx_axi_gpio)

#define GPIO_XLNX_AXI_DEV_DATA(inst) \
static struct gpio_xlnx_axi_dev_data gpio_xlnx_axi_dev_data_##inst

#define GPIO_XLNX_AXI_DEV_CFG(inst) \
static void gpio_xlnx_axi_config_interrupt_##inst(struct device* dev); \
static struct gpio_xlnx_axi_dev_cfg gpio_xlnx_axi_dev_cfg_##inst = { \
	.base_addr         = DT_INST_REG_ADDR(inst), \
	.supp_interrupt    = DT_PROP(GPIO_XLNX_AXI_NODEID(inst), polling) ? 0 : 1, \
	.data_reg_offset   = GPIO_XLNX_AXI_GPIO_DATA_REG_OFFSET, \
	.tri_reg_offset    = GPIO_XLNX_AXI_GPIO_TRI_REG_OFFSET, \
	.gier_reg_offset   = GPIO_XLNX_AXI_GPIO_GIER_REG_OFFSET, \
	.ip_ier_reg_offset = GPIO_XLNX_AXI_GPIO_IP_IER_REG_OFFSET, \
	.ip_isr_reg_offset = GPIO_XLNX_AXI_GPIO_IP_ISR_REG_OFFSET, \
	.config_func       = gpio_xlnx_axi_config_interrupt_##inst \
}

#define GPIO_XLNX_AXI_DEV_INIT(inst) \
DEVICE_AND_API_INIT(gpio_xlnx_axi_##inst, DT_INST_LABEL(inst), \
	gpio_xlnx_axi_init, \
	&gpio_xlnx_axi_dev_data_##inst, \
	&gpio_xlnx_axi_dev_cfg_##inst, \
	PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
	&gpio_xlnx_axi_driver_api)

#define GPIO_XLNX_AXI_IRQ_INIT(inst) \
static void gpio_xlnx_axi_config_interrupt_##inst(struct device* dev) \
{ \
	struct gpio_xlnx_axi_dev_cfg *dev_conf = DEV_CFG(dev); \
	if (dev_conf->supp_interrupt == 1) { \
		IRQ_CONNECT(DT_INST_IRQN(inst), \
			DT_INST_IRQ(inst, priority), \
			gpio_xlnx_axi_isr, \
			DEVICE_GET(gpio_xlnx_axi_##inst), \
			0); \
		irq_enable(DT_INST_IRQN(inst)); \
		sys_write32(GPIO_XLNX_AXI_GLOBAL_INT_ENABLE, \
			dev_conf->base_addr + dev_conf->gier_reg_offset); \
	} \
}

#define GPIO_XLNX_AXI_DEV_INSTANTIATE(inst) \
	GPIO_XLNX_AXI_DEV_DATA(inst); \
	GPIO_XLNX_AXI_DEV_CFG(inst); \
	GPIO_XLNX_AXI_DEV_INIT(inst); \
	GPIO_XLNX_AXI_IRQ_INIT(inst);

DT_INST_FOREACH_STATUS_OKAY(GPIO_XLNX_AXI_DEV_INSTANTIATE)

/* EOF */