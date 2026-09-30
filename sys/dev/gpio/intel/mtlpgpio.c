/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Austin Shafer
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/module.h>

#include <contrib/dev/acpica/include/acpi.h>

#include <dev/acpica/acpivar.h>
#include <dev/gpio/gpiobusvar.h>

#include "intelgpio.h"

#include "gpio_if.h"
#include "opt_acpi.h"

/*
 * Pad layout per Meteor Lake-P PCH EDS; cross-checked against Linux
 * drivers/pinctrl/intel/pinctrl-meteorlake.c.  gpio_base values follow
 * Intel's 32-aligned per-group GPIO numbering, which is the number space
 * ACPI GpioInt/GpioIo descriptors reference.
 */

/* Community 0: CPU, GPP_V, GPP_C */
static const struct intelgpio_padgroup mtlp_com0_groups[] = {
	{ .first_pad = 0, .npads = 5, .gpio_base = 0, .name = "CPU" },
	{ .first_pad = 5, .npads = 24, .gpio_base = 32, .name = "GPP_V" },
	{ .first_pad = 29, .npads = 24, .gpio_base = 64, .name = "GPP_C" },
};

/* Community 1: GPP_A, GPP_E */
static const struct intelgpio_padgroup mtlp_com1_groups[] = {
	{ .first_pad = 53, .npads = 25, .gpio_base = 96, .name = "GPP_A" },
	{ .first_pad = 78, .npads = 25, .gpio_base = 128, .name = "GPP_E" },
};

/* Community 2: GPP_H, GPP_F, SPI0, vGPIO_3 */
static const struct intelgpio_padgroup mtlp_com2_groups[] = {
	{ .first_pad = 103, .npads = 26, .gpio_base = 160, .name = "GPP_H" },
	{ .first_pad = 129, .npads = 26, .gpio_base = 192, .name = "GPP_F" },
	{ .first_pad = 155, .npads = 15, .gpio_base = 224, .name = "SPI0" },
	{ .first_pad = 170, .npads = 14, .gpio_base = 256, .name = "vGPIO_3" },
};

/* Community 3: GPP_S, JTAG */
static const struct intelgpio_padgroup mtlp_com3_groups[] = {
	{ .first_pad = 184, .npads = 8, .gpio_base = 288, .name = "GPP_S" },
	{ .first_pad = 192, .npads = 12, .gpio_base = 320, .name = "JTAG" },
};

/* Community 4: GPP_B, GPP_D, vGPIO_0, vGPIO_1 */
static const struct intelgpio_padgroup mtlp_com4_groups[] = {
	{ .first_pad = 204, .npads = 25, .gpio_base = 352, .name = "GPP_B" },
	{ .first_pad = 229, .npads = 25, .gpio_base = 384, .name = "GPP_D" },
	{ .first_pad = 254, .npads = 32, .gpio_base = 416, .name = "vGPIO_0" },
	{ .first_pad = 286, .npads = 3, .gpio_base = 448, .name = "vGPIO_1" },
};

static const struct intelgpio_community mtlp_communities[] = {
	{ .ngroups = nitems(mtlp_com0_groups), .groups = mtlp_com0_groups },
	{ .ngroups = nitems(mtlp_com1_groups), .groups = mtlp_com1_groups },
	{ .ngroups = nitems(mtlp_com2_groups), .groups = mtlp_com2_groups },
	{ .ngroups = nitems(mtlp_com3_groups), .groups = mtlp_com3_groups },
	{ .ngroups = nitems(mtlp_com4_groups), .groups = mtlp_com4_groups },
};

static char *mtlp_hids[] = { "INTC1083", "INTC105E", NULL };

static const struct intelgpio_platform mtlp_platform = {
	.communities = mtlp_communities,
	.ncommunities = nitems(mtlp_communities),
	.hids = mtlp_hids,
	.desc = "Intel Meteor Lake-P GPIO",
	.gpi_is = 0x200,
	.gpi_ie = 0x210,
	.hostown = 0x140,
};

static int
mtlp_gpio_probe(device_t dev)
{
	return (intelgpio_probe(dev, &mtlp_platform));
}

static int
mtlp_gpio_attach(device_t dev)
{
	return (intelgpio_attach(dev, &mtlp_platform));
}

static device_method_t mtlp_methods[] = {
	DEVMETHOD(device_probe, mtlp_gpio_probe),
	DEVMETHOD(device_attach, mtlp_gpio_attach),

	DEVMETHOD_END
};

DEFINE_CLASS_1(gpio, mtlp_driver, mtlp_methods,
    sizeof(struct intelgpio_softc), intelgpio_driver);

DRIVER_MODULE(mtlpgpio, acpi, mtlp_driver, NULL, NULL);
MODULE_DEPEND(mtlpgpio, acpi, 1, 1, 1);
MODULE_DEPEND(mtlpgpio, gpiobus, 1, 1, 1);
MODULE_VERSION(mtlpgpio, 1);
ACPI_PNP_INFO(mtlp_hids);
