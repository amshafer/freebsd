/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Beckhoff Automation GmbH & Co. KG
 */

#ifndef _DEV_GPIO_INTEL_INTELGPIO_H_
#define _DEV_GPIO_INTEL_INTELGPIO_H_

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/queue.h>

#include <machine/intr_machdep.h>

#include <contrib/dev/acpica/include/acpi.h>

#define INTELGPIO_PADBAR_REG	      0x00c
#define INTELGPIO_PAD_SIZE	      16

#define INTELGPIO_PADCFG0_GPIOTXSTATE (1 << 0)
#define INTELGPIO_PADCFG0_GPIORXSTATE (1 << 1)
#define INTELGPIO_PADCFG0_GPIOTXDIS   (1 << 8)
#define INTELGPIO_PADCFG0_GPIORXDIS   (1 << 9)
#define INTELGPIO_PADCFG0_PMODE_MASK  (0x7 << 10)
#define INTELGPIO_PADCFG0_PMODE_GPIO  0

/* Pad interrupt routing/trigger configuration (PADCFG0). */
#define INTELGPIO_PADCFG0_GPIROUTNMI	  (1 << 17)
#define INTELGPIO_PADCFG0_GPIROUTSMI	  (1 << 18)
#define INTELGPIO_PADCFG0_GPIROUTSCI	  (1 << 19)
#define INTELGPIO_PADCFG0_GPIROUTIOXAPIC  (1 << 20)
#define INTELGPIO_PADCFG0_RXINV		  (1 << 23)
#define INTELGPIO_PADCFG0_RXEVCFG_MASK	  (0x3 << 25)
#define INTELGPIO_PADCFG0_RXEVCFG_LEVEL	  (0x0 << 25)
#define INTELGPIO_PADCFG0_RXEVCFG_EDGE	  (0x1 << 25)
#define INTELGPIO_PADCFG0_RXEVCFG_DISABLED (0x2 << 25)
#define INTELGPIO_PADCFG0_RXEVCFG_EDGE_BOTH (0x3 << 25)

#define INTELGPIO_GPIO_NOMAP	      (-1)
#define INTELGPIO_MAX_COMMUNITIES     8

struct intelgpio_padgroup {
	int first_pad;
	int npads;
	int gpio_base;
	const char *name;
};

struct intelgpio_community {
	int ngroups;
	const struct intelgpio_padgroup *groups;
};

struct intelgpio_platform {
	const struct intelgpio_community *communities;
	int ncommunities;
	char **hids;
	const char *desc;
	/*
	 * Per-community register block offsets for interrupt support.
	 * Each is the base of an array of one 32-bit register per pad
	 * group (the group's GPI register index is its position in the
	 * community's groups array).  A gpi_is of 0 means the platform
	 * tables do not describe interrupt registers and GPIO_MAP_INTR
	 * is unsupported.
	 */
	bus_size_t gpi_is;	/* GPI interrupt status (write-1-clear) */
	bus_size_t gpi_ie;	/* GPI interrupt enable */
	bus_size_t hostown;	/* HOSTSW_OWN: 1 = GPIO driver owns pad */
};

struct intelgpio_isrc;

struct intelgpio_softc {
	device_t sc_dev;
	device_t sc_busdev;
	struct mtx sc_mtx;
	const struct intelgpio_platform *sc_plat;

	struct resource *sc_mem_res[INTELGPIO_MAX_COMMUNITIES];
	uint32_t sc_padbar[INTELGPIO_MAX_COMMUNITIES];

	/* Interrupt cascade state (unused when sc_plat->gpi_is == 0). */
	struct pic sc_pic;
	int sc_irq_rid;
	struct resource *sc_irq_res;
	void *sc_irq_ih;
	SLIST_HEAD(, intelgpio_isrc) sc_isrcs;	/* sc_mtx */
};

DECLARE_CLASS(intelgpio_driver);

int intelgpio_probe(device_t dev, const struct intelgpio_platform *plat);
int intelgpio_attach(device_t dev, const struct intelgpio_platform *plat);

#endif /* _DEV_GPIO_INTEL_INTELGPIO_H_ */
