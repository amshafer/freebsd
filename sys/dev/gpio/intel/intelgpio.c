/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Beckhoff Automation GmbH & Co. KG
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/interrupt.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/proc.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <contrib/dev/acpica/include/accommon.h>

#include <dev/acpica/acpivar.h>
#include <dev/gpio/gpiobusvar.h>

#include "intelgpio.h"

#include "gpio_if.h"
#include "opt_acpi.h"

#define INTELGPIO_LOCK(_sc)   mtx_lock_spin(&(_sc)->sc_mtx)
#define INTELGPIO_UNLOCK(_sc) mtx_unlock_spin(&(_sc)->sc_mtx)
#define INTELGPIO_LOCK_INIT(_sc)                                     \
	mtx_init(&(_sc)->sc_mtx, device_get_nameunit((_sc)->sc_dev), \
	    "intelgpio", MTX_SPIN)
#define INTELGPIO_LOCK_DESTROY(_sc) mtx_destroy(&(_sc)->sc_mtx)

/*
 * A GPIO pin mapped as an x86 interrupt source.  Entries are created by
 * GPIO_MAP_INTR and stay registered for the life of the driver: the x86
 * interrupt code has no way to unregister a source, so GPIO_UNMAP_INTR
 * only disables the pad and keeps the entry for re-mapping of the same
 * pin.  Consequently detach is refused once any pin has been mapped.
 */
struct intelgpio_isrc {
	struct intsrc gi_isrc;		/* must be first */
	SLIST_ENTRY(intelgpio_isrc) gi_next;
	struct intelgpio_softc *gi_sc;
	uint32_t gi_pin;		/* GPIO (ACPI) pin number */
	u_int gi_irq;
	uint32_t gi_mode;		/* GPIO_INTR_* trigger mode */
	int gi_com;			/* community index */
	int gi_reg;			/* GPI_IS/GPI_IE register index */
	uint32_t gi_mask;		/* bit within that register */
	bus_size_t gi_pad_off;		/* PADCFG0 offset */
	bool gi_level;
	bool gi_mapped;
};

static MALLOC_DEFINE(M_INTELGPIO, "intelgpio", "intelgpio interrupt sources");

static int
intelgpio_gpio_to_pad(struct intelgpio_softc *sc, uint32_t pin,
    struct resource **res, bus_size_t *pad_off,
    const struct intelgpio_padgroup **pgp)
{
	const struct intelgpio_platform *plat = sc->sc_plat;
	const struct intelgpio_community *com;
	const struct intelgpio_padgroup *pg;
	int i, j, offset, local_pad;

	for (i = 0; i < plat->ncommunities; i++) {
		com = &plat->communities[i];
		for (j = 0; j < com->ngroups; j++) {
			pg = &com->groups[j];
			if (pg->gpio_base == INTELGPIO_GPIO_NOMAP)
				continue;
			if (pin >= pg->gpio_base &&
			    pin < pg->gpio_base + pg->npads) {
				if (res != NULL)
					*res = sc->sc_mem_res[i];
				if (pad_off != NULL) {
					offset = pin - pg->gpio_base;
					local_pad = (pg->first_pad + offset) -
					    com->groups[0].first_pad;
					*pad_off = sc->sc_padbar[i] +
					    (local_pad * INTELGPIO_PAD_SIZE);
				}
				if (pgp != NULL)
					*pgp = pg;
				return (0);
			}
		}
	}
	return (-1);
}

static device_t
intelgpio_get_bus(device_t dev)
{
	struct intelgpio_softc *sc;

	sc = device_get_softc(dev);
	return (sc->sc_busdev);
}

static int
intelgpio_pin_max(device_t dev, int *maxpin)
{
	struct intelgpio_softc *sc;
	const struct intelgpio_platform *plat;
	const struct intelgpio_community *com;
	const struct intelgpio_padgroup *pg;
	int i, j;

	if (maxpin == NULL)
		return (EINVAL);

	sc = device_get_softc(dev);
	plat = sc->sc_plat;

	/* Find the last group with a valid gpio_base, skipping NOMAP */
	*maxpin = 0;
	for (i = 0; i < plat->ncommunities; i++) {
		com = &plat->communities[i];
		for (j = 0; j < com->ngroups; j++) {
			pg = &com->groups[j];
			if (pg->gpio_base == INTELGPIO_GPIO_NOMAP)
				continue;
			*maxpin = pg->gpio_base + (pg->npads - 1);
		}
	}
	return (0);
}

static int
intelgpio_pin_getname(device_t dev, uint32_t pin, char *name)
{
	struct intelgpio_softc *sc;
	const struct intelgpio_padgroup *pg;
	int offset;

	sc = device_get_softc(dev);
	if (intelgpio_gpio_to_pad(sc, pin, NULL, NULL, &pg) != 0)
		return (EINVAL);

	offset = pin - pg->gpio_base;
	snprintf(name, GPIOMAXNAME, "%s%d", pg->name, offset);
	return (0);
}

static int
intelgpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{
	struct intelgpio_softc *sc;

	sc = device_get_softc(dev);

	/*
	 * The getcaps function is expected to return the capabilities
	 * of the pin. Validate that the pin number maps to an existing
	 * pad first.
	 */
	if (intelgpio_gpio_to_pad(sc, pin, NULL, NULL, NULL) != 0)
		return (EINVAL);

	*caps = GPIO_PIN_INPUT | GPIO_PIN_OUTPUT;
	return (0);
}

static int
intelgpio_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct intelgpio_softc *sc;
	struct resource *res;
	bus_size_t pad_off;
	uint32_t val;

	sc = device_get_softc(dev);
	if (intelgpio_gpio_to_pad(sc, pin, &res, &pad_off, NULL) != 0)
		return (EINVAL);
	if (res == NULL)
		return (EINVAL);

	*flags = 0;

	INTELGPIO_LOCK(sc);
	val = bus_read_4(res, pad_off);
	INTELGPIO_UNLOCK(sc);

	/*
	 * If the pad is not in GPIO mode (i.e. configured for a native
	 * function like PCIe, UART, I2C, etc.), report no flags. The pin
	 * cannot be used as GPIO without first switching it via setflags.
	 */
	if ((val & INTELGPIO_PADCFG0_PMODE_MASK) !=
	    INTELGPIO_PADCFG0_PMODE_GPIO)
		return (0);

	/*
	 * Report OUTPUT only if TX is explicitly enabled. Default to INPUT
	 * otherwise: a non-driving pin is considered an input from the user's
	 * perspective.
	 */
	if (!(val & INTELGPIO_PADCFG0_GPIOTXDIS))
		*flags = GPIO_PIN_OUTPUT;
	else
		*flags = GPIO_PIN_INPUT;

	return (0);
}

static int
intelgpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct intelgpio_softc *sc;
	struct resource *res;
	bus_size_t pad_off;
	uint32_t val;

	sc = device_get_softc(dev);
	if (intelgpio_gpio_to_pad(sc, pin, &res, &pad_off, NULL) != 0)
		return (EINVAL);
	if (res == NULL)
		return (EINVAL);

	/*
	 * We currently only support input and output, not both simultaneously
	 */
	if (flags & ~(GPIO_PIN_INPUT | GPIO_PIN_OUTPUT))
		return (EINVAL);
	if ((flags & GPIO_PIN_INPUT) && (flags & GPIO_PIN_OUTPUT))
		return (EINVAL);

	INTELGPIO_LOCK(sc);
	val = bus_read_4(res, pad_off);

	/*
	 * Switch the pad to GPIO mode. Intel pads can operate in different
	 * modes (native function 1-15 or GPIO). Since setflags is only called
	 * when userspace or a consumer explicitly wants to use the pin as a
	 * GPIO, we force the pad into GPIO mode here so the direction bits
	 * (TXDIS/RXDIS) below take effect.
	 */
	val &= ~INTELGPIO_PADCFG0_PMODE_MASK;
	val |= INTELGPIO_PADCFG0_PMODE_GPIO;

	if (flags & GPIO_PIN_INPUT)
		val &= ~INTELGPIO_PADCFG0_GPIORXDIS;
	else
		val |= INTELGPIO_PADCFG0_GPIORXDIS;

	if (flags & GPIO_PIN_OUTPUT)
		val &= ~INTELGPIO_PADCFG0_GPIOTXDIS;
	else
		val |= INTELGPIO_PADCFG0_GPIOTXDIS;

	bus_write_4(res, pad_off, val);
	INTELGPIO_UNLOCK(sc);

	return (0);
}

static int
intelgpio_pin_get(device_t dev, uint32_t pin, unsigned int *value)
{
	struct intelgpio_softc *sc;
	struct resource *res;
	bus_size_t pad_off;
	uint32_t val;

	sc = device_get_softc(dev);
	if (intelgpio_gpio_to_pad(sc, pin, &res, &pad_off, NULL) != 0)
		return (EINVAL);
	if (res == NULL)
		return (EINVAL);

	INTELGPIO_LOCK(sc);
	val = bus_read_4(res, pad_off);
	INTELGPIO_UNLOCK(sc);

	*value = (val & INTELGPIO_PADCFG0_GPIORXSTATE) ?
	    GPIO_PIN_HIGH : GPIO_PIN_LOW;

	return (0);
}

static int
intelgpio_pin_set(device_t dev, uint32_t pin, unsigned int value)
{
	struct intelgpio_softc *sc;
	struct resource *res;
	bus_size_t pad_off;
	uint32_t val;

	sc = device_get_softc(dev);
	if (intelgpio_gpio_to_pad(sc, pin, &res, &pad_off, NULL) != 0)
		return (EINVAL);
	if (res == NULL)
		return (EINVAL);

	INTELGPIO_LOCK(sc);
	val = bus_read_4(res, pad_off);
	if (value == GPIO_PIN_LOW)
		val &= ~INTELGPIO_PADCFG0_GPIOTXSTATE;
	else
		val |= INTELGPIO_PADCFG0_GPIOTXSTATE;
	bus_write_4(res, pad_off, val);
	INTELGPIO_UNLOCK(sc);

	return (0);
}

static int
intelgpio_pin_toggle(device_t dev, uint32_t pin)
{
	struct intelgpio_softc *sc;
	struct resource *res;
	bus_size_t pad_off;
	uint32_t val;

	sc = device_get_softc(dev);
	if (intelgpio_gpio_to_pad(sc, pin, &res, &pad_off, NULL) != 0)
		return (EINVAL);
	if (res == NULL)
		return (EINVAL);

	INTELGPIO_LOCK(sc);
	val = bus_read_4(res, pad_off);
	val ^= INTELGPIO_PADCFG0_GPIOTXSTATE;
	bus_write_4(res, pad_off, val);
	INTELGPIO_UNLOCK(sc);

	return (0);
}

/*
 * Interrupt support: the controller demultiplexes its single IOAPIC IRQ
 * across pads via per-group GPI_IS/GPI_IE registers.  Each mapped pin is
 * registered as an x86 interrupt source (struct intsrc) with an IRQ
 * number from the machdep GPIO range, and the cascade filter dispatches
 * pending pins through intr_execute_handlers() the way Xen event
 * channels do.
 */

static int
intelgpio_gpio_to_gpi(struct intelgpio_softc *sc, uint32_t pin, int *comp,
    int *regp, uint32_t *maskp, bus_size_t *pad_offp)
{
	const struct intelgpio_platform *plat = sc->sc_plat;
	const struct intelgpio_community *com;
	const struct intelgpio_padgroup *pg;
	int i, j, offset, local_pad;

	for (i = 0; i < plat->ncommunities; i++) {
		com = &plat->communities[i];
		for (j = 0; j < com->ngroups; j++) {
			pg = &com->groups[j];
			if (pg->gpio_base == INTELGPIO_GPIO_NOMAP)
				continue;
			if (pin < pg->gpio_base ||
			    pin >= pg->gpio_base + pg->npads)
				continue;
			if (sc->sc_mem_res[i] == NULL)
				return (ENXIO);
			offset = pin - pg->gpio_base;
			*comp = i;
			*regp = j;
			*maskp = 1u << offset;
			local_pad = (pg->first_pad + offset) -
			    com->groups[0].first_pad;
			*pad_offp = sc->sc_padbar[i] +
			    (local_pad * INTELGPIO_PAD_SIZE);
			return (0);
		}
	}
	return (EINVAL);
}

static inline uint32_t
intelgpio_gpi_read(struct intelgpio_isrc *gi, bus_size_t base)
{
	return (bus_read_4(gi->gi_sc->sc_mem_res[gi->gi_com],
	    base + gi->gi_reg * 4));
}

static inline void
intelgpio_gpi_write(struct intelgpio_isrc *gi, bus_size_t base, uint32_t val)
{
	bus_write_4(gi->gi_sc->sc_mem_res[gi->gi_com],
	    base + gi->gi_reg * 4, val);
}

/* Program the pad as a GPIO input generating GPI interrupts in gi_mode. */
static void
intelgpio_program_pad(struct intelgpio_isrc *gi)
{
	struct intelgpio_softc *sc = gi->gi_sc;
	uint32_t val;

	INTELGPIO_LOCK(sc);
	val = bus_read_4(sc->sc_mem_res[gi->gi_com], gi->gi_pad_off);
	val &= ~(INTELGPIO_PADCFG0_PMODE_MASK |
	    INTELGPIO_PADCFG0_RXEVCFG_MASK | INTELGPIO_PADCFG0_RXINV |
	    INTELGPIO_PADCFG0_GPIROUTNMI | INTELGPIO_PADCFG0_GPIROUTSMI |
	    INTELGPIO_PADCFG0_GPIROUTSCI | INTELGPIO_PADCFG0_GPIROUTIOXAPIC |
	    INTELGPIO_PADCFG0_GPIORXDIS);
	val |= INTELGPIO_PADCFG0_PMODE_GPIO | INTELGPIO_PADCFG0_GPIOTXDIS;
	switch (gi->gi_mode & GPIO_INTR_MASK) {
	case GPIO_INTR_LEVEL_LOW:
		val |= INTELGPIO_PADCFG0_RXEVCFG_LEVEL |
		    INTELGPIO_PADCFG0_RXINV;
		break;
	case GPIO_INTR_LEVEL_HIGH:
		val |= INTELGPIO_PADCFG0_RXEVCFG_LEVEL;
		break;
	case GPIO_INTR_EDGE_RISING:
		val |= INTELGPIO_PADCFG0_RXEVCFG_EDGE;
		break;
	case GPIO_INTR_EDGE_FALLING:
		val |= INTELGPIO_PADCFG0_RXEVCFG_EDGE |
		    INTELGPIO_PADCFG0_RXINV;
		break;
	case GPIO_INTR_EDGE_BOTH:
		val |= INTELGPIO_PADCFG0_RXEVCFG_EDGE_BOTH;
		break;
	}
	bus_write_4(sc->sc_mem_res[gi->gi_com], gi->gi_pad_off, val);
	/* Discard any status latched while (re)configuring. */
	intelgpio_gpi_write(gi, sc->sc_plat->gpi_is, gi->gi_mask);
	INTELGPIO_UNLOCK(sc);
}

static void
intelgpio_pic_enable_source(struct intsrc *isrc)
{
	struct intelgpio_isrc *gi = (struct intelgpio_isrc *)isrc;
	struct intelgpio_softc *sc = gi->gi_sc;
	uint32_t ie;

	INTELGPIO_LOCK(sc);
	ie = intelgpio_gpi_read(gi, sc->sc_plat->gpi_ie);
	intelgpio_gpi_write(gi, sc->sc_plat->gpi_ie, ie | gi->gi_mask);
	INTELGPIO_UNLOCK(sc);
}

static void
intelgpio_pic_disable_source(struct intsrc *isrc, int eoi)
{
	struct intelgpio_isrc *gi = (struct intelgpio_isrc *)isrc;
	struct intelgpio_softc *sc = gi->gi_sc;
	uint32_t ie;

	INTELGPIO_LOCK(sc);
	ie = intelgpio_gpi_read(gi, sc->sc_plat->gpi_ie);
	intelgpio_gpi_write(gi, sc->sc_plat->gpi_ie, ie & ~gi->gi_mask);
	if (eoi == PIC_EOI)
		intelgpio_gpi_write(gi, sc->sc_plat->gpi_is, gi->gi_mask);
	INTELGPIO_UNLOCK(sc);
}

static void
intelgpio_pic_eoi_source(struct intsrc *isrc)
{
	struct intelgpio_isrc *gi = (struct intelgpio_isrc *)isrc;
	struct intelgpio_softc *sc = gi->gi_sc;

	INTELGPIO_LOCK(sc);
	intelgpio_gpi_write(gi, sc->sc_plat->gpi_is, gi->gi_mask);
	INTELGPIO_UNLOCK(sc);
}

static void
intelgpio_pic_enable_intr(struct intsrc *isrc)
{
	struct intelgpio_isrc *gi = (struct intelgpio_isrc *)isrc;

	intelgpio_program_pad(gi);
}

static void
intelgpio_pic_disable_intr(struct intsrc *isrc)
{
	struct intelgpio_isrc *gi = (struct intelgpio_isrc *)isrc;
	struct intelgpio_softc *sc = gi->gi_sc;
	uint32_t val;

	INTELGPIO_LOCK(sc);
	val = bus_read_4(sc->sc_mem_res[gi->gi_com], gi->gi_pad_off);
	val &= ~INTELGPIO_PADCFG0_RXEVCFG_MASK;
	val |= INTELGPIO_PADCFG0_RXEVCFG_DISABLED;
	bus_write_4(sc->sc_mem_res[gi->gi_com], gi->gi_pad_off, val);
	INTELGPIO_UNLOCK(sc);
}

static int
intelgpio_pic_vector(struct intsrc *isrc)
{
	struct intelgpio_isrc *gi = (struct intelgpio_isrc *)isrc;

	return (gi->gi_irq);
}

static int
intelgpio_pic_source_pending(struct intsrc *isrc)
{
	struct intelgpio_isrc *gi = (struct intelgpio_isrc *)isrc;
	struct intelgpio_softc *sc = gi->gi_sc;
	uint32_t is;

	INTELGPIO_LOCK(sc);
	is = intelgpio_gpi_read(gi, sc->sc_plat->gpi_is);
	INTELGPIO_UNLOCK(sc);
	return ((is & gi->gi_mask) != 0);
}

static int
intelgpio_pic_config_intr(struct intsrc *isrc, enum intr_trigger trig,
    enum intr_polarity pol)
{
	struct intelgpio_isrc *gi = (struct intelgpio_isrc *)isrc;
	uint32_t mode;

	if (trig == INTR_TRIGGER_CONFORM && pol == INTR_POLARITY_CONFORM)
		return (0);
	if (trig == INTR_TRIGGER_LEVEL)
		mode = pol == INTR_POLARITY_LOW ?
		    GPIO_INTR_LEVEL_LOW : GPIO_INTR_LEVEL_HIGH;
	else
		mode = pol == INTR_POLARITY_LOW ?
		    GPIO_INTR_EDGE_FALLING : GPIO_INTR_EDGE_RISING;
	if (mode != (gi->gi_mode & GPIO_INTR_MASK)) {
		gi->gi_mode = mode;
		gi->gi_level = trig == INTR_TRIGGER_LEVEL;
		intelgpio_program_pad(gi);
	}
	return (0);
}

static int
intelgpio_pic_assign_cpu(struct intsrc *isrc, u_int apic_id)
{

	/* Pins share the parent cascade IRQ; no per-pin affinity. */
	return (EOPNOTSUPP);
}

static void
intelgpio_pic_resume(struct pic *pic, bool suspend_cancelled)
{
	struct intelgpio_softc *sc =
	    __containerof(pic, struct intelgpio_softc, sc_pic);
	struct intelgpio_isrc *gi;

	if (suspend_cancelled)
		return;
	SLIST_FOREACH(gi, &sc->sc_isrcs, gi_next) {
		if (!gi->gi_mapped)
			continue;
		intelgpio_program_pad(gi);
		if (gi->gi_isrc.is_handlers > 0)
			intelgpio_pic_enable_source(&gi->gi_isrc);
	}
}

static struct pic intelgpio_pic_template = {
	.pic_enable_source = intelgpio_pic_enable_source,
	.pic_disable_source = intelgpio_pic_disable_source,
	.pic_eoi_source = intelgpio_pic_eoi_source,
	.pic_enable_intr = intelgpio_pic_enable_intr,
	.pic_disable_intr = intelgpio_pic_disable_intr,
	.pic_vector = intelgpio_pic_vector,
	.pic_source_pending = intelgpio_pic_source_pending,
	.pic_config_intr = intelgpio_pic_config_intr,
	.pic_assign_cpu = intelgpio_pic_assign_cpu,
	.pic_resume = intelgpio_pic_resume,
};

static int
intelgpio_intr_filter(void *arg)
{
	struct intelgpio_softc *sc = arg;
	struct trapframe *frame = curthread->td_intr_frame;
	struct intelgpio_isrc *gi;
	uint32_t pending;

	/*
	 * Entries are only ever appended to sc_isrcs (see GPIO_UNMAP_INTR
	 * note above), so it is safe to drop the lock around dispatch.
	 */
	INTELGPIO_LOCK(sc);
	SLIST_FOREACH(gi, &sc->sc_isrcs, gi_next) {
		pending = intelgpio_gpi_read(gi, sc->sc_plat->gpi_is) &
		    intelgpio_gpi_read(gi, sc->sc_plat->gpi_ie) & gi->gi_mask;
		if (pending == 0)
			continue;
		/* An edge arriving during dispatch must not be lost. */
		if (!gi->gi_level)
			intelgpio_gpi_write(gi, sc->sc_plat->gpi_is,
			    gi->gi_mask);
		INTELGPIO_UNLOCK(sc);
		intr_execute_handlers(&gi->gi_isrc, frame);
		INTELGPIO_LOCK(sc);
	}
	INTELGPIO_UNLOCK(sc);
	return (FILTER_HANDLED);
}

static int
intelgpio_map_intr(device_t dev, uint32_t pin, uint32_t intr_mode, u_int *irqp)
{
	struct intelgpio_softc *sc;
	struct intelgpio_isrc *gi;
	bus_size_t pad_off;
	uint32_t mask, mode, own;
	u_int irq;
	int com, error, reg;

	sc = device_get_softc(dev);
	if (sc->sc_plat->gpi_is == 0 || sc->sc_irq_res == NULL)
		return (EOPNOTSUPP);

	mode = intr_mode & GPIO_INTR_MASK & ~GPIO_INTR_ATTACHED;
	if (mode == 0 || (mode & (mode - 1)) != 0)
		return (EINVAL);

	if (intelgpio_gpio_to_gpi(sc, pin, &com, &reg, &mask, &pad_off) != 0)
		return (EINVAL);

	/* Refuse pads firmware kept for itself (ACPI SCI events etc). */
	if (sc->sc_plat->hostown != 0) {
		own = bus_read_4(sc->sc_mem_res[com],
		    sc->sc_plat->hostown + reg * 4);
		if ((own & mask) == 0) {
			device_printf(dev,
			    "pin %u is ACPI-owned, not mapping\n", pin);
			return (EPERM);
		}
	}

	/* Re-mapping an already known pin returns the existing IRQ. */
	INTELGPIO_LOCK(sc);
	SLIST_FOREACH(gi, &sc->sc_isrcs, gi_next) {
		if (gi->gi_pin != pin)
			continue;
		if (gi->gi_mapped && (gi->gi_mode & GPIO_INTR_MASK) != mode) {
			INTELGPIO_UNLOCK(sc);
			return (EBUSY);
		}
		gi->gi_mode = mode;
		gi->gi_level = (mode & GPIO_INTR_EDGE_MASK) == 0;
		gi->gi_mapped = true;
		INTELGPIO_UNLOCK(sc);
		intelgpio_program_pad(gi);
		*irqp = gi->gi_irq;
		return (0);
	}
	INTELGPIO_UNLOCK(sc);

	error = gpio_irq_alloc(&irq);
	if (error != 0)
		return (error);

	gi = malloc(sizeof(*gi), M_INTELGPIO, M_WAITOK | M_ZERO);
	gi->gi_isrc.is_pic = &sc->sc_pic;
	gi->gi_sc = sc;
	gi->gi_pin = pin;
	gi->gi_irq = irq;
	gi->gi_mode = mode;
	gi->gi_com = com;
	gi->gi_reg = reg;
	gi->gi_mask = mask;
	gi->gi_pad_off = pad_off;
	gi->gi_level = (mode & GPIO_INTR_EDGE_MASK) == 0;
	gi->gi_mapped = true;

	intelgpio_program_pad(gi);

	error = intr_register_source(&gi->gi_isrc);
	if (error != 0) {
		free(gi, M_INTELGPIO);
		gpio_irq_release(irq);
		return (error);
	}
	nexus_add_irq(irq);

	INTELGPIO_LOCK(sc);
	SLIST_INSERT_HEAD(&sc->sc_isrcs, gi, gi_next);
	INTELGPIO_UNLOCK(sc);

	*irqp = irq;
	return (0);
}

static int
intelgpio_unmap_intr(device_t dev, u_int irq)
{
	struct intelgpio_softc *sc;
	struct intelgpio_isrc *gi;

	sc = device_get_softc(dev);
	INTELGPIO_LOCK(sc);
	SLIST_FOREACH(gi, &sc->sc_isrcs, gi_next) {
		if (gi->gi_irq != irq)
			continue;
		if (gi->gi_isrc.is_handlers > 0) {
			INTELGPIO_UNLOCK(sc);
			return (EBUSY);
		}
		gi->gi_mapped = false;
		INTELGPIO_UNLOCK(sc);
		/*
		 * The interrupt source stays registered (x86 cannot
		 * unregister them); disable the pad and keep the entry
		 * and IRQ number for a later re-map of the same pin.
		 */
		intelgpio_pic_disable_intr(&gi->gi_isrc);
		intelgpio_pic_disable_source(&gi->gi_isrc, PIC_EOI);
		return (0);
	}
	INTELGPIO_UNLOCK(sc);
	return (EINVAL);
}

static void
intelgpio_intr_attach(struct intelgpio_softc *sc)
{
	device_t dev = sc->sc_dev;

	if (sc->sc_plat->gpi_is == 0)
		return;

	sc->sc_irq_rid = 0;
	sc->sc_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &sc->sc_irq_rid, RF_ACTIVE | RF_SHAREABLE);
	if (sc->sc_irq_res == NULL) {
		device_printf(dev,
		    "cannot allocate cascade IRQ, interrupts disabled\n");
		return;
	}
	if (bus_setup_intr(dev, sc->sc_irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
	    intelgpio_intr_filter, NULL, sc, &sc->sc_irq_ih) != 0) {
		device_printf(dev,
		    "cannot setup cascade IRQ, interrupts disabled\n");
		bus_release_resource(dev, SYS_RES_IRQ, sc->sc_irq_rid,
		    sc->sc_irq_res);
		sc->sc_irq_res = NULL;
		return;
	}
	sc->sc_pic = intelgpio_pic_template;
	intr_register_pic(&sc->sc_pic);
}

int
intelgpio_probe(device_t dev, const struct intelgpio_platform *plat)
{
	int rv;

	if (acpi_disabled("intelgpio"))
		return (ENXIO);

	rv = ACPI_ID_PROBE(device_get_parent(dev), dev, plat->hids, NULL);
	if (rv <= 0)
		device_set_desc(dev, plat->desc);
	return (rv);
}

static int
intelgpio_detach(device_t dev)
{
	struct intelgpio_softc *sc;
	int i;

	sc = device_get_softc(dev);

	/*
	 * Registered interrupt sources cannot be unregistered from the
	 * x86 interrupt code; refuse to go away once a pin was mapped.
	 */
	if (!SLIST_EMPTY(&sc->sc_isrcs))
		return (EBUSY);

	if (sc->sc_irq_ih != NULL)
		bus_teardown_intr(dev, sc->sc_irq_res, sc->sc_irq_ih);
	if (sc->sc_irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, sc->sc_irq_rid,
		    sc->sc_irq_res);

	if (sc->sc_busdev != NULL)
		gpiobus_detach_bus(dev);

	for (i = 0; i < sc->sc_plat->ncommunities; i++) {
		if (sc->sc_mem_res[i] != NULL)
			bus_release_resource(dev, SYS_RES_MEMORY, i,
			    sc->sc_mem_res[i]);
	}

	INTELGPIO_LOCK_DESTROY(sc);
	return (0);
}

int
intelgpio_attach(device_t dev, const struct intelgpio_platform *plat)
{
	struct intelgpio_softc *sc;
	bool have_res;
	int i;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	sc->sc_plat = plat;

	KASSERT(plat->ncommunities <= INTELGPIO_MAX_COMMUNITIES,
	    ("too many communities: %d > %d", plat->ncommunities,
		INTELGPIO_MAX_COMMUNITIES));

	INTELGPIO_LOCK_INIT(sc);
	SLIST_INIT(&sc->sc_isrcs);

	have_res = false;
	for (i = 0; i < plat->ncommunities; i++) {
		sc->sc_mem_res[i] = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
		    &i, RF_ACTIVE);
		if (sc->sc_mem_res[i] == NULL) {
			device_printf(dev,
			    "failed to allocate memory resource "
			    "for community %d\n",
			    i);
			continue;
		}
		sc->sc_padbar[i] = bus_read_4(sc->sc_mem_res[i],
		    INTELGPIO_PADBAR_REG);
		have_res = true;
	}

	if (!have_res) {
		device_printf(dev, "no memory resources found\n");
		INTELGPIO_LOCK_DESTROY(sc);
		return (ENXIO);
	}

	intelgpio_intr_attach(sc);

	sc->sc_busdev = gpiobus_add_bus(dev);
	if (sc->sc_busdev == NULL) {
		device_printf(dev, "failed to add gpiobus\n");
		intelgpio_detach(dev);
		return (ENXIO);
	}

	bus_attach_children(dev);

	return (0);
}


static device_method_t intelgpio_methods[] = {
	DEVMETHOD(device_detach, intelgpio_detach),

	DEVMETHOD(gpio_get_bus, intelgpio_get_bus),
	DEVMETHOD(gpio_pin_max, intelgpio_pin_max),
	DEVMETHOD(gpio_pin_getname, intelgpio_pin_getname),
	DEVMETHOD(gpio_pin_getcaps, intelgpio_pin_getcaps),
	DEVMETHOD(gpio_pin_getflags, intelgpio_pin_getflags),
	DEVMETHOD(gpio_pin_setflags, intelgpio_pin_setflags),
	DEVMETHOD(gpio_pin_get, intelgpio_pin_get),
	DEVMETHOD(gpio_pin_set, intelgpio_pin_set),
	DEVMETHOD(gpio_pin_toggle, intelgpio_pin_toggle),
	DEVMETHOD(gpio_map_intr, intelgpio_map_intr),
	DEVMETHOD(gpio_unmap_intr, intelgpio_unmap_intr),

	DEVMETHOD_END
};

DEFINE_CLASS_0(intelgpio, intelgpio_driver, intelgpio_methods,
    sizeof(struct intelgpio_softc));
