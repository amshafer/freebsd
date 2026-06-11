/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Abdelkader Boudih <freebsd@seuros.com>
 */

/*
 * intel_pmc -- Intel PCH Power Management Controller driver
 *
 * Exposes sleep state residency counters and LTR block status
 * for debugging S0ix (Modern Standby) power states.
 *
 * The PMC register space (PWRM) is mapped via the PWRM_BASE register
 * at PCI config offset 0x48, not through BAR0.  See Intel 100 Series
 * Chipset Family PCH Datasheet Vol 2, section 4.3.53.
 *
 * The SLP_S0_RESIDENCY counter increments (100 us per step) while the
 * PCH SLP_S0 signal is asserted.  SLP_S0 is asserted only when all of
 * the following conditions are met simultaneously:
 *   - CPU in Package C10 (deepest package C-state)
 *   - PCH in low power mode
 *   - ModPhy lanes power-gated
 *   - PLLs idle
 * A counter that never advances indicates S0ix transitions are not
 * occurring.  The counter is 32-bit and wraps after approximately
 * 4.97 days of cumulative S0ix residency.
 *
 * LTR_IGNORE (PWRM+0x30C) has one bit per IP block.  Setting a bit
 * tells the PMC to ignore that IP's LTR requirements, allowing S0ix
 * entry even if the IP would otherwise block it.  A clear bit means
 * the IP's LTR requirements are considered and can prevent S0ix.
 *
 * SPT bit assignments (0-16):
 *   0=SOUTHPORT_A     1=SOUTHPORT_B     2=SATA
 *   3=GIGABIT_ETHERNET 4=XHCI           5=reserved
 *   6=ME              7=EVA             8=SOUTHPORT_C
 *   9=HD_AUDIO       10=LPSS           11=SOUTHPORT_D
 *  12=SOUTHPORT_E    13=CAMERA         14=ESPI
 *  15=SCC            16=ISH
 *
 * Ref: Intel 100 Series Chipset Family PCH Datasheet Vol 2, section 4.3
 * Ref: Linux commit 9c2ee19987ef (initial SPT PMC support)
 * Ref: Linux commit 2eb150558bb7 (LTR IP block documentation)
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/errno.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <machine/pmap.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

/*
 * Sunrise Point (SPT) PMC Register Definitions
 * Reference: Intel 100 Series Chipset Family PCH Datasheet Vol 2
 */
#define SPT_PMC_PWRM_BASE_OFFSET	0x48	/* PCI config offset for PWRM */
#define SPT_PMC_PWRM_BASE_MASK		0xfffff000
#define SPT_PMC_MMIO_SIZE		0x1000

#define SPT_PMC_SLP_S0_RES_OFFSET	0x13C
#define SPT_PMC_SLP_S0_RES_STEP	100	/* 100us per datasheet */
#define SPT_PMC_LTR_IGNORE_OFFSET	0x30C
#define SPT_PMC_LTR_IGNORE_BITS	17	/* bits 0-16 are valid IP blocks */

struct intel_pmc_softc {
	bus_space_tag_t		bst;
	bus_space_handle_t	bsh;
	int			access_denied;
};

static const struct pci_device_table intel_pmc_devices[] = {
	/* Only Sunrise Point - other generations have different layouts */
	{ PCI_DEV(0x8086, 0x9d21), PCI_DESCR("Sunrise Point-LP PMC")},
	{ PCI_DEV(0x8086, 0xa121), PCI_DESCR("Sunrise Point-H PMC")},
};

static inline uint32_t
intel_pmc_read(struct intel_pmc_softc *sc, uint32_t offset)
{

	return (bus_space_read_4(sc->bst, sc->bsh, offset));
}

static inline void
intel_pmc_write(struct intel_pmc_softc *sc, uint32_t offset, uint32_t val)
{

	bus_space_write_4(sc->bst, sc->bsh, offset, val);
}

static int
intel_pmc_slp_s0_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intel_pmc_softc *sc = oidp->oid_arg1;
	uint64_t val, usec;

	if (sc->access_denied)
		return (EACCES);
	val = intel_pmc_read(sc, SPT_PMC_SLP_S0_RES_OFFSET);
	usec = val * SPT_PMC_SLP_S0_RES_STEP;
	return (sysctl_handle_64(oidp, &usec, 0, req));
}

static int
intel_pmc_ltr_ignore_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intel_pmc_softc *sc = oidp->oid_arg1;
	uint32_t val;
	int error;

	if (sc->access_denied)
		return (EACCES);
	val = intel_pmc_read(sc, SPT_PMC_LTR_IGNORE_OFFSET);
	error = sysctl_handle_32(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	/* Only bits 0-16 are valid IP blocks */
	val &= (1u << SPT_PMC_LTR_IGNORE_BITS) - 1;
	intel_pmc_write(sc, SPT_PMC_LTR_IGNORE_OFFSET, val);
	return (0);
}

static int
intel_pmc_probe(device_t dev)
{
	const struct pci_device_table *tbl;

	tbl = PCI_MATCH(dev, intel_pmc_devices);
	if (tbl == NULL)
		return (ENXIO);
	device_set_desc(dev, tbl->descr);
	return (BUS_PROBE_DEFAULT);
}

static int
intel_pmc_attach(device_t dev)
{
	struct intel_pmc_softc *sc = device_get_softc(dev);
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *tree;
	vm_paddr_t pwrm_base;
	uint32_t slp_s0, ltr_ign;

	pwrm_base = pci_read_config(dev, SPT_PMC_PWRM_BASE_OFFSET, 4) &
	    SPT_PMC_PWRM_BASE_MASK;
	if (pwrm_base == 0) {
		device_printf(dev, "PWRM base not configured by firmware\n");
		return (ENXIO);
	}

	sc->bst = X86_BUS_SPACE_MEM;
	sc->bsh = (bus_space_handle_t)pmap_mapdev(pwrm_base,
	    SPT_PMC_MMIO_SIZE);

	/*
	 * Detect firmware access lock: both key registers returning
	 * 0xFFFFFFFF indicates PCIe Unsupported Request completion,
	 * meaning the PMC MMIO BAR is inaccessible.
	 */
	slp_s0 = intel_pmc_read(sc, SPT_PMC_SLP_S0_RES_OFFSET);
	ltr_ign = intel_pmc_read(sc, SPT_PMC_LTR_IGNORE_OFFSET);

	if (slp_s0 == 0xFFFFFFFF && ltr_ign == 0xFFFFFFFF) {
		sc->access_denied = 1;
		device_printf(dev, "PMC registers inaccessible "
		    "(locked by firmware)\n");
	} else
		sc->access_denied = 0;

	ctx = device_get_sysctl_ctx(dev);
	tree = device_get_sysctl_tree(dev);

	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "access_denied", CTLFLAG_RD | CTLFLAG_MPSAFE, &sc->access_denied,
	    0, "Firmware locked PMC registers");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "ltr_ignore",
	    CTLTYPE_U32 | CTLFLAG_RW | CTLFLAG_MPSAFE,
	    sc, 0, intel_pmc_ltr_ignore_sysctl, "IU",
	    "LTR ignore mask: set bit = ignore IP, allowing S0ix");

	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "slp_s0_residency",
	    CTLTYPE_U64 | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    sc, 0, intel_pmc_slp_s0_sysctl, "QU",
	    "SLP_S0 residency time in microseconds");

	return (0);
}

static int
intel_pmc_detach(device_t dev)
{
	struct intel_pmc_softc *sc = device_get_softc(dev);

	pmap_unmapdev((void *)sc->bsh, SPT_PMC_MMIO_SIZE);
	return (0);
}

static device_method_t intel_pmc_methods[] = {
	DEVMETHOD(device_probe,		intel_pmc_probe),
	DEVMETHOD(device_attach,	intel_pmc_attach),
	DEVMETHOD(device_detach,	intel_pmc_detach),
	DEVMETHOD_END
};

static driver_t intel_pmc_driver = {
	"intel_pmc",
	intel_pmc_methods,
	sizeof(struct intel_pmc_softc)
};

DRIVER_MODULE(intel_pmc, pci, intel_pmc_driver, 0, 0);
PCI_PNP_INFO(intel_pmc_devices);
MODULE_VERSION(intel_pmc, 1);
MODULE_DEPEND(intel_pmc, pci, 1, 1, 1);
