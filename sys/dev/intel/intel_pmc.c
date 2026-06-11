/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Abdelkader Boudih <seuros@FreeBSD.org>
 */

/*
 * intel_pmc -- Intel Performance Management Controller (PMC) driver
 *
 * Exposes information and controls from the PMC of Intel platforms.
 * Sunrise Point (SPT, Skylake generation) and Meteor Lake (MTL) are
 * supported.
 *
 * The driver reports the cumulative time spent in low power mode
 * (S0ix, Modern Standby) while in S0, and lets the administrator
 * force the PMC to ignore IP blocks whose Latency Tolerance
 * Reporting (LTR) prevents the platform from entering S0ix.
 *
 * Sunrise Point (SPT / 100-series PCH):
 *   The PMC register space (PWRM) is mapped via the PWRM_BASE register
 *   at PCI config offset 0x48, not through BAR0.  See Intel 100 Series
 *   Chipset Family PCH Datasheet Vol 2, section 4.3.53.
 *
 * Meteor Lake (MTL):
 *   MTL is a multi-die package and has more than one PMC.  The SoC die
 *   PMC ("SoC-M", device ID 0x7e7f) owns the SLP_S0 residency counter and
 *   the LTR aggregate for the SoC die's IPs.  The IO Expander die
 *   ("IOE-P" 0x7ecf on MTL-H, "IOE-M" 0x7ebf on MTL-U) has its own PMC
 *   covering the PCIe Gen5 and Thunderbolt root ports that live on that
 *   die.  The IOE PMC is hidden from PCI enumeration.
 *
 *   Discovery goes through the Shared SRAM device at PCI 0:20:2, which is
 *   the device this driver attaches to.  Its BAR0 holds a small header:
 *     +0x14  64-bit PWRM base of the SoC PMC (low 3 bits are flags)
 *     +0x70  16-bit PCI device ID of the SoC PMC
 *     +0x68  64-bit physical address of the IOE PMC's SSRAM header, or 0
 *     +0x60  same for a PCH PMC (not present on MTL)
 *   The IOE header has the same layout, so its +0x14 and +0x70 give the
 *   IOE PMC's PWRM base and device ID.  This is the same walk the Linux
 *   driver does in ssram_telemetry.c.
 *
 *   If the SSRAM header is unusable the driver falls back to the ACPI
 *   LPIT table, which supplies the physical address of the SLP_S0
 *   residency counter, from which the SoC PWRM base is back-calculated by
 *   subtracting the register offset.  Failing that it uses the MTL
 *   default PWRM base (0xfe000000).  Both fallbacks match Linux's
 *   get_primary_reg_base().  The IOE PMC is only reachable via SSRAM.
 *
 * The SLP_S0_RESIDENCY counter increments while the PCH SLP_S0 signal is
 * asserted.  SLP_S0 is asserted only when all of the following conditions
 * are met simultaneously:
 *   - CPU in Package C10 (deepest package C-state)
 *   - PCH in low power mode
 *   - ModPhy lanes power-gated
 *   - PLLs idle
 * A counter that never advances indicates S0ix transitions are not
 * occurring.  The counter is 32-bit.  The tick is 104 us on SPT and
 * 122 us on TGL and later (including MTL).
 *
 * LTR_IGNORE has one bit per IP block.  Setting a bit tells the PMC to
 * ignore that IP's LTR requirements, allowing S0ix entry even if the IP
 * would otherwise block it.  A clear bit means the IP's LTR requirements
 * are considered and can prevent S0ix.  The bit index of each IP is the
 * index of its entry in the per-generation LTR table below, which is
 * also the order the "ltr" sysctl prints them in.
 *
 * Ref: Intel 100 Series Chipset Family PCH Datasheet Vol 2, section 4.3
 * Ref: Linux commit 9c2ee19987ef (initial SPT PMC support)
 * Ref: Linux drivers/platform/x86/intel/pmc/{core.h,mtl.c,ssram_telemetry.c}
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/errno.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/proc.h>
#include <sys/rman.h>
#include <sys/sbuf.h>
#include <sys/sched.h>
#include <sys/smp.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <machine/cpufunc.h>
#include <machine/resource.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <machine/pmap.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <contrib/dev/acpica/include/actbl2.h>
#include <dev/acpica/acpivar.h>

/*
 * One per-IP LTR register.  Entries whose index is below the generation's
 * ltr_ignore_bits correspond to bits in LTR_IGNORE.  The trailing entries
 * (CURRENT_PLATFORM, AGGREGATED_SYSTEM) are informational only.
 */
struct intel_pmc_ltr {
	const char	*name;
	uint32_t	 offset;
};

/*
 * One named bit in a 32-bit PMC status register (LPM status, PPFEAR).
 */
struct intel_pmc_bit {
	const char	*name;
	uint32_t	 mask;
};

/* MTL LPM status and power gating bit-name tables. */
#include "intel_pmc_mtl.h"

/*
 * Low power mode (S0ix substate) bookkeeping, common to TGL and later.
 *
 * The PMC tracks up to eight substates.  LPM_EN says which are enabled,
 * LPM_PRI gives each a priority level, and there is a residency counter
 * per substate.  The LPM status registers hold one readiness bit per IP
 * (power gated, in D3, VNN released, clock off, ...).  The PMC latches a
 * copy of them either when the SoC enters package C10 or when it enters a
 * chosen substate, selected through the latch enable register.  In that
 * copy a clear bit is an IP that was not ready at that moment, which is
 * what kept the PMC from going deeper.
 */
#define	LPM_MAX_NUM_MODES	8
#define	LPM_STS_LATCH_MODE	(1u << 31)	/* set: latch on substate, clear: on C10 */
#define	LPM_DEFAULT_PRI		{ 7, 6, 2, 5, 4, 1, 3, 0 }
#define	ETR3_CLEAR_LPM_EVENTS	(1u << 28)

static const char *const intel_pmc_lpm_modes[LPM_MAX_NUM_MODES] = {
	"S0i2.0", "S0i2.1", "S0i2.2", "S0i3.0",
	"S0i3.1", "S0i3.2", "S0i3.3", "S0i3.4",
};

/*
 * Per-generation PMC register layout and PWRM discovery method.
 */
struct intel_pmc_gen {
	const char	*name;
	uint32_t	slp_s0_offset;		/* SLP_S0 residency reg, 0 if none */
	uint32_t	slp_s0_step_us;		/* microseconds per counter step */
	uint32_t	ltr_ignore_offset;	/* LTR_IGNORE reg offset */
	uint32_t	ltr_ignore_bits;	/* number of valid LTR bits */
	uint32_t	mmio_size;		/* PWRM MMIO mapping size */
	const struct intel_pmc_ltr *ltr;	/* per-IP LTR regs, name==NULL ends */
	bool		use_ssram;		/* PWRM base(s) from SSRAM BAR0 */
	bool		use_lpit;		/* find PWRMBASE via ACPI LPIT */
	uint8_t		pwrm_cfg_offset;	/* PCI cfg offset (if neither) */
	vm_paddr_t	pwrm_default;		/* fallback base (if use_lpit) */
	const struct intel_pmc_gen *ioe;	/* secondary PMC layout, or NULL */

	/* Low power mode (S0ix substate) status.  NULL lpm_maps if absent. */
	const struct intel_pmc_bit *const *lpm_maps;
	uint32_t	lpm_num_maps;		/* 32-bit status regs, one per map */
	uint32_t	lpm_status_offset;	/* latched copy */
	uint32_t	lpm_live_status_offset;
	uint32_t	lpm_latch_en_offset;
	uint32_t	lpm_en_offset;
	uint32_t	lpm_priority_offset;
	uint32_t	lpm_residency_offset;	/* one counter per substate */
	uint32_t	lpm_res_step_x2;	/* residency tick in us, times 2 */
	uint32_t	etr3_offset;		/* clears latched events, 0 if none */

	/* PCH IP power gating status (PPFEAR).  NULL pfear_maps if absent. */
	const struct intel_pmc_bit *const *pfear_maps;
	uint32_t	pfear_offset;
	uint32_t	pfear_buckets;		/* number of bytes to read */
};

/*
 * Sunrise Point (SPT) -- Intel 100/200-series PCH
 * Ref: Intel 100 Series PCH Datasheet Vol 2, section 4.3
 */
static const struct intel_pmc_ltr spt_ltr[] = {
	{ "SOUTHPORT_A",	0x360 },
	{ "SOUTHPORT_B",	0x364 },
	{ "SATA",		0x368 },
	{ "GIGABIT_ETHERNET",	0x36C },
	{ "XHCI",		0x370 },
	{ "Reserved",		0x374 },
	{ "ME",			0x378 },
	{ "EVA",		0x37C },
	{ "SOUTHPORT_C",	0x380 },
	{ "HD_AUDIO",		0x384 },
	{ "LPSS",		0x38C },
	{ "SOUTHPORT_D",	0x394 },
	{ "SOUTHPORT_E",	0x398 },
	{ "CAMERA",		0x390 },
	{ "ESPI",		0x39C },
	{ "SCC",		0x3A0 },
	{ "ISH",		0x3A4 },
	/* Not ignorable. */
	{ "CURRENT_PLATFORM",	0x350 },
	{ "AGGREGATED_SYSTEM",	0x354 },
	{ NULL,			0 },
};

/*
 * ACTL (PCI config 0x44) bit 8 says whether the PWRM range is decoded.
 * Only SPT finds its PWRM base in PCI config space.
 */
#define SPT_PMC_ACTL_OFFSET		0x44
#define SPT_PMC_ACTL_PWRM_EN		(1u << 8)

static const struct intel_pmc_gen spt_gen = {
	.name			= "SPT",
	.slp_s0_offset		= 0x13C,
	.slp_s0_step_us		= 104,
	.ltr_ignore_offset	= 0x30C,
	.ltr_ignore_bits	= 17,
	.mmio_size		= 0x1000,
	.ltr			= spt_ltr,
	.pwrm_cfg_offset	= 0x48,
};

/*
 * Meteor Lake (MTL) -- IO Expander die PMC (IOE-P 0x7ecf, IOE-M 0x7ebf)
 * Ref: Linux mtl_ioep_reg_map / mtl_ioep_ltr_show_map
 * Hidden from PCI enumeration, found through the SoC PMC's SSRAM header.
 * Has no SLP_S0 counter of its own.
 */
static const struct intel_pmc_ltr mtl_ioe_ltr[] = {
	{ "SOUTHPORT_A",	0x1B60 },
	{ "SOUTHPORT_B",	0x1B64 },
	{ "SATA",		0x1B68 },
	{ "GIGABIT_ETHERNET",	0x1B6C },
	{ "XHCI",		0x1B70 },
	{ "SOUTHPORT_F",	0x1C00 },
	{ "ME",			0x1B78 },
	{ "SATA1",		0x1B7C },
	{ "SOUTHPORT_C",	0x1B80 },
	{ "HD_AUDIO",		0x1B84 },
	{ "CNV",		0x1BF0 },
	{ "LPSS",		0x1B8C },
	{ "SOUTHPORT_D",	0x1B94 },
	{ "SOUTHPORT_E",	0x1B98 },
	{ "SATA2",		0x1B90 },
	{ "ESPI",		0x1B9C },
	{ "SCC",		0x1BA0 },
	{ "Reserved",		0x1BA4 },
	{ "UFSX2",		0x1BF8 },
	{ "EMMC",		0x1BF4 },
	{ "WIGIG",		0x1BFC },
	{ "THC0",		0x1C04 },
	{ "THC1",		0x1C08 },
	{ "SOUTHPORT_G",	0x1B74 },
	/* Not ignorable. */
	{ "CURRENT_PLATFORM",	0x1B50 },
	{ "AGGREGATED_SYSTEM",	0x1B54 },
	{ NULL,			0 },
};

static const struct intel_pmc_gen mtl_ioe_gen = {
	.name			= "MTL-IOE",
	.slp_s0_offset		= 0,
	.ltr_ignore_offset	= 0x1B0C,
	.ltr_ignore_bits	= 24,
	.mmio_size		= 0x23A4,
	.ltr			= mtl_ioe_ltr,
	.lpm_maps		= mtl_ioep_lpm_maps,
	.lpm_num_maps		= 14,
	.lpm_status_offset	= 0x1700,
	.lpm_live_status_offset	= 0x175C,
	.lpm_latch_en_offset	= 0x16F8,
	.lpm_en_offset		= 0x1798,
	.lpm_priority_offset	= 0x179C,
	.lpm_residency_offset	= 0x17A0,
	.lpm_res_step_x2	= 61,
	.pfear_maps		= ext_mtl_ioep_pfear_map,
	.pfear_offset		= 0x1D90,
	.pfear_buckets		= 10,
};

/*
 * Meteor Lake (MTL) -- SoC die PMC (SoC-M, Shared SRAM device PCI 0:20:2)
 * Ref: Linux mtl_socm_reg_map / mtl_socm_ltr_show_map
 */
static const struct intel_pmc_ltr mtl_soc_ltr[] = {
	{ "SOUTHPORT_A",	0x1B60 },
	{ "SOUTHPORT_B",	0x1B64 },
	{ "SATA",		0x1B68 },
	{ "GIGABIT_ETHERNET",	0x1B6C },
	{ "XHCI",		0x1B70 },
	{ "SOUTHPORT_F",	0x1C00 },
	{ "ME",			0x1B78 },
	{ "SATA1",		0x1B7C },
	{ "SOUTHPORT_C",	0x1B80 },
	{ "HD_AUDIO",		0x1B84 },
	{ "CNV",		0x1BF0 },
	{ "LPSS",		0x1B8C },
	{ "SOUTHPORT_D",	0x1B94 },
	{ "SOUTHPORT_E",	0x1B98 },
	{ "SATA2",		0x1B90 },
	{ "ESPI",		0x1B9C },
	{ "SCC",		0x1BA0 },
	{ "ISH",		0x1BA4 },
	{ "UFSX2",		0x1BF8 },
	{ "EMMC",		0x1BF4 },
	{ "WIGIG",		0x1BFC },
	{ "THC0",		0x1C04 },
	{ "THC1",		0x1C08 },
	{ "SOUTHPORT_G",	0x1B74 },
	{ "ESE",		0x1BAC },
	{ "IOE_PMC",		0x1C0C },
	/* Not ignorable. */
	{ "CURRENT_PLATFORM",	0x1B50 },
	{ "AGGREGATED_SYSTEM",	0x1B54 },
	{ NULL,			0 },
};

static const struct intel_pmc_gen mtl_gen = {
	.name			= "MTL-SoC",
	.slp_s0_offset		= 0x193C,
	.slp_s0_step_us		= 122,
	.ltr_ignore_offset	= 0x1B0C,
	.ltr_ignore_bits	= 26,
	.mmio_size		= 0x2708,
	.ltr			= mtl_soc_ltr,
	.use_ssram		= true,
	.use_lpit		= true,
	.pwrm_default		= 0xfe000000,
	.ioe			= &mtl_ioe_gen,
	.lpm_maps		= mtl_socm_lpm_maps,
	.lpm_num_maps		= 14,
	.lpm_status_offset	= 0x1700,
	.lpm_live_status_offset	= 0x175C,
	.lpm_latch_en_offset	= 0x16F8,
	.lpm_en_offset		= 0x1798,
	.lpm_priority_offset	= 0x179C,
	.lpm_residency_offset	= 0x17A0,
	.lpm_res_step_x2	= 61,
	.etr3_offset		= 0x1048,
	.pfear_maps		= ext_mtl_socm_pfear_map,
	.pfear_offset		= 0x1D90,
	.pfear_buckets		= 8,
};

/* Shared SRAM header layout (Linux ssram_telemetry.c). */
#define	SSRAM_HDR_SIZE		0x100
#define	SSRAM_PWRM_OFFSET	0x14
#define	SSRAM_PCH_OFFSET	0x60
#define	SSRAM_IOE_OFFSET	0x68
#define	SSRAM_DEVID_OFFSET	0x70
#define	SSRAM_BASE_ADDR_MASK	(~(uint64_t)0x7)

/* Device IDs that may appear in the IOE SSRAM header. */
#define	PMC_DEVID_MTL_IOEP	0x7ecf
#define	PMC_DEVID_MTL_IOEM	0x7ebf

/* LTR register decode (PCIe LTR ECN, as in Linux pmc_core_ltr_show). */
#define	LTR_REQ_SNOOP		(1u << 15)
#define	LTR_REQ_NONSNOOP	(1u << 31)
#define	LTR_VAL_MASK		0x3ff
#define	LTR_SCALE_SHIFT		10
#define	LTR_SCALE_MASK		0x7

/*
 * One mapped PMC.  A device has one (SoC) or two (SoC and IOE).
 */
struct intel_pmc_regs {
	const struct intel_pmc_gen *gen;
	bus_space_tag_t		 bst;
	bus_space_handle_t	 bsh;
	uint64_t		 base;
	uint16_t		 devid;
	int			 access_denied;
	bool			 mapped;
	/* Enabled S0ix substates, lowest priority first. */
	uint8_t			 lpm_modes[LPM_MAX_NUM_MODES];
	uint8_t			 num_lpm_modes;
};

struct intel_pmc_softc {
	device_t		 dev;
	struct intel_pmc_regs	 soc;
	struct intel_pmc_regs	 ioe;
	struct resource		*ssram;
	int			 ssram_rid;
};

/*
 * Only Sunrise Point - other generations have different layouts.
 * 0xa121 is documented in the Intel 100 Series Chipset Family PCH
 * Datasheet Vol 2.  Sunrise Point-LP has no public datasheet; 0x9d21
 * comes from the pci_vendors database, where Linux also sourced it.
 */
static const struct pci_device_table intel_pmc_devices[] = {
	/* Sunrise Point */
	{ PCI_DEV(0x8086, 0x9d21), .driver_data = (uintptr_t)&spt_gen,
	  PCI_DESCR("Sunrise Point-LP PMC") },
	{ PCI_DEV(0x8086, 0xa121), .driver_data = (uintptr_t)&spt_gen,
	  PCI_DESCR("Sunrise Point-H PMC") },
	/* Meteor Lake -- Shared SRAM device hosts the SoC-M PMC */
	{ PCI_DEV(0x8086, 0x7e7f), .driver_data = (uintptr_t)&mtl_gen,
	  PCI_DESCR("Meteor Lake-P/H PMC") },
};

static inline uint32_t
intel_pmc_read(struct intel_pmc_regs *r, uint32_t offset)
{

	return (bus_space_read_4(r->bst, r->bsh, offset));
}

static inline void
intel_pmc_write(struct intel_pmc_regs *r, uint32_t offset, uint32_t val)
{

	bus_space_write_4(r->bst, r->bsh, offset, val);
}

/*
 * Walk the ACPI LPIT table to find the physical address of the SLP_S0
 * residency counter.  The first enabled, memory-mapped Native C-state
 * entry is used.  Returns the PWRM base address on success, or 0 if no
 * suitable entry is found.
 */
static vm_paddr_t
intel_pmc_lpit_pwrmbase(const struct intel_pmc_gen *gen)
{
	ACPI_TABLE_LPIT *lpit;
	ACPI_LPIT_NATIVE *entry;
	ACPI_STATUS status;
	uint32_t offset, end;
	vm_paddr_t base;

	status = AcpiGetTable(ACPI_SIG_LPIT, 1, (ACPI_TABLE_HEADER **)&lpit);
	if (ACPI_FAILURE(status))
		return (0);

	base = 0;
	end = lpit->Header.Length;
	offset = sizeof(ACPI_TABLE_LPIT);

	while (offset + sizeof(ACPI_LPIT_HEADER) <= end) {
		entry = (ACPI_LPIT_NATIVE *)((uint8_t *)lpit + offset);
		if (entry->Header.Length < sizeof(ACPI_LPIT_NATIVE)) {
			/* Corrupt or unknown subtable; stop walking. */
			break;
		}
		offset += entry->Header.Length;

		/* Skip non-native, disabled, and no-counter entries. */
		if (entry->Header.Type != ACPI_LPIT_TYPE_NATIVE_CSTATE)
			continue;
		if (entry->Header.Flags & ACPI_LPIT_STATE_DISABLED)
			continue;
		if (entry->Header.Flags & ACPI_LPIT_NO_COUNTER)
			continue;

		/* We need a memory-mapped counter. */
		if (entry->ResidencyCounter.SpaceId !=
		    ACPI_ADR_SPACE_SYSTEM_MEMORY)
			continue;
		if (entry->ResidencyCounter.Address == 0)
			continue;

		/* PWRM base = counter address - register offset within PWRM */
		base = (vm_paddr_t)entry->ResidencyCounter.Address -
		    gen->slp_s0_offset;
		break;
	}

	AcpiPutTable((ACPI_TABLE_HEADER *)lpit);
	return (base);
}

/*
 * Read a PMC's PWRM base and device ID out of an SSRAM header.  Returns
 * false if the header does not look like one (all ones, or no base).
 */
static bool
intel_pmc_ssram_hdr(bus_space_tag_t bst, bus_space_handle_t bsh,
    vm_paddr_t *base, uint16_t *devid)
{
	uint64_t pwrm;

	pwrm = bus_space_read_8(bst, bsh, SSRAM_PWRM_OFFSET);
	if (pwrm == ~(uint64_t)0 || (pwrm & SSRAM_BASE_ADDR_MASK) == 0)
		return (false);

	*base = (vm_paddr_t)(pwrm & SSRAM_BASE_ADDR_MASK);
	*devid = bus_space_read_2(bst, bsh, SSRAM_DEVID_OFFSET);
	return (true);
}

/*
 * Decode LPM_EN and LPM_PRI into the list of enabled substates, lowest
 * priority first, the way Linux's pmc_core_pmc_get_low_power_modes does.
 * LPM_PRI holds a 4-bit level per mode.  It is trusted only if all eight
 * levels are in range and distinct, otherwise the default order is used.
 */
static void
intel_pmc_get_lpm_modes(struct intel_pmc_regs *r)
{
	static const uint8_t def_pri[LPM_MAX_NUM_MODES] = LPM_DEFAULT_PRI;
	uint8_t pri_order[LPM_MAX_NUM_MODES], mode_order[LPM_MAX_NUM_MODES];
	uint32_t lpm_en, lpm_pri;
	bool pri_ok;
	int i, j, p;

	r->num_lpm_modes = 0;
	if (r->gen->lpm_maps == NULL || r->access_denied)
		return;

	/* Bit 31 of LPM_EN is a global enable on MTL, modes are the low byte. */
	lpm_en = intel_pmc_read(r, r->gen->lpm_en_offset) & 0xff;
	lpm_pri = intel_pmc_read(r, r->gen->lpm_priority_offset);

	pri_ok = (lpm_pri != 0);
	for (i = 0; pri_ok && i < LPM_MAX_NUM_MODES; i++) {
		mode_order[i] = (lpm_pri >> (4 * i)) & 0xf;
		if (mode_order[i] >= LPM_MAX_NUM_MODES)
			pri_ok = false;
	}
	for (i = 0; pri_ok && i < LPM_MAX_NUM_MODES - 1; i++)
		for (j = i + 1; j < LPM_MAX_NUM_MODES; j++)
			if (mode_order[i] == mode_order[j])
				pri_ok = false;

	memcpy(pri_order, def_pri, sizeof(pri_order));
	if (pri_ok)
		for (i = 0; i < LPM_MAX_NUM_MODES; i++)
			pri_order[mode_order[i]] = i;

	for (p = LPM_MAX_NUM_MODES - 1; p >= 0; p--) {
		uint8_t mode = pri_order[p];

		if ((lpm_en & (1u << mode)) != 0)
			r->lpm_modes[r->num_lpm_modes++] = mode;
	}
}

/*
 * Map a PMC's PWRM range and check whether firmware has locked it.  Both
 * probe registers reading 0xFFFFFFFF means the reads completed as PCIe
 * Unsupported Requests.
 */
static void
intel_pmc_map(device_t dev, struct intel_pmc_regs *r,
    const struct intel_pmc_gen *gen, vm_paddr_t base, uint16_t devid)
{
	uint32_t probe_a, probe_b;

	r->gen = gen;
	r->base = base;
	r->devid = devid;
	r->bst = X86_BUS_SPACE_MEM;
	r->bsh = (bus_space_handle_t)pmap_mapdev(base, gen->mmio_size);
	r->mapped = true;

	probe_a = intel_pmc_read(r, gen->ltr_ignore_offset);
	probe_b = intel_pmc_read(r, gen->slp_s0_offset != 0 ?
	    gen->slp_s0_offset : gen->ltr[0].offset);

	if (probe_a == 0xFFFFFFFF && probe_b == 0xFFFFFFFF) {
		r->access_denied = 1;
		device_printf(dev, "%s PMC registers inaccessible "
		    "(locked by firmware)\n", gen->name);
	} else
		r->access_denied = 0;

	intel_pmc_get_lpm_modes(r);
}

static void
intel_pmc_unmap(struct intel_pmc_regs *r)
{

	if (r->mapped) {
		pmap_unmapdev((void *)r->bsh, r->gen->mmio_size);
		r->mapped = false;
	}
}

static int
intel_pmc_slp_s0_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intel_pmc_regs *r = oidp->oid_arg1;
	uint64_t val, usec;

	if (r->access_denied)
		return (EACCES);
	val = intel_pmc_read(r, r->gen->slp_s0_offset);
	usec = val * r->gen->slp_s0_step_us;
	return (sysctl_handle_64(oidp, &usec, 0, req));
}

static int
intel_pmc_ltr_ignore_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intel_pmc_regs *r = oidp->oid_arg1;
	uint32_t val, mask;
	int error;

	if (r->access_denied)
		return (EACCES);
	val = intel_pmc_read(r, r->gen->ltr_ignore_offset);
	error = sysctl_handle_32(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	mask = (r->gen->ltr_ignore_bits < 32) ?
	    (1u << r->gen->ltr_ignore_bits) - 1 : ~0u;
	val &= mask;
	intel_pmc_write(r, r->gen->ltr_ignore_offset, val);
	return (0);
}

/*
 * Format one half (snoop or non-snoop) of an LTR register.  With the
 * requirement bit clear the IP has no latency requirement at all and
 * cannot block anything, printed as "none".  With the requirement bit set
 * the value is a tolerance in ns, and 0 ns is the most restrictive
 * possible demand, not the absence of one.  The two must not look alike.
 */
static void
intel_pmc_ltr_format(struct sbuf *sb, uint32_t raw, bool nonsnoop)
{
	static const uint64_t scale[] = { 1, 32, 1024, 32768, 1048576,
	    33554432, 0, 0 };
	uint32_t half, val, sc;

	if (nonsnoop) {
		if ((raw & LTR_REQ_NONSNOOP) == 0) {
			sbuf_printf(sb, "none");
			return;
		}
		half = raw >> 16;
	} else {
		if ((raw & LTR_REQ_SNOOP) == 0) {
			sbuf_printf(sb, "none");
			return;
		}
		half = raw & 0xffff;
	}
	val = half & LTR_VAL_MASK;
	sc = (half >> LTR_SCALE_SHIFT) & LTR_SCALE_MASK;
	sbuf_printf(sb, "%ju ns", (uintmax_t)val * scale[sc]);
}

/*
 * One line per IP: index, name, raw register, snoop and non-snoop latency
 * tolerance, and whether its LTR_IGNORE bit is set.  A small tolerance is
 * what keeps the PMC out of S0ix.  "0 ns" is the worst case: a device that
 * has asserted a requirement with no tolerance at all, typically a device
 * that does not really do LTR sitting behind a port that does.
 */
static int
intel_pmc_ltr_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intel_pmc_regs *r = oidp->oid_arg1;
	const struct intel_pmc_ltr *ltr;
	struct sbuf *sb;
	uint32_t ign, raw;
	int error, i;

	if (r->access_denied)
		return (EACCES);

	sb = sbuf_new_for_sysctl(NULL, NULL, 4096, req);
	if (sb == NULL)
		return (ENOMEM);

	ign = intel_pmc_read(r, r->gen->ltr_ignore_offset);
	sbuf_printf(sb, "\n");
	for (i = 0, ltr = r->gen->ltr; ltr->name != NULL; i++, ltr++) {
		raw = intel_pmc_read(r, ltr->offset);
		sbuf_printf(sb, "%2d %-18s raw=0x%08x snoop=", i, ltr->name,
		    raw);
		intel_pmc_ltr_format(sb, raw, false);
		sbuf_printf(sb, " nonsnoop=");
		intel_pmc_ltr_format(sb, raw, true);
		if ((uint32_t)i < r->gen->ltr_ignore_bits &&
		    (ign & (1u << i)) != 0)
			sbuf_printf(sb, " IGNORED");
		sbuf_printf(sb, "\n");
	}

	error = sbuf_finish(sb);
	sbuf_delete(sb);
	return (error);
}

/*
 * Dump the LPM status registers, one 32-bit register per name table.
 * arg2 selects the latched copy (0) or the live one (1).  In the latched
 * copy, taken when the PMC last hit the latch target (see lpm_latch_mode),
 * a 0 is an IP that was not ready at that moment and so a reason the
 * platform did not go deeper.
 */
static int
intel_pmc_lpm_status_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intel_pmc_regs *r = oidp->oid_arg1;
	const struct intel_pmc_gen *gen = r->gen;
	const struct intel_pmc_bit *bit;
	struct sbuf *sb;
	uint32_t offset, val;
	u_int i;
	int error;

	if (r->access_denied)
		return (EACCES);

	sb = sbuf_new_for_sysctl(NULL, NULL, 8192, req);
	if (sb == NULL)
		return (ENOMEM);

	offset = arg2 ? gen->lpm_live_status_offset : gen->lpm_status_offset;
	sbuf_printf(sb, "\n");
	for (i = 0; i < gen->lpm_num_maps && gen->lpm_maps[i] != NULL; i++) {
		val = intel_pmc_read(r, offset + 4 * i);
		sbuf_printf(sb, "LPM_%s_STATUS_%u: 0x%08x\n",
		    arg2 ? "LIVE" : "LATCHED", i, val);
		for (bit = gen->lpm_maps[i]; bit->name != NULL; bit++)
			sbuf_printf(sb, "  %-34s %u\n", bit->name,
			    (val & bit->mask) != 0 ? 1 : 0);
	}

	error = sbuf_finish(sb);
	sbuf_delete(sb);
	return (error);
}

/*
 * Enabled S0ix substates in priority order, with how long each has been
 * resident.  A platform that never leaves S0 shows zero everywhere.  One
 * that reaches a shallow substate but never SLP_S0 shows residency there
 * and nothing deeper, which narrows the search to the requirements of the
 * next substate.
 */
static int
intel_pmc_lpm_modes_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intel_pmc_regs *r = oidp->oid_arg1;
	const struct intel_pmc_gen *gen = r->gen;
	struct sbuf *sb;
	uint64_t res;
	uint8_t mode;
	int error, i;

	if (r->access_denied)
		return (EACCES);

	sb = sbuf_new_for_sysctl(NULL, NULL, 512, req);
	if (sb == NULL)
		return (ENOMEM);

	sbuf_printf(sb, "\n");
	if (r->num_lpm_modes == 0)
		sbuf_printf(sb, "(no substates enabled)\n");
	for (i = 0; i < r->num_lpm_modes; i++) {
		mode = r->lpm_modes[i];
		res = intel_pmc_read(r, gen->lpm_residency_offset + 4 * mode);
		sbuf_printf(sb, "%-7s residency=%ju us\n",
		    intel_pmc_lpm_modes[mode],
		    (uintmax_t)(res * gen->lpm_res_step_x2 / 2));
	}

	error = sbuf_finish(sb);
	sbuf_delete(sb);
	return (error);
}

/*
 * Select what event latches the LPM status: "c10" for package C10 entry,
 * or the name of an enabled substate for entry into that substate.
 * "clear" discards the latched copy so the next attempt starts clean.
 * Mirrors Linux's lpm_latch_mode debugfs file.
 */
static int
intel_pmc_lpm_latch_mode_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intel_pmc_regs *r = oidp->oid_arg1;
	const struct intel_pmc_gen *gen = r->gen;
	char buf[16];
	uint32_t reg;
	int error, i;

	if (r->access_denied)
		return (EACCES);

	reg = intel_pmc_read(r, gen->lpm_latch_en_offset);
	if ((reg & LPM_STS_LATCH_MODE) == 0)
		strlcpy(buf, "c10", sizeof(buf));
	else {
		strlcpy(buf, "?", sizeof(buf));
		for (i = 0; i < LPM_MAX_NUM_MODES; i++) {
			if ((reg & (1u << i)) != 0) {
				strlcpy(buf, intel_pmc_lpm_modes[i], sizeof(buf));
				break;
			}
		}
	}

	error = sysctl_handle_string(oidp, buf, sizeof(buf), req);
	if (error != 0 || req->newptr == NULL)
		return (error);

	if (strcmp(buf, "clear") == 0) {
		if (gen->etr3_offset == 0)
			return (EOPNOTSUPP);
		reg = intel_pmc_read(r, gen->etr3_offset);
		intel_pmc_write(r, gen->etr3_offset, reg | ETR3_CLEAR_LPM_EVENTS);
		return (0);
	}
	if (strcmp(buf, "c10") == 0) {
		reg = intel_pmc_read(r, gen->lpm_latch_en_offset);
		intel_pmc_write(r, gen->lpm_latch_en_offset,
		    reg & ~LPM_STS_LATCH_MODE);
		return (0);
	}
	for (i = 0; i < r->num_lpm_modes; i++) {
		if (strcmp(buf, intel_pmc_lpm_modes[r->lpm_modes[i]]) == 0) {
			intel_pmc_write(r, gen->lpm_latch_en_offset,
			    LPM_STS_LATCH_MODE | (1u << r->lpm_modes[i]));
			return (0);
		}
	}
	return (EINVAL);
}

/*
 * PCH IP power gating status.  One bit per IP across pfear_buckets bytes.
 * A set bit means the IP is power gated ("Off").  An IP still "On" while
 * the system is otherwise idle is a candidate S0ix blocker.
 */
static int
intel_pmc_pfear_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intel_pmc_regs *r = oidp->oid_arg1;
	const struct intel_pmc_gen *gen = r->gen;
	const struct intel_pmc_bit *bit;
	struct sbuf *sb;
	uint8_t regs[16];
	u_int buckets, i, m, idx, ip;
	int error;

	if (r->access_denied)
		return (EACCES);

	sb = sbuf_new_for_sysctl(NULL, NULL, 4096, req);
	if (sb == NULL)
		return (ENOMEM);

	buckets = MIN(gen->pfear_buckets, nitems(regs));
	for (i = 0; i < buckets; i++)
		regs[i] = bus_space_read_1(r->bst, r->bsh, gen->pfear_offset + i);

	sbuf_printf(sb, "\n");
	ip = 0;
	for (m = 0; gen->pfear_maps[m] != NULL; m++) {
		for (bit = gen->pfear_maps[m], idx = 0;
		    bit->name != NULL && idx < buckets * 8; bit++, idx++, ip++)
			sbuf_printf(sb, "%3u %-34s %s\n", ip, bit->name,
			    (regs[idx / 8] & bit->mask) != 0 ? "Off" : "On");
	}

	error = sbuf_finish(sb);
	sbuf_delete(sb);
	return (error);
}

/*
 * Package C-state residency counters.  These are core MSRs, not PMC
 * registers, but they are the other half of the S0ix picture: SLP_S0 can
 * only be asserted from package C10, so if the PC10 counter does not move
 * across a suspend the CPUs never got deep enough for the PCH side to
 * matter.  Linux's pmc driver samples the same MSRs around suspend for
 * its "S0ix failed" warning.  Package scope, so any CPU will do.
 */
static const struct {
	const char	*name;
	u_int		 msr;
} intel_pmc_pkgc_msrs[] = {
	{ "PC2",	0x60D },
	{ "PC3",	0x3F8 },
	{ "PC6",	0x3F9 },
	{ "PC7",	0x3FA },
	{ "PC8",	0x630 },
	{ "PC9",	0x631 },
	{ "PC10",	0x632 },
}, intel_pmc_corec_msrs[] = {
	/* Core scope: whichever CPU services the sysctl. */
	{ "CC3",	0x3FC },
	{ "CC6",	0x3FD },
	{ "CC7",	0x3FE },
};

/*
 * MSR_PKG_CST_CONFIG_CONTROL.  Bits 3:0 are the deepest package C-state
 * firmware allows, bit 15 locks the register.  The field encoding is the
 * Haswell-and-later client one, which turbostat also uses for CNL, ADL and
 * MTL.  A limit of PC0 means the package can never leave C0 no matter what
 * the cores do, and that is a firmware setting the OS cannot change once
 * locked.
 */
#define	MSR_PKG_CST_CONFIG_CONTROL	0xE2
#define	PKG_CST_LIMIT_MASK		0xF
#define	PKG_CST_LOCK			(1u << 15)

static const char *const intel_pmc_pkg_cst_limits[16] = {
	"PC0", "PC2", "PC3", "PC6", "PC7", "PC7S", "PC8", "PC9",
	"unlimited", "rsvd", "rsvd", "rsvd", "rsvd", "rsvd", "rsvd", "rsvd",
};

static int
intel_pmc_pkgc_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct thread *td = curthread;
	struct sbuf *sb;
	uint64_t val, core[nitems(intel_pmc_corec_msrs)];
	u_int i, cpu;
	int error;

	sb = sbuf_new_for_sysctl(NULL, NULL, 2048, req);
	if (sb == NULL)
		return (ENOMEM);

	sbuf_printf(sb, "\n");
	if (rdmsr_safe(MSR_PKG_CST_CONFIG_CONTROL, &val) == 0) {
		sbuf_printf(sb, "PKG_CST_CONFIG_CONTROL 0x%016jx limit=%s %s\n",
		    (uintmax_t)val,
		    intel_pmc_pkg_cst_limits[val & PKG_CST_LIMIT_MASK],
		    (val & PKG_CST_LOCK) != 0 ? "locked" : "unlocked");
	}
	for (i = 0; i < nitems(intel_pmc_pkgc_msrs); i++) {
		if (rdmsr_safe(intel_pmc_pkgc_msrs[i].msr, &val) != 0)
			continue;
		sbuf_printf(sb, "%-5s %ju\n", intel_pmc_pkgc_msrs[i].name,
		    (uintmax_t)val);
	}

	/*
	 * The core residency MSRs are core scoped, so read them from every
	 * CPU by binding to each in turn.  On a hybrid part the P-cores,
	 * E-cores and low power E-cores account their deepest state in
	 * different counters, and package C10 needs all of them deep, so a
	 * single sample from whichever CPU ran the sysctl says very little.
	 */
	sbuf_printf(sb, "%-6s", "");
	for (i = 0; i < nitems(intel_pmc_corec_msrs); i++)
		sbuf_printf(sb, " %16s", intel_pmc_corec_msrs[i].name);
	sbuf_printf(sb, "\n");
	CPU_FOREACH(cpu) {
		thread_lock(td);
		sched_bind(td, cpu);
		thread_unlock(td);
		for (i = 0; i < nitems(intel_pmc_corec_msrs); i++) {
			if (rdmsr_safe(intel_pmc_corec_msrs[i].msr, &core[i]) != 0)
				core[i] = UINT64_MAX;
		}
		thread_lock(td);
		sched_unbind(td);
		thread_unlock(td);

		sbuf_printf(sb, "cpu%-3u", cpu);
		for (i = 0; i < nitems(intel_pmc_corec_msrs); i++) {
			if (core[i] == UINT64_MAX)
				sbuf_printf(sb, " %16s", "-");
			else
				sbuf_printf(sb, " %16ju", (uintmax_t)core[i]);
		}
		sbuf_printf(sb, "\n");
	}

	error = sbuf_finish(sb);
	sbuf_delete(sb);
	return (error);
}

/*
 * Register the sysctls for one PMC under the given parent.
 */
static void
intel_pmc_add_sysctls(device_t dev, struct sysctl_oid_list *parent,
    struct intel_pmc_regs *r)
{
	struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(dev);

	SYSCTL_ADD_INT(ctx, parent, OID_AUTO,
	    "access_denied", CTLFLAG_RD | CTLFLAG_MPSAFE, &r->access_denied,
	    0, "Firmware locked PMC registers");

	SYSCTL_ADD_U16(ctx, parent, OID_AUTO,
	    "devid", CTLFLAG_RD | CTLFLAG_MPSAFE, &r->devid, 0,
	    "PCI device ID of this PMC");

	SYSCTL_ADD_U64(ctx, parent, OID_AUTO,
	    "pwrm_base", CTLFLAG_RD | CTLFLAG_MPSAFE, &r->base, 0,
	    "Physical address of the PWRM register block");

	SYSCTL_ADD_PROC(ctx, parent, OID_AUTO,
	    "ltr_ignore",
	    CTLTYPE_U32 | CTLFLAG_RW | CTLFLAG_MPSAFE,
	    r, 0, intel_pmc_ltr_ignore_sysctl, "IU",
	    "LTR ignore mask: set bit = ignore IP, allowing S0ix");

	SYSCTL_ADD_PROC(ctx, parent, OID_AUTO,
	    "ltr",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    r, 0, intel_pmc_ltr_sysctl, "A",
	    "Per-IP latency tolerance requirements");

	if (r->gen->slp_s0_offset != 0) {
		SYSCTL_ADD_PROC(ctx, parent, OID_AUTO,
		    "slp_s0_residency",
		    CTLTYPE_U64 | CTLFLAG_RD | CTLFLAG_MPSAFE,
		    r, 0, intel_pmc_slp_s0_sysctl, "QU",
		    "SLP_S0 residency time in microseconds");

		/* Package scope, so once per device is enough. */
		SYSCTL_ADD_PROC(ctx, parent, OID_AUTO,
		    "pkgc_residency",
		    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
		    NULL, 0, intel_pmc_pkgc_sysctl, "A",
		    "Package C-state residency counters (raw MSR values)");
	}

	if (r->gen->lpm_maps != NULL) {
		SYSCTL_ADD_PROC(ctx, parent, OID_AUTO,
		    "lpm_status",
		    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
		    r, 0, intel_pmc_lpm_status_sysctl, "A",
		    "LPM readiness bits latched at the last latch event "
		    "(0 = IP was not ready)");

		SYSCTL_ADD_PROC(ctx, parent, OID_AUTO,
		    "lpm_live_status",
		    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
		    r, 1, intel_pmc_lpm_status_sysctl, "A",
		    "LPM readiness bits right now");

		SYSCTL_ADD_PROC(ctx, parent, OID_AUTO,
		    "lpm_modes",
		    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
		    r, 0, intel_pmc_lpm_modes_sysctl, "A",
		    "Enabled S0ix substates and their residency");

		SYSCTL_ADD_PROC(ctx, parent, OID_AUTO,
		    "lpm_latch_mode",
		    CTLTYPE_STRING | CTLFLAG_RW | CTLFLAG_MPSAFE,
		    r, 0, intel_pmc_lpm_latch_mode_sysctl, "A",
		    "What latches lpm_status: c10, a substate name, or clear");
	}

	if (r->gen->pfear_maps != NULL) {
		SYSCTL_ADD_PROC(ctx, parent, OID_AUTO,
		    "pfear",
		    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
		    r, 0, intel_pmc_pfear_sysctl, "A",
		    "PCH IP power gating status (Off = power gated)");
	}
}

/*
 * Find the SoC PWRM base and, if present, the IOE PMC, through the Shared
 * SRAM header in BAR0.  Returns true if the SoC base was found this way.
 * On success and if the generation has an IOE layout, sc->ioe is mapped.
 */
static bool
intel_pmc_ssram_discover(struct intel_pmc_softc *sc,
    const struct intel_pmc_gen *gen, vm_paddr_t *soc_base)
{
	device_t dev = sc->dev;
	bus_space_handle_t hdr;
	uint64_t ioe_ssram;
	vm_paddr_t base;
	uint16_t devid;

	sc->ssram_rid = PCIR_BAR(0);
	sc->ssram = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->ssram_rid,
	    RF_ACTIVE);
	if (sc->ssram == NULL) {
		device_printf(dev, "cannot map SSRAM BAR0\n");
		return (false);
	}

	if (!intel_pmc_ssram_hdr(rman_get_bustag(sc->ssram),
	    rman_get_bushandle(sc->ssram), &base, &devid)) {
		device_printf(dev, "SSRAM header unreadable\n");
		return (false);
	}
	if (devid != pci_get_device(dev)) {
		device_printf(dev, "SSRAM header devid 0x%04x does not match "
		    "device 0x%04x, ignoring\n", devid, pci_get_device(dev));
		return (false);
	}
	*soc_base = base;

	if (gen->ioe == NULL)
		return (true);

	/*
	 * The IOE PMC is a hidden PCI device.  Its SSRAM header address is
	 * published in ours, and its header has the same layout.
	 */
	ioe_ssram = bus_read_8(sc->ssram, SSRAM_IOE_OFFSET) &
	    SSRAM_BASE_ADDR_MASK;
	if (ioe_ssram == 0) {
		device_printf(dev, "no IOE PMC advertised in SSRAM\n");
		return (true);
	}

	hdr = (bus_space_handle_t)pmap_mapdev(ioe_ssram, SSRAM_HDR_SIZE);
	if (!intel_pmc_ssram_hdr(X86_BUS_SPACE_MEM, hdr, &base, &devid)) {
		device_printf(dev, "IOE SSRAM header at 0x%jx unreadable\n",
		    (uintmax_t)ioe_ssram);
		pmap_unmapdev((void *)hdr, SSRAM_HDR_SIZE);
		return (true);
	}
	pmap_unmapdev((void *)hdr, SSRAM_HDR_SIZE);

	if (devid != PMC_DEVID_MTL_IOEP && devid != PMC_DEVID_MTL_IOEM) {
		device_printf(dev, "IOE SSRAM header has unknown devid "
		    "0x%04x, not attaching IOE PMC\n", devid);
		return (true);
	}

	intel_pmc_map(dev, &sc->ioe, gen->ioe, base, devid);
	device_printf(dev, "IOE PMC devid 0x%04x, PWRM base 0x%jx\n",
	    devid, (uintmax_t)base);
	return (true);
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
	const struct pci_device_table *tbl;
	const struct intel_pmc_gen *gen;
	struct sysctl_oid *tree, *ioe_node;
	vm_paddr_t pwrm_base;
	const char *how;

	sc->dev = dev;
	tbl = PCI_MATCH(dev, intel_pmc_devices);
	gen = (const struct intel_pmc_gen *)tbl->driver_data;

	/*
	 * PWRM base discovery, in Linux's order of preference: SSRAM header
	 * (which is also the only way to find the IOE PMC), then the LPIT
	 * residency counter address, then the platform default.
	 */
	pwrm_base = 0;
	how = NULL;
	if (gen->use_ssram && intel_pmc_ssram_discover(sc, gen, &pwrm_base))
		how = "SSRAM";
	if (pwrm_base == 0 && gen->use_lpit) {
		pwrm_base = intel_pmc_lpit_pwrmbase(gen);
		if (pwrm_base != 0)
			how = "LPIT";
	}
	if (pwrm_base == 0 && gen->use_lpit) {
		pwrm_base = gen->pwrm_default;
		how = "default";
	}
	if (pwrm_base == 0 && !gen->use_lpit && !gen->use_ssram) {
		if ((pci_read_config(dev, SPT_PMC_ACTL_OFFSET, 4) &
		    SPT_PMC_ACTL_PWRM_EN) == 0) {
			device_printf(dev,
			    "PWRM decode not enabled by firmware\n");
			goto fail;
		}
		pwrm_base = pci_read_config(dev, gen->pwrm_cfg_offset, 4) &
		    0xfffff000;
		how = "PWRM_BASE";
		if (pwrm_base == 0) {
			device_printf(dev,
			    "PWRM base not configured by firmware\n");
			goto fail;
		}
	}

	intel_pmc_map(dev, &sc->soc, gen, pwrm_base, pci_get_device(dev));
	device_printf(dev, "%s PMC PWRM base 0x%jx (from %s)\n", gen->name,
	    (uintmax_t)pwrm_base, how);

	tree = device_get_sysctl_tree(dev);
	intel_pmc_add_sysctls(dev, SYSCTL_CHILDREN(tree), &sc->soc);

	if (sc->ioe.mapped) {
		ioe_node = SYSCTL_ADD_NODE(device_get_sysctl_ctx(dev),
		    SYSCTL_CHILDREN(tree), OID_AUTO, "ioe",
		    CTLFLAG_RD | CTLFLAG_MPSAFE, NULL,
		    "IO Expander die PMC");
		intel_pmc_add_sysctls(dev, SYSCTL_CHILDREN(ioe_node), &sc->ioe);
	}

	return (0);

fail:
	if (sc->ssram != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->ssram_rid,
		    sc->ssram);
	return (ENXIO);
}

static int
intel_pmc_detach(device_t dev)
{
	struct intel_pmc_softc *sc = device_get_softc(dev);

	intel_pmc_unmap(&sc->ioe);
	intel_pmc_unmap(&sc->soc);
	if (sc->ssram != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->ssram_rid,
		    sc->ssram);
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
MODULE_DEPEND(intel_pmc, acpi, 1, 1, 1);
