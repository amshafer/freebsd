/*-
 * Copyright (c) 2026, Austin Shafer
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/pcpu.h>
#include <sys/power.h>
#include <sys/proc.h>
#include <sys/sched.h>
#include <sys/sbuf.h>
#include <sys/smp.h>
#include <sys/sysctl.h>

#include <dev/pci/pcivar.h>
#include <machine/atomic.h>
#include <machine/bus.h>
#include <machine/clock.h>
#include <machine/cpu.h>
#include <machine/cpufunc.h>
#include <machine/cputypes.h>
#include <machine/specialreg.h>
#include <machine/md_var.h>
#include <sys/rman.h>

#include <x86/x86_var.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <contrib/dev/acpica/include/accommon.h>
#include <dev/acpica/acpivar.h>

#include <x86/acpica_machdep.h>

enum intel_cstate {
    C0 = 0,
    C1,
    C1E,
    C2,
    C3,
    C4,
    C6,
    C6N,
    C6S,
    C6SP,
    C7,
    C7S,
    C8,
    C9,
    C10,
};

struct intel_cx {
    char		 name[4];	/* State name string (e.g., "C1", "C6") */
    enum intel_cstate	 type;		/* the state name (C1, C6, etc) */
    uint32_t		 latency;	/* Exit latency (usec) */
    uint32_t		 power;		/* Power consumed (mW). */
    uint32_t		 enter_time;	/* Time in this state to make entering worth it (usec) */
    uint32_t		 mwait_hint;    /* argument to pass to the mwait instruction */
    int			 req_ibrs;	/* this state requires disabling IBRS to enter */
};

#define MAX_CX_STATES 16

struct intel_idle_softc {
    device_t		 cpu_dev;
    ACPI_HANDLE		 cpu_handle;
    struct pcpu		*cpu_pcpu;
    struct intel_cx	*cpu_cx_states;
    int			 cpu_prev_sleep;	/* Last idle sleep duration. */
    u_int		 cpu_cx_stats[MAX_CX_STATES];/* Cx usage history. */
    uint64_t		 cpu_cx_duration[MAX_CX_STATES];/* Cx cumulative sleep */
    int			 cpu_disable_idle; 	/* Disable entry to idle function */
    /* Values for sysctl. */
    struct sysctl_ctx_list cpu_sysctl_ctx;
    struct sysctl_oid	*cpu_sysctl_tree;
    char 		 cpu_cx_supported[64];
};

/*
 * C-state tables for various Intel CPU generations.
 *
 * Table format: { type, latency, power, enter_time, mwait_hint, req_ibrs }
 *   type       - C-state type (C1, C1E, C3, C6, C7, C8, C9, C10)
 *   latency    - Exit latency in microseconds
 *   power      - Power consumption in milliwatts (estimated)
 *   enter_time - Target residency: minimum time worth entering state (usec)
 *   mwait_hint - MWAIT hint value for this state
 *   req_ibrs   - If 1, requires disabling IBRS mitigations to enter this state
 */

/* Nehalem (Core i7/i5/i3 1st gen) - Model 0x1A, 0x1E, 0x1F, 0x2E */
static struct intel_cx nehalem_cstates[] = {
	/* name type latency power enter_time hint ibrs */
	{ "C1",  C1,  3,   1000,  6,    0x00, 0 },
	{ "C1E", C1E, 10,  500,   20,   0x01, 0 },
	{ "C3",  C3,  20,  350,   80,   0x10, 0 },
	{ "C6",  C6,  200, 15,    800,  0x20, 0 },
	{ "",    C0,  0,   0,     0,    0,    0 },  /* Terminator */
};

/* Sandy Bridge / Ivy Bridge (2nd/3rd gen Core) - Model 0x2A, 0x2D, 0x3A, 0x3E */
static struct intel_cx snb_cstates[] = {
	/* name type latency power enter_time hint ibrs */
	{ "C1",  C1,  2,   1000,  2,    0x00, 0 },
	{ "C1E", C1E, 10,  500,   20,   0x01, 0 },
	{ "C3",  C3,  80,  200,   211,  0x10, 0 },
	{ "C6",  C6,  104, 15,    345,  0x20, 0 },
	{ "C7",  C7,  109, 10,    345,  0x30, 0 },
	{ "",    C0,  0,   0,     0,    0,    0 },  /* Terminator */
};

/* Haswell (4th gen Core) - Model 0x3C, 0x3F, 0x45, 0x46 */
static struct intel_cx hsw_cstates[] = {
	/* name type latency power enter_time hint ibrs */
	{ "C1",  C1,  2,    1000,  2,    0x00, 0 },  /* C1 */
	{ "C1E", C1E, 10,   500,   20,   0x01, 0 },  /* C1E */
	{ "C3",  C3,  33,   200,   100,  0x10, 0 },  /* C3 */
	{ "C6",  C6,  133,  15,    400,  0x20, 1 },  /* C6 - requires IBRS disable */
	{ "C7S", C7S, 166,  10,    500,  0x32, 1 },  /* C7s */
	{ "C8",  C8,  300,  5,     900,  0x40, 1 },  /* C8 */
	{ "C9",  C9,  600,  2,     1800, 0x50, 1 },  /* C9 */
	{ "C10", C10, 2600, 1,     7700, 0x60, 1 },  /* C10 */
	{ "",    C0,  0,    0,     0,    0,    0 },  /* Terminator */
};

/* Broadwell (5th gen Core) - Model 0x3D, 0x47, 0x4F, 0x56 */
static struct intel_cx bdw_cstates[] = {
	/* name type latency power enter_time hint ibrs */
	{ "C1",  C1,  2,    1000,  2,    0x00, 0 },  /* C1 */
	{ "C1E", C1E, 10,   500,   20,   0x01, 0 },  /* C1E */
	{ "C3",  C3,  40,   200,   100,  0x10, 0 },  /* C3 */
	{ "C6",  C6,  133,  15,    400,  0x20, 1 },  /* C6 */
	{ "C7S", C7S, 166,  10,    500,  0x32, 1 },  /* C7s */
	{ "C8",  C8,  300,  5,     900,  0x40, 1 },  /* C8 */
	{ "C9",  C9,  600,  2,     1800, 0x50, 1 },  /* C9 */
	{ "C10", C10, 2600, 1,     7700, 0x60, 1 },  /* C10 */
	{ "",    C0,  0,    0,     0,    0,    0 },  /* Terminator */
};

/* Skylake / Kaby Lake / Coffee Lake (6th-9th gen Core) - Model 0x4E, 0x5E, 0x8E, 0x9E */
static struct intel_cx skl_cstates[] = {
	/* name type latency power enter_time hint ibrs */
	{ "C1",  C1,  2,   1000,  2,    0x00, 0 },  /* C1 */
	{ "C1E", C1E, 10,  500,   20,   0x01, 0 },  /* C1E */
	{ "C3",  C3,  70,  200,   100,  0x10, 0 },  /* C3 */
	{ "C6",  C6,  85,  15,    200,  0x20, 1 },  /* C6 - requires IBRS disable */
	{ "C7S", C7S, 124, 10,    800,  0x33, 1 },  /* C7s */
	{ "C8",  C8,  200, 5,     800,  0x40, 1 },  /* C8 */
	{ "C9",  C9,  480, 2,     5000, 0x50, 1 },  /* C9 */
	{ "C10", C10, 890, 1,     5000, 0x60, 1 },  /* C10 */
	{ "",    C0,  0,   0,     0,    0,    0 },  /* Terminator */
};

/* Alder Lake P-core (12th gen+) - Model 0x97, 0x9A */
static struct intel_cx adl_cstates[] = {
	/* name type latency power enter_time hint ibrs */
	{ "C1",  C1,  1,   1000,  1,    0x00, 0 },  /* C1 */
	{ "C1E", C1E, 2,   500,   4,    0x01, 0 },  /* C1E */
	{ "C6",  C6,  220, 15,    600,  0x20, 0 },  /* C6 */
	{ "C8",  C8,  280, 5,     800,  0x40, 0 },  /* C8 */
	{ "C10", C10, 680, 1,     2000, 0x60, 0 },  /* C10 */
	{ "",    C0,  0,   0,     0,    0,    0 },  /* Terminator */
};

/* Alder Lake E-core (12th gen+ Gracemont) - Model 0x97, 0x9A (E-cores) */
static struct intel_cx adl_l_cstates[] = {
	/* name type latency power enter_time hint ibrs */
	{ "C1",  C1,  1,   1000,  1,    0x00, 0 },  /* C1 */
	{ "C1E", C1E, 2,   500,   4,    0x01, 0 },  /* C1E */
	{ "C6",  C6,  170, 15,    500,  0x20, 0 },  /* C6 */
	{ "C8",  C8,  200, 5,     600,  0x40, 0 },  /* C8 */
	{ "C10", C10, 480, 1,     1500, 0x60, 0 },  /* C10 */
	{ "",    C0,  0,   0,     0,    0,    0 },  /* Terminator */
};

/* Ivy Bridge (3rd gen Core) - Model 0x3A */
static struct intel_cx ivb_cstates[] = {
	/* name type latency power enter_time hint ibrs */
	{ "C1",  C1,  1,   1000,  1,    0x00, 0 },  /* C1 */
	{ "C1E", C1E, 10,  500,   20,   0x01, 0 },  /* C1E */
	{ "C3",  C3,  59,  200,   156,  0x10, 0 },  /* C3 */
	{ "C6",  C6,  80,  15,    300,  0x20, 0 },  /* C6 */
	{ "C7",  C7,  87,  10,    300,  0x30, 0 },  /* C7 */
	{ "",    C0,  0,   0,     0,    0,    0 },  /* Terminator */
};

/* Skylake-X / Cascade Lake (Server) - Model 0x55 */
static struct intel_cx skx_cstates[] = {
	/* name type latency power enter_time hint ibrs */
	{ "C1",  C1,  2,   1000,  2,    0x00, 0 },  /* C1 */
	{ "C1E", C1E, 10,  500,   20,   0x01, 0 },  /* C1E */
	{ "C6",  C6,  133, 15,    600,  0x20, 1 },  /* C6 - requires IBRS disable */
	{ "",    C0,  0,   0,     0,    0,    0 },  /* Terminator */
};

/* Ice Lake-X / Sunny Cove (Server) - Model 0x6A, 0x6C */
static struct intel_cx icx_cstates[] = {
	/* name type latency power enter_time hint ibrs */
	{ "C1",  C1,  1,   1000,  1,    0x00, 0 },  /* C1 */
	{ "C1E", C1E, 4,   500,   4,    0x01, 0 },  /* C1E */
	{ "C6",  C6,  170, 15,    600,  0x20, 0 },  /* C6 */
	{ "",    C0,  0,   0,     0,    0,    0 },  /* Terminator */
};

/* Sapphire Rapids / Emerald Rapids (Server) - Model 0x8F, 0xCF */
static struct intel_cx spr_cstates[] = {
	/* name type latency power enter_time hint ibrs */
	{ "C1",  C1,  1,   1000,  1,    0x00, 0 },  /* C1 */
	{ "C1E", C1E, 2,   500,   4,    0x01, 0 },  /* C1E */
	{ "C6",  C6,  290, 15,    800,  0x20, 0 },  /* C6 */
	{ "",    C0,  0,   0,     0,    0,    0 },  /* Terminator */
};

/* Meteor Lake (14th gen) - Model 0xAA */
static struct intel_cx mtl_l_cstates[] = {
	/* name type latency power enter_time hint ibrs */
	{ "C1E", C1E, 1,   500,   1,    0x01, 0 },  /* C1E */
	{ "C6",  C6,  140, 15,    420,  0x20, 0 },  /* C6 */
	{ "C10", C10, 310, 1,     930,  0x60, 0 },  /* C10 */
	{ "",    C0,  0,   0,     0,    0,    0 },  /* Terminator */
};

/* Bay Trail / Valleyview (Silvermont Atom) - Model 0x37 */
static struct intel_cx byt_cstates[] = {
	/* name type latency power enter_time hint ibrs */
	{ "C1",  C1,  1,    1000,  1,     0x00, 0 },  /* C1 */
	{ "C6N", C6N, 300,  50,    275,   0x58, 0 },  /* C6N */
	{ "C6S", C6S, 500,  30,    560,   0x52, 0 },  /* C6S */
	{ "C7",  C7,  1200, 10,    4000,  0x60, 0 },  /* C7 */
	{ "C7S", C7S, 10000,5,     20000, 0x64, 0 },  /* C7S */
	{ "",    C0,  0,    0,     0,     0,    0 },  /* Terminator */
};

/* Cherry Trail / Braswell (Airmont Atom) - Model 0x4C */
static struct intel_cx cht_cstates[] = {
	/* name type latency power enter_time hint ibrs */
	{ "C1",  C1,  1,    1000,  1,     0x00, 0 },  /* C1 */
	{ "C6N", C6N, 80,   50,    275,   0x58, 0 },  /* C6N */
	{ "C6S", C6S, 200,  30,    560,   0x52, 0 },  /* C6S */
	{ "C7",  C7,  1200, 10,    4000,  0x60, 0 },  /* C7 */
	{ "C7S", C7S, 10000,5,     20000, 0x64, 0 },  /* C7S */
	{ "",    C0,  0,    0,     0,     0,    0 },  /* Terminator */
};

/* Apollo Lake / Gemini Lake (Goldmont Atom) - Model 0x5C, 0x7A */
static struct intel_cx bxt_cstates[] = {
	/* name type latency power enter_time hint ibrs */
	{ "C1",  C1,  2,    1000,  2,    0x00, 0 },  /* C1 */
	{ "C1E", C1E, 10,   500,   20,   0x01, 0 },  /* C1E */
	{ "C6",  C6,  133,  15,    133,  0x20, 0 },  /* C6 */
	{ "C7S", C7S, 155,  10,    155,  0x31, 0 },  /* C7s */
	{ "C8",  C8,  1000, 5,     1000, 0x40, 0 },  /* C8 */
	{ "C9",  C9,  2000, 2,     2000, 0x50, 0 },  /* C9 */
	{ "C10", C10, 10000,1,     10000,0x60, 0 },  /* C10 */
	{ "",    C0,  0,    0,     0,    0,    0 },  /* Terminator */
};

/* Per-CPU softc array for quick lookup in idle hook */
static struct intel_idle_softc **cpu_softc;

/* External reference to cpu_idle_fn from cpu_machdep.c */
extern void (*cpu_idle_fn)(sbintime_t);

/* Forward declarations */
static void	intel_idle_identify(driver_t *driver, device_t parent);
static int	intel_idle_probe(device_t dev);
static int	intel_idle_attach(device_t dev);
static int	intel_idle_detach(device_t dev);

static void	intel_idle_hook(sbintime_t sbt);
static struct intel_cx *intel_idle_select_cstates(void);
static int	intel_idle_cx_usage_sysctl(SYSCTL_HANDLER_ARGS);

/*
 * Select the appropriate C-state table for the current CPU model.
 * Returns NULL if the CPU model is not supported.
 */
static struct intel_cx *
intel_idle_select_cstates(void)
{
	u_int cpu_model;

	cpu_model = CPUID_TO_MODEL(cpu_id);

	switch (cpu_model) {
	/* Nehalem (1st gen Core) */
	case 0x1A:
	case 0x1E:
	case 0x1F:
	case 0x2E:
		return (nehalem_cstates);
	/* Sandy Bridge (2nd gen Core) */
	case 0x2A:
	case 0x2D:
		return (snb_cstates);
	/* Ivy Bridge (3rd gen Core) */
	case 0x3A:
		return (ivb_cstates);
	/* Ivy Bridge-X (Server) */
	case 0x3E:
		return (snb_cstates);  /* Use SNB table for IVB-X */
	/* Haswell (4th gen Core) */
	case 0x3C:
	case 0x3F:
	case 0x45:
	case 0x46:
		return (hsw_cstates);
	/* Broadwell (5th gen Core) */
	case 0x3D:
	case 0x47:
	case 0x4F:
	case 0x56:
		return (bdw_cstates);
	/* Skylake / Kaby Lake / Coffee Lake (6th-9th gen Core) */
	case 0x4E:
	case 0x5E:
	case 0x8E:
	case 0x9E:
		return (skl_cstates);
	/* Skylake-X / Cascade Lake (Server) */
	case 0x55:
		return (skx_cstates);
	/* Ice Lake-X (Server) */
	case 0x6A:
	case 0x6C:
		return (icx_cstates);
	/* Sapphire Rapids / Emerald Rapids (Server) */
	case 0x8F:
	case 0xCF:
		return (spr_cstates);
	/* Alder Lake (12th gen) */
	case 0x97:
		return (adl_cstates);
	/* Alder Lake-L (12th gen) */
	case 0x9A:
		return (adl_l_cstates);
	/* Meteor Lake (14th gen) */
	case 0xAA:
		return (mtl_l_cstates);
	/* Bay Trail / Valleyview (Silvermont Atom) */
	case 0x37:
		return (byt_cstates);
	/* Cherry Trail / Braswell (Airmont Atom) */
	case 0x4C:
		return (cht_cstates);
	/* Apollo Lake (Goldmont Atom) */
	case 0x5C:
		return (bxt_cstates);
	/* Gemini Lake (Goldmont Plus Atom) */
	case 0x7A:
		return (bxt_cstates);
	default:
		return (NULL);
	}
}

static int
is_idle_disabled(struct intel_idle_softc *sc)
{
	return (sc->cpu_disable_idle);
}

static void
enable_idle(struct intel_idle_softc *sc)
{
	sc->cpu_disable_idle = FALSE;
}

static void
disable_idle(struct intel_idle_softc *sc)
{
	cpuset_t cpuset;

	CPU_SETOF(sc->cpu_pcpu->pc_cpuid, &cpuset);
	sc->cpu_disable_idle = TRUE;

	/*
	 * Ensure that the CPU is not in idle state or in acpi_cpu_idle().
	 * Note that this code depends on the fact that the rendezvous IPI
	 * can not penetrate context where interrupts are disabled and acpi_cpu_idle
	 * is called and executed in such a context with interrupts being re-enabled
	 * right before return.
	 */
	smp_rendezvous_cpus(cpuset, smp_no_rendezvous_barrier, NULL,
		smp_no_rendezvous_barrier, NULL);
}

/* Device methods */
static device_method_t intel_idle_methods[] = {
	/* Device interface */
	DEVMETHOD(device_identify,	intel_idle_identify),
	DEVMETHOD(device_probe,		intel_idle_probe),
	DEVMETHOD(device_attach,	intel_idle_attach),
	DEVMETHOD(device_detach,	intel_idle_detach),
	DEVMETHOD_END
};

static driver_t intel_idle_driver = {
	"intel_idle",
	intel_idle_methods,
	sizeof(struct intel_idle_softc),
};

/* create an intel_idle dev for each cpu dev in the system */
DRIVER_MODULE(intel_idle, cpu, intel_idle_driver, 0, 0);
MODULE_VERSION(intel_idle, 1);

/*
 * Identify method - called by the bus to see if we want to create
 * a child device for this parent CPU device.
 */
static void
intel_idle_identify(driver_t *driver, device_t parent)
{

	/* Check if already attached to this CPU */
	if (device_find_child(parent, "intel_idle", DEVICE_UNIT_ANY) != NULL)
		return;

	/* Only attach to Intel CPUs */
	if (cpu_vendor_id != CPU_VENDOR_INTEL)
		return;

	/* Require MONITOR/MWAIT support */
	if ((cpu_feature2 & CPUID2_MON) == 0)
		return;

	/* Check MWAIT extensions and interrupt break */
	if (!cpu_mwait_usable())
		return;

	/* Check if we have a C-state table for this CPU model */
	if (intel_idle_select_cstates() == NULL) {
		if (bootverbose)
			device_printf(parent,
			    "intel_idle: No C-state table for CPU model 0x%x\n",
			    CPUID_TO_MODEL(cpu_id));
		return;
	}

	if (BUS_ADD_CHILD(parent, 10, "intel_idle", device_get_unit(parent))
	    == NULL)
		device_printf(parent, "intel_idle: add child failed\n");
}

static int
intel_idle_probe(device_t dev)
{
	device_set_desc(dev, "Intel Idle Driver");
	return (BUS_PROBE_NOWILDCARD);
}

/*
 * Attach to a CPU and initialize C-states.
 */
static int
intel_idle_attach(device_t dev)
{
	struct intel_idle_softc *sc;
	struct pcpu *pc;
	int cpu_id;

	sc = device_get_softc(dev);
	/* register our per-cpu info so we can look it up in intel_idle */
	sc->cpu_dev = dev;
	sc->cpu_handle = acpi_get_handle(dev);

	/* Allocate the per-CPU softc array on first attach */
	cpu_id = device_get_unit(dev);
	if (cpu_softc == NULL)
		cpu_softc = malloc(sizeof(struct intel_idle_softc *) *
		    (mp_maxid + 1), M_DEVBUF, M_WAITOK | M_ZERO);

	/* Store this softc in the array for quick lookup */
	cpu_softc[cpu_id] = sc;

	/* Get the pcpu structure for this CPU */
	pc = pcpu_find(cpu_id);
	if (pc == NULL) {
		device_printf(dev, "pcpu_find(%d) failed\n", cpu_id);
		return (ENXIO);
	}
	pc->pc_device = dev;
	sc->cpu_pcpu = pc;

	/* Select and assign the appropriate C-state table for this CPU model */
	sc->cpu_cx_states = intel_idle_select_cstates();
	if (sc->cpu_cx_states == NULL) {
		device_printf(dev, "No C-state table for CPU model 0x%x\n",
		    CPUID_TO_MODEL(cpu_id));
		return (ENXIO);
	}
	device_printf(dev, "Found C-state table %p for CPU model 0x%x\n",
		sc->cpu_cx_states, CPUID_TO_MODEL(cpu_id));

	/* Add sysctl to display C-state usage */
	sysctl_ctx_init(&sc->cpu_sysctl_ctx);
	sc->cpu_sysctl_tree = SYSCTL_ADD_NODE(&sc->cpu_sysctl_ctx,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "cx", CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, "C-state statistics");
	if (sc->cpu_sysctl_tree != NULL) {
		SYSCTL_ADD_PROC(&sc->cpu_sysctl_ctx,
		    SYSCTL_CHILDREN(sc->cpu_sysctl_tree), OID_AUTO,
		    "usage", CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE,
		    (void *)sc, 0, intel_idle_cx_usage_sysctl, "A",
		    "C-state usage percentage");
	}

	/* Set intel_idle as the active hook function */
	cpu_idle_hook = intel_idle_hook;
	if (bootverbose)
		device_printf(dev, "intel_idle enabled\n");

	return (0);
}

/*
 * Detach from a CPU.
 */
static int
intel_idle_detach(device_t dev)
{
	struct intel_idle_softc *sc;
	char idle_name[] = "acpi";
	int cpu_id, error;

	sc = device_get_softc(dev);

	/* Disable idle entry before detaching */
	disable_idle(sc);

	/* Restore idle function to ACPI using machdep.idle sysctl */
	error = kernel_sysctlbyname(curthread, "machdep.idle",
	    NULL, NULL, idle_name, sizeof(idle_name), NULL, 0);
	if (error != 0 && bootverbose)
		device_printf(dev, "Failed to restore idle to acpi: %d\n", error);
	else if (bootverbose)
		device_printf(dev, "Restored idle to acpi\n");

	/* Clean up sysctl context */
	sysctl_ctx_free(&sc->cpu_sysctl_ctx);

	/* Clear the per-CPU softc entry */
	cpu_id = device_get_unit(dev);
	if (cpu_softc != NULL && cpu_id <= mp_maxid)
		cpu_softc[cpu_id] = NULL;

	if (sc->cpu_pcpu != NULL)
		sc->cpu_pcpu->pc_device = NULL;

	return (0);
}

/*
 * Sysctl handler to display C-state usage statistics.
 */
static int
intel_idle_cx_usage_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct intel_idle_softc *sc = (struct intel_idle_softc *)arg1;
	struct sbuf sb;
	char buf[256];
	int error, i, cx_count;
	uintmax_t fract, sum, whole;

	sbuf_new_for_sysctl(&sb, buf, sizeof(buf), req);

	/* Count how many C-states we have (until C0 terminator) */
	cx_count = 0;
	for (i = 0; sc->cpu_cx_states[i].type != C0; i++)
		cx_count++;

	/* Calculate total usage */
	sum = 0;
	for (i = 0; i < cx_count; i++)
		sum += sc->cpu_cx_stats[i];

	/* Print each C-state name and percentage */
	for (i = 0; i < cx_count; i++) {
		if (i > 0)
			sbuf_putc(&sb, ' ');
		sbuf_printf(&sb, "%s:", sc->cpu_cx_states[i].name);
		if (sum > 0) {
			whole = (uintmax_t)sc->cpu_cx_stats[i] * 100;
			fract = (whole % sum) * 100;
			sbuf_printf(&sb, "%u.%02u%%", (u_int)(whole / sum),
			    (u_int)(fract / sum));
		} else {
			sbuf_printf(&sb, "0.00%%");
		}
	}

	error = sbuf_finish(&sb);
	sbuf_delete(&sb);
	return (error);
}

/*
 * Main idle function - cpu_idle_fn will point to this.
 */
static void
intel_idle_hook(sbintime_t sbt)
{
	struct intel_idle_softc *sc;
	struct intel_cx *cx, *cx_next;
	int cpuid, i;

	/* Get the softc for this CPU */
	cpuid = PCPU_GET(cpuid);
	sc = cpu_softc[cpuid];
	if (sc == NULL || is_idle_disabled(sc))
		return;

	/*
	 * Find the deepest C-state (lowest power).
	 * The C-state tables are ordered with the deepest state last,
	 * terminated by C0. Walk through to find the last non-C0 state.
	 */
	cx = sc->cpu_cx_states;
	cx_next = &cx[0];  /* Default to first state (C1) */

	for (i = 0; cx[i].type != C0; i++) {
		cx_next = &cx[i];
	}

	/* Update statistics */
	sc->cpu_cx_stats[i - 1]++;

	/* Enter the selected C-state using MWAIT */
	acpi_cpu_idle_mwait(cx_next->mwait_hint);
}
