# x86 GPIO Interrupt Support for ACPI GpioInt Consumers

Date: 2026-09-30
Status: Draft for review
Target: FreeBSD 16-CURRENT, upstreamable via Phabricator in two commits

## Problem statement

I2C-HID input devices (touchpads, touchscreens) on modern Intel laptops
signal data availability through a GPIO pin. Their ACPI `_CRS` carries a
`GpioInt` descriptor (controller handle + pin + trigger mode) instead of an
`Interrupt` descriptor naming an IOAPIC IRQ.

FreeBSD/amd64 cannot deliver these interrupts today:

1. `intelgpio(4)` has no Meteor Lake support (only TGL-H and ADL-N tables).
2. `intelgpio(4)` is not an interrupt controller: it never allocates the
   controller's own IOAPIC IRQ and has no demux of per-pin status.
3. amd64 has no INTRNG, so `gpio_alloc_intr_resource()` is a stub returning
   NULL; there is no way to represent a GPIO pin as a `SYS_RES_IRQ`.
4. `acpi_iicbus(4)` explicitly skips `GpioInt` resources
   (`sys/dev/iicbus/acpi_iicbus.c` ~line 412, "Not supported by FreeBSD
   yet"), so children never receive an IRQ resource.

Consequence: `iichid(4)` falls back to `IICHID_SAMPLING` polled mode, whose
artifacts (rate decay to 10 Hz, load-induced poll jitter, synthetic
touch-release packets) make touchpads drop and stutter, badly under CPU
load. The `hmt(4)` source carries a comment that its zero-length-packet
workaround "is to be removed after GPIO interrupt support is added" — this
spec is that work.

## Goal and success criteria

Goal: a GpioInt-declared device on an Intel MTL laptop gets a first-class
interrupt through the standard newbus resource model, with **zero changes
to iichid(4)** or anything above it.

Success criteria:
- `iichid1` (Goodix touchpad) and `iichid0` (ELAN touchscreen) attach in
  interrupt mode (`sampling_rate_slow == -1`, no "Using sampling mode"
  message).
- The pin interrupt appears in `vmstat -i` and fires per touch event.
- Input latency remains smooth under `make -j24 buildworld` (the polled
  mode fails this today).
- Suspend/resume preserves interrupt operation.
- Every failure mode degrades to today's sampling mode, never to a broken
  device.

## Non-goals

- Porting INTRNG to x86.
- Interrupt support for bytgpio/chvgpio (the new interfaces permit adding
  it later).
- Deferred re-parse of GpioInt when the GPIO controller attaches after
  acpi_iicbus (v1 answer: load/compile intelgpio early; a bus rescan can
  be added later).
- Shareable GpioInt pins (`ACPI_SHARED`); acpi_gpiobus already marks this
  NOT_YET.
- CPU affinity for individual pins (they inherit the parent IRQ's CPU).

## Architecture overview

```
pad asserts -> GPIO controller asserts its own IOAPIC IRQ
  -> intelgpio filter reads GPI_IS per group, finds pin N
    -> intr_execute_handlers(intsrc for pin N)   [x86 intr_machdep]
      -> iichid_intr() ithread -> I2C read -> hidbus -> hmt -> evdev
```

The mechanism follows two existing x86 precedents: MSI (boot-reserved
dynamic IRQ number range) and Xen event channels (software-demuxed cascade
dispatching via `intr_execute_handlers()`).

Four design layers, delivered as two commits:

- Commit 1 (layer D2): MTL community/pad tables for intelgpio.
- Commit 2 (layers D1, D3, D4): GPIO interrupt support end to end —
  reserved GPIO IRQ range in `intr_machdep.c`, intelgpio as cascaded PIC
  + `GPIO_MAP_INTR` interface + non-INTRNG `gpio_alloc_intr_resource()`,
  and acpi_iicbus GpioInt -> IRQ resource mapping.

The D1/D3/D4 layer names below remain as design vocabulary; they describe
components of commit 2, not separate commits.

## D1: MD layer (sys/x86/x86/intr_machdep.c)

Folded into `intr_machdep.c` (no new file):

- `u_int first_gpio_irq`, `num_gpio_irqs` (loader tunable
  `machdep.num_gpio_irqs`, default 64, sysctl RD). Range reserved after the
  MSI range during interrupt init, before `intr_init_sources()` sizes
  `interrupt_sources[]`, with the same overflow guard `msi_init()` uses.
  Default is ON: 64 unused source-table slots cost pointer-array space
  only.
- `int gpio_irq_alloc(u_int *irq)` / `void gpio_irq_release(u_int irq)`:
  bitmap allocator over the range, protected by `icu_lock`. Release is
  required because GPIO drivers are unloadable (unlike MSI's bump
  allocator).
- No PIC here; PICs live in the GPIO drivers.

Failure mode: range exhausted or tunable set to 0 -> `gpio_irq_alloc()`
fails -> consumer gets no IRQ resource -> sampling fallback.

Estimated diff: ~120 lines.

## D2: MTL support in intelgpio (sys/dev/gpio/intel/)

New `mtlpgpio.c` sibling to `tglhgpio.c`/`adlngpio.c`:

- ACPI `_HID` match (expected `INTC1083` for MTL-P; confirm against the
  target DSDT via acpidump before coding).
- Community tables: per-community pad groups, names, pad counts, register
  offsets (PADBAR, GPI_IS, GPI_IE, HOSTSW_OWN). Derived from the Meteor
  Lake PCH datasheet with Linux `pinctrl-meteorlake.c` as cross-reference;
  tables are re-derived hardware facts, not copied GPL text.
- Verify PADCFG register layout version during implementation.

Independently useful: makes gpioc(4)/gpioctl(8) work on MTL with no
interrupt code at all.

## D3: intelgpio as a cascaded PIC + MI glue

### Parent interrupt and demux

`intelgpio_attach()` allocates the controller's own `SYS_RES_IRQ` (the
`Interrupt` descriptor in its `_CRS`) and installs a **filter**. The filter
iterates communities/groups reading `GPI_IS & GPI_IE`; for each pending
pin:

- Edge-configured pad: clear `GPI_IS` **before** dispatch (an edge during
  handling must not be lost).
- Level-configured pad: leave `GPI_IS`; it is cleared at EOI after the
  ithread completes (standard cascade semantics; `GPI_IE` masking keeps
  the parent line from screaming meanwhile via pic_disable_source).
- Pending pin with no registered intsrc: clear status, count as stray
  (device_printf rate-limited).

Dispatch is `intr_execute_handlers(isrc, frame)`, as in
`x86/xen/xen_arch_intr.c`.

### PIC implementation

One `struct pic` per controller softc, `intr_register_pic()` at attach.
Per-pin `struct intsrc` created lazily by `GPIO_MAP_INTR` (below); a
laptop uses a handful of interrupt pins out of hundreds.

Method mapping (pad community MMIO):

| pic method          | hardware action                                    |
|---------------------|----------------------------------------------------|
| pic_enable_source   | set GPI_IE bit                                     |
| pic_disable_source  | clear GPI_IE bit; also clear GPI_IS if PIC_EOI     |
| pic_eoi_source      | write-1-clear GPI_IS bit                           |
| pic_enable_intr     | program PADCFG0 RXEVCFG/RXINV from stored mode     |
| pic_disable_intr    | set RXEVCFG to disabled                            |
| pic_vector          | return stored dynamic IRQ number                   |
| pic_source_pending  | read GPI_IS bit                                    |
| pic_config_intr     | validate/translate trigger+polarity to RXEVCFG     |
| pic_suspend/resume  | save/restore GPI_IE and pad interrupt config       |
| pic_assign_cpu      | EOPNOTSUPP (inherits parent IRQ affinity)          |

### Locking

PIC methods run under `icu_lock` (spin). Add an `MTX_SPIN` mutex to the
softc covering GPI_IE/GPI_IS and PADCFG0 writes made from PIC methods and
the filter. Existing pad get/set paths keep their current lock but must
not touch the interrupt registers; where both paths touch PADCFG0, the
spin mutex covers the read-modify-write.

### Pad ownership guard

Before claiming a pin as an interrupt: verify HOSTSW_OWN gives the host
GPIO driver ownership and the pad is not routed to SCI/SMI/NMI. If the
check fails, refuse the mapping (consumer falls back to sampling) rather
than fight firmware.

### New MI interface (gpio_if.m)

```
int GPIO_MAP_INTR(device_t dev, uint32_t pin, uint32_t intr_mode,
                  u_int *irq);
int GPIO_UNMAP_INTR(device_t dev, u_int irq);
```

Default implementation returns EOPNOTSUPP so all existing GPIO drivers are
unaffected. intelgpio's implementation: ownership guard, gpio_irq_alloc(),
create intsrc, intr_register_source(), store mode, return IRQ. Unmap on
consumer detach releases the number (module unload of a consumer must not
leak the range).

### gpio_alloc_intr_resource() for non-INTRNG

Replace the NULL stub in `gpiobus.c` with: `GPIO_MAP_INTR()` on the pin's
controller, then `bus_alloc_resource()` of that IRQ, `GPIO_UNMAP_INTR()`
on failure. Mirrors the INTRNG variant with GPIO_MAP_INTR standing in for
intr_map_irq(). This gives gpio_aei(4) and gpiokeys(4) working interrupts
on amd64 as a side effect — worth stating in the upstream review.

## D4: acpi_iicbus GpioInt wiring (sys/dev/iicbus/acpi_iicbus.c)

In `acpi_iicbus_parse_resources_cb()`, replace the "Not supported" arm for
`ACPI_RESOURCE_GPIO_TYPE_INT`:

1. Resolve `ResourceSource.StringPtr` -> ACPI handle -> `acpi_get_device()`
   -> controller `device_t`. NULL (not attached) -> skip: no IRQ resource,
   sampling fallback.
2. `acpi_gpiobus_convflags()` (existing, exported in acpi_gpiobusvar.h) to
   get the `GPIO_INTR_*` mode.
3. `GPIO_MAP_INTR(controller, pin, mode, &irq)`; on success
   `resource_list_add(rl, SYS_RES_IRQ, 0, irq, irq, 1)` — the same call
   the ExtendedIrq arm makes today.
4. Track the mapping in the ivars so detach can `GPIO_UNMAP_INTR`.

iichid(4) is untouched: its existing `bus_alloc_resource_any(SYS_RES_IRQ)`
now succeeds, interrupt mode engages, `IICHID_SAMPLING` machinery stays
dormant as fallback.

Note: this consumes one `Interrupt`-or-`GpioInt` (first match wins, as
today's AE_CTRL_TERMINATE logic does); devices listing both prefer the
ExtendedIrq arm unchanged.

## Failure/fallback matrix

| Failure                                   | Result                     |
|-------------------------------------------|----------------------------|
| No MTL tables / intelgpio not loaded      | sampling mode (status quo) |
| machdep.num_gpio_irqs=0 or exhausted      | sampling mode              |
| GPIO_MAP_INTR unsupported by controller   | sampling mode              |
| Pad owned by firmware / routed to SCI     | sampling mode              |
| GpioInt parse error                       | sampling mode              |

No new failure mode is worse than current behavior.

## Testing plan

- D1: boot with tunable 0/64/4096; verify vmstat -i sizing, no MSI
  regressions (MSI-X NVMe/net devices still attach).
- D2: intelgpio attaches on MTL; gpioctl -l lists pads; pad names match
  DSDT usage.
- D3+D4 on-target (the MTL laptop, plus the ELAN touchscreen as a second
  device on a different community):
  - dmesg: no "Using sampling mode"; sysctl dev.iichid.%d.sampling_rate_slow
    reads -1.
  - vmstat -i: pin IRQ increments with touch activity; no storm at idle.
  - evemu-record latency subjectively and under make -j24 buildworld.
  - suspend/resume (S3 and s0ix if reachable): touch works after resume.
  - kldunload/kldload iichid repeatedly: no IRQ-number leak
    (machdep sysctl or leak counter), no panic.
  - Level and edge pads both exercised if the two devices differ.
- Regression: a machine (or VM) without GpioInt devices boots and behaves
  identically.

## Commit plan

- Commit 1 — `gpio: intelgpio: add Meteor Lake-P support`. The D2 tables
  only. Independently landable and useful on its own (gpioc/gpioctl pad
  access on MTL); no interrupt code, no MD changes.
- Commit 2 — `x86/gpio: GPIO interrupt support for ACPI GpioInt consumers
  (iichid)`. Layers D1 + D3 + D4 together: the intr_machdep IRQ range,
  intelgpio PIC/demux + GPIO_MAP_INTR + the non-INTRNG
  gpio_alloc_intr_resource(), and the acpi_iicbus GpioInt mapping.
  Reviewed as one Phabricator revision so the mechanism is judged with
  its first real consumer attached; removes wulf@'s long-standing
  "sampling mode" limitation for MTL laptops. Reviewers: x86 interrupt
  maintainers plus wulf@ for the iichid-facing behavior.

Follow-up (out of scope, enabled by this work): remove the hmt(4)
zero-length-packet workaround once sampling is rare; bytgpio/chvgpio
GPIO_MAP_INTR implementations.

## Open questions to resolve during implementation

1. Exact MTL-P `_HID` and community layout — confirm via acpidump on the
   target before D2.
2. Whether the touchpad pad is level- or edge-configured in the DSDT
   (determines which EOI path gets primary test coverage).
3. PADCFG register layout revision on MTL (Linux source cross-check).
4. Where interrupt-init ordering places the range reservation (tunable must
   be read before SI_SUB_INTR / intr_init_sources).
