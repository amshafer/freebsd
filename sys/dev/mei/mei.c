/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Austin Shafer <ashafer@badland.io>
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

/*
 * Intel Management Engine Interface (MEI) driver.
 *
 * This is a minimal MEI driver that establishes a connection to the GSC
 * (Graphics Security Controller) proxy client on the Intel CSME (Converged
 * Security and Manageability Engine) firmware.  This enables the i915 GPU
 * driver (via drm-kmod) to proxy GSC authentication messages through the
 * PCH MEI PCI device, which is required for GSC and HuC firmware loading
 * on Intel Meteor Lake and newer platforms.
 *
 * The MEI hardware is a simple MMIO-based interface at PCI BAR0 consisting
 * of two circular buffers (host-to-ME and ME-to-host) accessed through
 * write/read window registers, plus control/status registers for flow
 * control and interrupt signaling.
 *
 * On top of the hardware layer, the Host Bus Message (HBM) protocol handles
 * version negotiation, client enumeration, connection management, and
 * per-client flow control.  This driver implements just enough of HBM to
 * find and connect to the GSC proxy client.
 *
 * References:
 *   - Intel DCMI-HI 1.0 Specification (HECI host interface):
 *     https://www.intel.com/content/dam/www/public/us/en/documents/
 *     technical-specifications/dcmi-hi-10-spec.pdf
 *   - Intel Management Engine manageability docs:
 *     https://software.intel.com/en-us/manageability/
 *   - Intel PCH datasheets, "ME Host Interface" chapter
 *     (register layouts for H_CSR, ME_CSR_HA, H_CB_WW, ME_CB_RW)
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/condvar.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <sys/rman.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <dev/mei/mei.h>
#include <dev/mei/mei_reg.h>

/* Maximum RX buffer size - GSC proxy messages are at most 32K */
#define MEI_RX_BUF_SIZE		(32 * 1024)

/* Maximum number of ME clients to enumerate */
#define MEI_MAX_ME_CLIENTS	256

static const uint8_t mei_uuid_gsc_proxy[16] = MEI_UUID_GSC_PROXY;

/*
 * HBM protocol state machine.
 *
 * The Host Bus Message protocol is a handshake between the host driver
 * and ME firmware that runs over the MEI circular buffers using
 * me_addr=0, host_addr=0.  The init sequence is:
 *
 *   IDLE ──► STARTING ──► ENUM ──► CLIENT_PROPERTIES ──► CONNECTING ──► STARTED
 *         start_req    enum_req    props for each       connect to
 *         /start_res   /enum_res   valid ME client      GSC proxy
 *
 * Ref: DCMI-HI 1.0 spec, Section 7.1 "Host Bus Message Protocol"
 */
enum mei_hbm_state {
	MEI_HBM_IDLE,			/* no HBM exchange yet */
	MEI_HBM_STARTING,		/* HOST_START_RES received */
	MEI_HBM_ENUM,			/* HOST_ENUM_RES received */
	MEI_HBM_CLIENT_PROPERTIES,	/* querying client properties */
	MEI_HBM_CONNECTING,		/* CLIENT_CONNECT_REQ sent */
	MEI_HBM_STARTED,		/* GSC proxy connected, ready */
};

/*
 * Per-device softc for the MEI PCI driver.
 *
 * The MEI hardware presents four MMIO registers at BAR0 (see mei_reg.h):
 *   H_CB_WW  (0x00) - write window into the host-to-ME circular buffer
 *   H_CSR    (0x04) - host control/status (interrupt enable, ready, reset,
 *                      circular buffer read/write pointers and depth)
 *   ME_CB_RW (0x08) - read window from the ME-to-host circular buffer
 *   ME_CSR_HA(0x0C) - ME control/status (read-only from host side;
 *                      ME ready, ME buffer pointers)
 *
 * Communication with ME firmware uses DWORD-granularity circular buffers.
 * Each message is preceded by a 4-byte header (mei_msg_hdr) that contains
 * source/destination addresses, payload length, and a msg_complete flag.
 * The host writes header+payload DWORDs to H_CB_WW, then sets the H_IG
 * (interrupt generate) bit in H_CSR to notify the ME.  The ME writes
 * responses to its buffer and asserts an interrupt; the host reads them
 * from ME_CB_RW.
 *
 * Ref: DCMI-HI 1.0 spec, Section 4 "HECI Hardware Interface"
 * Ref: Intel PCH datasheet, "ME Host Interface Registers"
 */
struct mei_softc {
	device_t		dev;

	/* PCI BAR0 MMIO mapping for the four MEI registers */
	struct resource		*mem_res;
	int			mem_rid;
	struct resource		*irq_res;
	int			irq_rid;
	void			*irq_handle;
	bus_space_tag_t		bst;	/* bus space tag for MMIO access */
	bus_space_handle_t	bsh;	/* bus space handle for MMIO access */

	/*
	 * Single mutex protects all softc state and serializes HW access.
	 * The condvars are used for synchronous waiting during:
	 *   hbm_cv - HBM protocol steps (init sequence, connect)
	 *   tx_cv  - waiting for ME to grant a TX flow control credit
	 *   rx_cv  - waiting for ME to deliver client data
	 */
	struct mtx		mtx;
	struct cv		hbm_cv;
	struct cv		tx_cv;
	struct cv		rx_cv;

	/*
	 * Host circular buffer depth in DWORD slots, read from H_CSR[31:24]
	 * during attach.  This is the maximum number of DWORDs that can be
	 * written to H_CB_WW before the buffer is full.
	 * Ref: DCMI-HI 1.0 spec, Section 4.1.1 "H_CSR"
	 */
	int			hbuf_depth;

	/*
	 * HBM protocol state, driven by the init sequence in mei_init_hw().
	 * hbm_ver_{major,minor} are negotiated with ME during HOST_START.
	 * hw_ready is set when ME asserts ME_RDY in ME_CSR_HA after reset.
	 */
	enum mei_hbm_state	hbm_state;
	uint8_t			hbm_ver_major;
	uint8_t			hbm_ver_minor;
	bool			hw_ready;

	/*
	 * Client enumeration state.
	 * me_clients_map is a 256-bit bitmap from HOST_ENUM_RES indicating
	 * which ME client addresses are valid.  For each valid address, we
	 * send HOST_CLIENT_PROPERTIES_REQ to get the client's UUID and MTU.
	 * prop_result/prop_received are used to pass the async response
	 * from the interrupt handler back to the synchronous init thread.
	 */
	uint8_t			me_clients_map[32];
	int			prop_query_addr;
	struct mei_client_properties prop_result;
	bool			prop_received;

	/*
	 * GSC proxy client connection state.
	 * After enumeration finds the client matching MEI_UUID_GSC_PROXY,
	 * we record its ME address and send CLIENT_CONNECT_REQ.
	 * gsc_host_addr is our locally-assigned address (always 1).
	 * gsc_mtu is the maximum message size the ME client supports.
	 */
	int			gsc_me_addr;	/* ME-side client address */
	int			gsc_host_addr;	/* host-side address (1) */
	uint32_t		gsc_mtu;	/* max msg length in bytes */
	bool			gsc_connected;
	int			gsc_connect_status;  /* from CONNECT_RES */
	bool			gsc_connect_received;

	/*
	 * Per-client TX flow control.
	 * The ME grants credits via HBM FLOW_CONTROL messages (cmd 0x08).
	 * Each mei_gsc_proxy_send() consumes one credit; if none are
	 * available, the caller blocks on tx_cv until the interrupt handler
	 * receives a new credit grant from ME.
	 * Ref: DCMI-HI 1.0 spec, Section 7.2.5 "Flow Control"
	 */
	int			tx_credits;

	/*
	 * Single RX buffer for incoming GSC proxy client data.
	 * The interrupt handler copies payload from ME_CB_RW into rx_buf
	 * and sets rx_ready=true, waking any thread blocked in
	 * mei_gsc_proxy_recv().  Only one outstanding RX is supported
	 * (matching the synchronous GSC proxy protocol).
	 *
	 * A client message larger than the hardware limit (511 bytes per
	 * MEI message, 9-bit length field) arrives as multiple fragments;
	 * only the final one has MEI_HDR_MSG_COMPLETE set.  rx_off tracks
	 * the reassembly position of the partially-received message.
	 */
	uint8_t			*rx_buf;
	size_t			rx_len;
	size_t			rx_off;
	bool			rx_ready;
};

static int	mei_pci_detach(device_t dev);

/* PCI device table */
static const struct mei_pci_id {
	uint16_t	vendor;
	uint16_t	device;
	const char	*desc;
} mei_pci_ids[] = {
	{ PCI_VENDOR_INTEL, MEI_DEV_ID_MTL_M, "Meteor Lake MEI" },
	{ PCI_VENDOR_INTEL, MEI_DEV_ID_ARL_S, "Arrow Lake MEI" },
	{ PCI_VENDOR_INTEL, MEI_DEV_ID_ARL_H, "Arrow Lake MEI" },
	{ PCI_VENDOR_INTEL, MEI_DEV_ID_LNL_M, "Lunar Lake MEI" },
};

/* Global device pointer for mei_find_gsc_proxy_device() */
static device_t mei_gsc_dev;

/* ================================================================
 * MMIO Register Helpers
 *
 * All MEI hardware interaction goes through four 32-bit registers at
 * BAR0.  See mei_reg.h for the register map and bit definitions.
 *
 * Ref: DCMI-HI 1.0 spec, Section 4 "HECI Hardware Interface"
 * Ref: Intel PCH Datasheet, "ME Host Interface" chapter
 * ================================================================ */

static __inline uint32_t
mei_read(struct mei_softc *sc, uint32_t reg)
{
	return (bus_space_read_4(sc->bst, sc->bsh, reg));
}

static __inline void
mei_write(struct mei_softc *sc, uint32_t reg, uint32_t val)
{
	bus_space_write_4(sc->bst, sc->bsh, reg, val);
}

/*
 * Read the host circular buffer depth from H_CSR[31:24] (H_CBD field).
 * This is a hardware-fixed value indicating how many DWORD slots the
 * host-to-ME buffer can hold.
 */
static __inline int
mei_hbuf_depth(struct mei_softc *sc)
{
	return ((mei_read(sc, MEI_H_CSR) & MEI_H_CSR_CBD_MASK) >>
	    MEI_H_CSR_CBD_SHIFT);
}

/*
 * Calculate how many DWORD slots are currently occupied in the host
 * circular buffer.  The write pointer (H_CBWP) is incremented by the
 * host on each DWORD write to H_CB_WW; the read pointer (H_CBRP) is
 * incremented by ME as it consumes data.  The difference (cast to
 * uint8_t for 8-bit wrap-around) gives the fill level.
 */
static __inline int
mei_hbuf_filled(struct mei_softc *sc)
{
	uint32_t csr;
	int wp, rp;

	csr = mei_read(sc, MEI_H_CSR);
	wp = (csr & MEI_H_CSR_CBWP_MASK) >> MEI_H_CSR_CBWP_SHIFT;
	rp = (csr & MEI_H_CSR_CBRP_MASK) >> MEI_H_CSR_CBRP_SHIFT;
	return ((uint8_t)(wp - rp));
}

static __inline int
mei_hbuf_empty_slots(struct mei_softc *sc)
{
	return (sc->hbuf_depth - mei_hbuf_filled(sc));
}

/*
 * Count available DWORD slots in the ME-to-host circular buffer.
 * Read from ME_CSR_HA (offset 0x0C, read-only from host).
 * ME_CBWP is updated by ME firmware when it writes data;
 * ME_CBRP is updated by the host as it reads from ME_CB_RW.
 */
static __inline int
mei_me_count_avail(struct mei_softc *sc)
{
	uint32_t csr;
	int wp, rp;

	csr = mei_read(sc, MEI_ME_CSR_HA);
	wp = (csr & MEI_ME_CSR_CBWP_MASK) >> MEI_ME_CSR_CBWP_SHIFT;
	rp = (csr & MEI_ME_CSR_CBRP_MASK) >> MEI_ME_CSR_CBRP_SHIFT;
	return ((uint8_t)(wp - rp));
}

/*
 * Set the Host Interrupt Generate (H_IG) bit in H_CSR.
 *
 * This is the mechanism by which the host notifies ME firmware that
 * new data has been written to the host circular buffer (H_CB_WW),
 * or that the host has consumed data from the ME circular buffer
 * (ME_CB_RW).  ME firmware monitors this bit to know when to process
 * outbound messages or reclaim inbound buffer space.
 *
 * We must be careful not to accidentally clear the interrupt status
 * bit (H_IS) by writing a 1 to it — it is write-1-to-clear.
 *
 * Ref: DCMI-HI 1.0 spec, Section 4.1.1 "H_CSR", H_IG description
 */
static void
mei_hcsr_set_hig(struct mei_softc *sc)
{
	uint32_t csr;

	csr = mei_read(sc, MEI_H_CSR);
	csr |= MEI_H_CSR_IG;
	csr &= ~MEI_H_CSR_IS;
	mei_write(sc, MEI_H_CSR, csr);
}

/* ================================================================
 * Circular Buffer I/O
 *
 * The MEI hardware uses a FIFO-style interface where each 32-bit write
 * to H_CB_WW (offset 0x00) pushes one DWORD into the host-to-ME buffer,
 * and each 32-bit read from ME_CB_RW (offset 0x08) pops one DWORD from
 * the ME-to-host buffer.  The hardware automatically advances the
 * internal read/write pointers — the driver just reads/writes the
 * window register repeatedly.
 *
 * All data on the circular buffers is DWORD-aligned.  Messages shorter
 * than a DWORD multiple are padded; we handle the tail bytes carefully
 * to avoid reading/writing beyond the payload.
 *
 * Ref: DCMI-HI 1.0 spec, Section 4.2 "Circular Buffer Operation"
 * ================================================================ */

/*
 * Write data to the host circular buffer, DWORD at a time.
 * Caller must ensure sufficient empty slots.
 */
static void
mei_write_slots(struct mei_softc *sc, const void *data, size_t len)
{
	const uint32_t *src = (const uint32_t *)data;
	size_t i;

	for (i = 0; i < len / MEI_SLOT_SIZE; i++)
		mei_write(sc, MEI_H_CB_WW, src[i]);

	/* Handle non-DWORD-aligned tail */
	if (len % MEI_SLOT_SIZE) {
		uint32_t tail = 0;
		memcpy(&tail, (const uint8_t *)data + (i * MEI_SLOT_SIZE),
		    len % MEI_SLOT_SIZE);
		mei_write(sc, MEI_H_CB_WW, tail);
	}
}

/*
 * Read data from the ME circular buffer, DWORD at a time.
 */
static void
mei_read_slots(struct mei_softc *sc, void *data, size_t len)
{
	uint32_t *dst = (uint32_t *)data;
	size_t i;

	for (i = 0; i < len / MEI_SLOT_SIZE; i++)
		dst[i] = mei_read(sc, MEI_ME_CB_RW);

	if (len % MEI_SLOT_SIZE) {
		uint32_t tail;
		tail = mei_read(sc, MEI_ME_CB_RW);
		memcpy((uint8_t *)data + (i * MEI_SLOT_SIZE), &tail,
		    len % MEI_SLOT_SIZE);
	}
}

/* ================================================================
 * HBM Message Send/Receive
 *
 * HBM (Host Bus Message) is the control protocol layered on top of the
 * MEI circular buffers.  HBM messages are distinguished from client data
 * by using me_addr=0, host_addr=0 in the MEI message header.
 *
 * The protocol is request-response: the host sends a request (e.g.,
 * HOST_START_REQ), and ME firmware replies (HOST_START_RES).  Flow
 * control messages (FLOW_CONTROL, cmd 0x08) are sent in both directions
 * to manage per-client credit-based TX throttling.
 *
 * Ref: DCMI-HI 1.0 spec, Section 7 "Host Bus Messages"
 * ================================================================ */

/*
 * Send an HBM message (me_addr=0, host_addr=0).
 * Must be called with mtx held.
 */
static int
mei_hbm_send(struct mei_softc *sc, const void *msg, size_t len)
{
	uint32_t hdr;
	int slots_needed;

	slots_needed = 1 + howmany(len, MEI_SLOT_SIZE); /* 1 for header */
	if (mei_hbuf_empty_slots(sc) < slots_needed) {
		device_printf(sc->dev, "HBM send: not enough buffer space\n");
		return (EBUSY);
	}

	hdr = mei_hdr_pack(0, 0, len, true);
	mei_write(sc, MEI_H_CB_WW, hdr);
	mei_write_slots(sc, msg, len);
	mei_hcsr_set_hig(sc);

	return (0);
}

/*
 * Send a flow control credit to ME for the GSC proxy client.
 * Must be called with mtx held.
 */
static int
mei_send_flow_control(struct mei_softc *sc)
{
	struct mei_hbm_flow_control fc;

	memset(&fc, 0, sizeof(fc));
	fc.hbm_cmd = MEI_HBM_FLOW_CONTROL;
	fc.me_addr = sc->gsc_me_addr;
	fc.host_addr = sc->gsc_host_addr;

	return (mei_hbm_send(sc, &fc, sizeof(fc)));
}

/*
 * HBM Response Processing (called from interrupt context)
 */
static void
mei_hbm_dispatch(struct mei_softc *sc, const uint8_t *data, size_t len)
{
	uint8_t cmd;

	if (len < 1)
		return;

	cmd = data[0];

	switch (cmd) {
	case MEI_HBM_HOST_START_RES: {
		const struct mei_hbm_host_start_res *res;

		if (len < sizeof(*res))
			break;
		res = (const struct mei_hbm_host_start_res *)data;
		if (!res->host_version_supported) {
			device_printf(sc->dev,
			    "ME rejected HBM version %d.%d\n",
			    MEI_HBM_MAJOR_VERSION, MEI_HBM_MINOR_VERSION);
			break;
		}
		sc->hbm_ver_major = res->major_version;
		sc->hbm_ver_minor = res->minor_version;
		sc->hbm_state = MEI_HBM_STARTING;
		cv_signal(&sc->hbm_cv);
		break;
	}

	case MEI_HBM_HOST_ENUM_RES: {
		const struct mei_hbm_host_enum_res *res;

		if (len < sizeof(*res))
			break;
		res = (const struct mei_hbm_host_enum_res *)data;
		memcpy(sc->me_clients_map, res->valid_addresses,
		    sizeof(sc->me_clients_map));
		sc->hbm_state = MEI_HBM_ENUM;
		cv_signal(&sc->hbm_cv);
		break;
	}

	case MEI_HBM_HOST_CLIENT_PROP_RES: {
		const struct mei_hbm_client_prop_res *res;

		if (len < sizeof(*res))
			break;
		res = (const struct mei_hbm_client_prop_res *)data;
		if (res->status == 0)
			memcpy(&sc->prop_result, &res->props,
			    sizeof(sc->prop_result));
		sc->prop_received = true;
		cv_signal(&sc->hbm_cv);
		break;
	}

	case MEI_HBM_CLIENT_CONNECT_RES: {
		const struct mei_hbm_client_connect_res *res;

		if (len < sizeof(*res))
			break;
		res = (const struct mei_hbm_client_connect_res *)data;
		sc->gsc_connect_status = res->status;
		sc->gsc_connect_received = true;
		cv_signal(&sc->hbm_cv);
		break;
	}

	case MEI_HBM_FLOW_CONTROL: {
		const struct mei_hbm_flow_control *fc;

		if (len < sizeof(*fc))
			break;
		fc = (const struct mei_hbm_flow_control *)data;
		if (fc->me_addr == sc->gsc_me_addr &&
		    fc->host_addr == sc->gsc_host_addr) {
			sc->tx_credits++;
			cv_signal(&sc->tx_cv);
		}
		break;
	}

	case MEI_HBM_CLIENT_DISCONNECT_REQ: {
		/* ME is disconnecting our client */
		if (sc->gsc_connected) {
			device_printf(sc->dev, "GSC proxy client disconnected by ME\n");
			sc->gsc_connected = false;
			cv_broadcast(&sc->tx_cv);
			cv_broadcast(&sc->rx_cv);
		}
		break;
	}

	default:
		device_printf(sc->dev, "unhandled HBM cmd 0x%02x\n", cmd);
		break;
	}
}

/*
 * Interrupt Handler
 *
 * ME firmware asserts an interrupt (H_IS bit in H_CSR) when:
 *   - ME has written data to the ME-to-host circular buffer
 *   - ME_RDY status changes (e.g., after reset completes)
 *   - ME grants flow control credits
 *
 * The handler reads all available messages from ME_CB_RW, dispatches
 * HBM messages to mei_hbm_dispatch(), and delivers client data to
 * the rx_buf for mei_gsc_proxy_recv() to consume.
 *
 * After processing, we set H_IG to acknowledge to ME that we've
 * consumed data from its circular buffer.
 *
 * Ref: DCMI-HI 1.0 spec, Section 4.1.1 "H_CSR", H_IS/H_IE bits
 */
static void
mei_intr(void *arg)
{
	struct mei_softc *sc = arg;
	uint32_t hcsr, mecsr;
	int slots;

	mtx_lock(&sc->mtx);

	hcsr = mei_read(sc, MEI_H_CSR);

	/* Check if this interrupt is for us */
	if (!(hcsr & MEI_H_CSR_IS)) {
		mtx_unlock(&sc->mtx);
		return;
	}

	/* Clear interrupt status */
	mei_write(sc, MEI_H_CSR, hcsr);

	/* Check if ME became ready */
	mecsr = mei_read(sc, MEI_ME_CSR_HA);
	if ((mecsr & MEI_ME_CSR_RDY) && !sc->hw_ready) {
		sc->hw_ready = true;
		cv_signal(&sc->hbm_cv);
	}

	/* Read any available messages from ME circular buffer */
	slots = mei_me_count_avail(sc);
	while (slots > 0) {
		uint32_t raw_hdr;
		uint8_t me_addr, host_addr;
		uint16_t msg_len;
		uint8_t buf[512]; /* temp buffer for HBM messages */
		int msg_slots;

		raw_hdr = mei_read(sc, MEI_ME_CB_RW);
		slots--;

		me_addr = (raw_hdr & MEI_HDR_ME_ADDR_MASK) >>
		    MEI_HDR_ME_ADDR_SHIFT;
		host_addr = (raw_hdr & MEI_HDR_HOST_ADDR_MASK) >>
		    MEI_HDR_HOST_ADDR_SHIFT;
		msg_len = (raw_hdr & MEI_HDR_LENGTH_MASK) >>
		    MEI_HDR_LENGTH_SHIFT;

		msg_slots = howmany(msg_len, MEI_SLOT_SIZE);
		if (msg_slots > slots) {
			device_printf(sc->dev,
			    "incomplete message: need %d slots, have %d\n",
			    msg_slots, slots);
			break;
		}

		if (me_addr == 0 && host_addr == 0) {
			/* HBM message */
			if (msg_len > sizeof(buf)) {
				device_printf(sc->dev,
				    "HBM message too large: %u\n", msg_len);
				/* drain the slots */
				while (msg_slots-- > 0)
					mei_read(sc, MEI_ME_CB_RW);
			} else {
				mei_read_slots(sc, buf, msg_len);
				mei_hbm_dispatch(sc, buf, msg_len);
			}
		} else if (me_addr == sc->gsc_me_addr &&
		    host_addr == sc->gsc_host_addr) {
			/*
			 * GSC proxy client data.  Messages larger than the
			 * 511-byte hardware limit arrive as fragments; append
			 * each at rx_off and only complete the message (and
			 * wake the receiver) when MSG_COMPLETE is set.
			 */
			if (sc->rx_off + msg_len > MEI_RX_BUF_SIZE) {
				device_printf(sc->dev,
				    "RX message too large: %zu\n",
				    sc->rx_off + msg_len);
				while (msg_slots-- > 0)
					mei_read(sc, MEI_ME_CB_RW);
				sc->rx_off = 0;
			} else {
				mei_read_slots(sc, sc->rx_buf + sc->rx_off,
				    msg_len);
				sc->rx_off += msg_len;
				if (raw_hdr & MEI_HDR_MSG_COMPLETE) {
					sc->rx_len = sc->rx_off;
					sc->rx_off = 0;
					sc->rx_ready = true;
					cv_signal(&sc->rx_cv);
				}
			}
		} else {
			/* Unknown client - drain */
			while (msg_slots-- > 0)
				mei_read(sc, MEI_ME_CB_RW);
		}

		slots -= msg_slots;
	}

	/* Acknowledge that we've read from the ME buffer */
	mei_hcsr_set_hig(sc);

	/*
	 * ME draining the host circular buffer is also signalled by an
	 * interrupt.  Wake any sender blocked mid-message waiting for
	 * buffer space to write its next fragment.
	 */
	cv_broadcast(&sc->tx_cv);

	mtx_unlock(&sc->mtx);
}

/* ================================================================
 * HBM Protocol Initialization (synchronous, runs during attach)
 *
 * The full init sequence executed during mei_pci_attach():
 *
 *   1. mei_hw_reset()         - Assert H_RST, wait for ME_RDY,
 *                               then set H_RDY to complete handshake
 *   2. mei_hbm_start()        - HOST_START_REQ/RES version negotiation
 *   3. mei_hbm_enum()         - HOST_ENUM_REQ/RES to get client bitmap
 *   4. mei_hbm_find_gsc_proxy() - Query properties of each valid ME
 *                               client to find the GSC proxy UUID
 *   5. mei_hbm_connect_gsc()  - CLIENT_CONNECT_REQ/RES to establish
 *                               a session, then send initial flow
 *                               control credit to ME
 *
 * All steps are synchronous: the calling thread sends a request, then
 * blocks on hbm_cv until the interrupt handler receives the response.
 *
 * Ref: DCMI-HI 1.0 spec, Section 5 "Reset Flow"
 * Ref: DCMI-HI 1.0 spec, Section 7.1 "Host Bus Message Protocol"
 * ================================================================ */
static int
mei_cv_timedwait(struct mei_softc *sc, struct cv *cv, int secs)
{
	int error;

	error = cv_timedwait(cv, &sc->mtx, secs * hz);
	return (error);
}

/*
 * Hardware reset sequence.
 *
 * Asserts H_RST in H_CSR to reset the MEI interface.  ME firmware will
 * de-assert ME_RST and set ME_RDY when it is ready to communicate again.
 * We wait for ME_RDY (signaled by the interrupt handler setting hw_ready),
 * then set H_RDY to complete the handshake.
 *
 * Ref: DCMI-HI 1.0 spec, Section 5 "Reset Flow"
 */
static int
mei_hw_reset(struct mei_softc *sc)
{
	uint32_t csr;
	int error;

	mtx_assert(&sc->mtx, MA_OWNED);

	sc->hw_ready = false;

	/* Assert host reset, enable interrupts */
	csr = mei_read(sc, MEI_H_CSR);
	csr |= MEI_H_CSR_RST | MEI_H_CSR_IG;
	csr |= MEI_H_CSR_IE;
	mei_write(sc, MEI_H_CSR, csr);

	/* Wait for ME to become ready */
	while (!sc->hw_ready) {
		error = mei_cv_timedwait(sc, &sc->hbm_cv,
		    MEI_HW_READY_TIMEOUT);
		if (error != 0) {
			device_printf(sc->dev,
			    "timeout waiting for ME ready\n");
			return (error);
		}
	}

	/* Clear host reset, set host ready */
	csr = mei_read(sc, MEI_H_CSR);
	csr &= ~MEI_H_CSR_RST;
	csr |= MEI_H_CSR_RDY | MEI_H_CSR_IG | MEI_H_CSR_IE;
	/* Don't re-assert IS */
	csr &= ~MEI_H_CSR_IS;
	mei_write(sc, MEI_H_CSR, csr);

	return (0);
}

/*
 * HBM version negotiation (step 1 of init).
 *
 * Send HOST_START_REQ with our supported HBM version.  ME responds
 * with HOST_START_RES containing the negotiated version and whether
 * our version is supported.
 *
 * Ref: DCMI-HI 1.0 spec, Section 7.2.1 "Host Start"
 */
static int
mei_hbm_start(struct mei_softc *sc)
{
	struct mei_hbm_host_start_req req;
	int error;

	mtx_assert(&sc->mtx, MA_OWNED);

	/* Step 1: HOST_START_REQ */
	memset(&req, 0, sizeof(req));
	req.hbm_cmd = MEI_HBM_HOST_START_REQ;
	req.major_version = MEI_HBM_MAJOR_VERSION;
	req.minor_version = MEI_HBM_MINOR_VERSION;

	sc->hbm_state = MEI_HBM_IDLE;
	error = mei_hbm_send(sc, &req, sizeof(req));
	if (error != 0)
		return (error);

	/* Wait for HOST_START_RES */
	while (sc->hbm_state == MEI_HBM_IDLE) {
		error = mei_cv_timedwait(sc, &sc->hbm_cv, MEI_HBM_TIMEOUT);
		if (error != 0) {
			device_printf(sc->dev,
			    "timeout waiting for HOST_START_RES\n");
			return (error);
		}
	}

	device_printf(sc->dev, "HBM version %d.%d negotiated\n",
	    sc->hbm_ver_major, sc->hbm_ver_minor);
	return (0);
}

/*
 * Client enumeration (step 2 of init).
 *
 * Send HOST_ENUM_REQ to get a bitmap of valid ME client addresses.
 * ME responds with HOST_ENUM_RES containing a 256-bit bitmap in
 * valid_addresses[32].
 *
 * Ref: DCMI-HI 1.0 spec, Section 7.2.2 "Host Enumeration"
 */
static int
mei_hbm_enum(struct mei_softc *sc)
{
	struct mei_hbm_host_enum_req req;
	int error;

	mtx_assert(&sc->mtx, MA_OWNED);

	/* Step 2: HOST_ENUM_REQ */
	memset(&req, 0, sizeof(req));
	req.hbm_cmd = MEI_HBM_HOST_ENUM_REQ;

	sc->hbm_state = MEI_HBM_STARTING;
	error = mei_hbm_send(sc, &req, sizeof(req));
	if (error != 0)
		return (error);

	while (sc->hbm_state == MEI_HBM_STARTING) {
		error = mei_cv_timedwait(sc, &sc->hbm_cv, MEI_HBM_TIMEOUT);
		if (error != 0) {
			device_printf(sc->dev,
			    "timeout waiting for HOST_ENUM_RES\n");
			return (error);
		}
	}

	return (0);
}

/*
 * Find the GSC proxy client (step 3 of init).
 *
 * For each valid ME client address from the enumeration bitmap,
 * query its properties (UUID, protocol version, MTU) via
 * HOST_CLIENT_PROPERTIES_REQ/RES.  Match the UUID against the
 * well-known GSC proxy UUID to find the right client.
 *
 * Ref: DCMI-HI 1.0 spec, Section 7.2.3 "Host Client Properties"
 */
static int
mei_hbm_find_gsc_proxy(struct mei_softc *sc)
{
	struct mei_hbm_client_prop_req req;
	int addr, error;

	mtx_assert(&sc->mtx, MA_OWNED);

	sc->gsc_me_addr = -1;

	/* Step 3: Query properties for each valid ME client */
	for (addr = 0; addr < MEI_MAX_ME_CLIENTS; addr++) {
		/* Check bitmap */
		if (!(sc->me_clients_map[addr / 8] & (1 << (addr % 8))))
			continue;

		memset(&req, 0, sizeof(req));
		req.hbm_cmd = MEI_HBM_HOST_CLIENT_PROP_REQ;
		req.me_addr = addr;

		sc->prop_received = false;
		error = mei_hbm_send(sc, &req, sizeof(req));
		if (error != 0)
			return (error);

		while (!sc->prop_received) {
			error = mei_cv_timedwait(sc, &sc->hbm_cv,
			    MEI_HBM_TIMEOUT);
			if (error != 0) {
				device_printf(sc->dev,
				    "timeout on client properties for addr %d\n",
				    addr);
				return (error);
			}
		}

		/* Compare UUID with GSC proxy */
		if (memcmp(sc->prop_result.protocol_name,
		    mei_uuid_gsc_proxy, 16) == 0) {
			sc->gsc_me_addr = addr;
			sc->gsc_mtu = sc->prop_result.max_msg_length;
			device_printf(sc->dev,
			    "found GSC proxy client at ME addr %d, MTU %u\n",
			    addr, sc->gsc_mtu);
			return (0);
		}
	}

	device_printf(sc->dev, "GSC proxy client not found in ME\n");
	return (ENOENT);
}

/*
 * Connect to the GSC proxy client (step 4 of init).
 *
 * Send CLIENT_CONNECT_REQ with the ME address found in step 3 and
 * a locally-chosen host address.  On success, send an initial
 * FLOW_CONTROL message to ME so it can begin sending us data.
 *
 * Ref: DCMI-HI 1.0 spec, Section 7.2.4 "Client Connect"
 * Ref: DCMI-HI 1.0 spec, Section 7.2.5 "Flow Control"
 */
static int
mei_hbm_connect_gsc(struct mei_softc *sc)
{
	struct mei_hbm_client_connect_req req;
	int error;

	mtx_assert(&sc->mtx, MA_OWNED);

	/* Step 4: Connect to GSC proxy client */
	sc->gsc_host_addr = 1; /* use host address 1 */

	memset(&req, 0, sizeof(req));
	req.hbm_cmd = MEI_HBM_CLIENT_CONNECT_REQ;
	req.me_addr = sc->gsc_me_addr;
	req.host_addr = sc->gsc_host_addr;

	sc->gsc_connect_received = false;
	error = mei_hbm_send(sc, &req, sizeof(req));
	if (error != 0)
		return (error);

	while (!sc->gsc_connect_received) {
		error = mei_cv_timedwait(sc, &sc->hbm_cv, MEI_HBM_TIMEOUT);
		if (error != 0) {
			device_printf(sc->dev,
			    "timeout on CLIENT_CONNECT_RES\n");
			return (error);
		}
	}

	if (sc->gsc_connect_status != 0) {
		device_printf(sc->dev,
		    "GSC proxy connect failed, status %d\n",
		    sc->gsc_connect_status);
		return (EIO);
	}

	sc->gsc_connected = true;

	/* Send initial flow control to ME so it can send us data */
	error = mei_send_flow_control(sc);
	if (error != 0)
		return (error);

	device_printf(sc->dev, "GSC proxy client connected\n");
	return (0);
}

static int
mei_init_hw(struct mei_softc *sc)
{
	int error;

	mtx_lock(&sc->mtx);

	error = mei_hw_reset(sc);
	if (error != 0)
		goto out;

	error = mei_hbm_start(sc);
	if (error != 0)
		goto out;

	error = mei_hbm_enum(sc);
	if (error != 0)
		goto out;

	error = mei_hbm_find_gsc_proxy(sc);
	if (error != 0)
		goto out;

	error = mei_hbm_connect_gsc(sc);
	if (error != 0)
		goto out;

	sc->hbm_state = MEI_HBM_STARTED;
	mei_gsc_dev = sc->dev;

out:
	mtx_unlock(&sc->mtx);
	return (error);
}

/* ================================================================
 * PCI Driver Methods
 * ================================================================ */

static int
mei_pci_probe(device_t dev)
{
	uint16_t vendor, devid;
	size_t i;

	vendor = pci_get_vendor(dev);
	devid = pci_get_device(dev);

	for (i = 0; i < nitems(mei_pci_ids); i++) {
		if (vendor == mei_pci_ids[i].vendor &&
		    devid == mei_pci_ids[i].device) {
			device_set_desc(dev, mei_pci_ids[i].desc);
			return (BUS_PROBE_DEFAULT);
		}
	}

	return (ENXIO);
}

static int
mei_pci_attach(device_t dev)
{
	struct mei_softc *sc;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->gsc_me_addr = -1;

	/* Initialize synchronization primitives */
	mtx_init(&sc->mtx, "mei", NULL, MTX_DEF);
	cv_init(&sc->hbm_cv, "mei_hbm");
	cv_init(&sc->tx_cv, "mei_tx");
	cv_init(&sc->rx_cv, "mei_rx");

	/* Allocate RX buffer */
	sc->rx_buf = malloc(MEI_RX_BUF_SIZE, M_DEVBUF, M_WAITOK | M_ZERO);

	/* Map BAR0 */
	sc->mem_rid = PCIR_BAR(0);
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->mem_rid, RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "failed to allocate BAR0\n");
		error = ENXIO;
		goto fail;
	}
	sc->bst = rman_get_bustag(sc->mem_res);
	sc->bsh = rman_get_bushandle(sc->mem_res);

	/* Get host circular buffer depth */
	sc->hbuf_depth = mei_hbuf_depth(sc);
	device_printf(dev, "host buffer depth: %d slots\n", sc->hbuf_depth);

	/* Enable PCI bus mastering */
	pci_enable_busmaster(dev);

	/* Allocate and setup interrupt */
	sc->irq_rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &sc->irq_rid, RF_SHAREABLE | RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "failed to allocate IRQ\n");
		error = ENXIO;
		goto fail;
	}

	error = bus_setup_intr(dev, sc->irq_res,
	    INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, mei_intr, sc, &sc->irq_handle);
	if (error != 0) {
		device_printf(dev, "failed to setup interrupt: %d\n", error);
		goto fail;
	}

	/* Initialize hardware and run HBM protocol */
	error = mei_init_hw(sc);
	if (error != 0) {
		device_printf(dev, "hardware init failed: %d\n", error);
		goto fail;
	}

	return (0);

fail:
	mei_pci_detach(dev);
	return (error);
}

static int
mei_pci_detach(device_t dev)
{
	struct mei_softc *sc;

	sc = device_get_softc(dev);

	if (mei_gsc_dev == dev)
		mei_gsc_dev = NULL;

	/* Disable interrupts */
	if (sc->mem_res != NULL) {
		uint32_t csr;
		csr = mei_read(sc, MEI_H_CSR);
		csr &= ~MEI_H_CSR_IE;
		csr |= MEI_H_CSR_IS;
		mei_write(sc, MEI_H_CSR, csr);
	}

	if (sc->irq_handle != NULL)
		bus_teardown_intr(dev, sc->irq_res, sc->irq_handle);
	if (sc->irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid,
		    sc->irq_res);
	if (sc->mem_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid,
		    sc->mem_res);

	pci_disable_busmaster(dev);

	if (sc->rx_buf != NULL)
		free(sc->rx_buf, M_DEVBUF);

	cv_destroy(&sc->rx_cv);
	cv_destroy(&sc->tx_cv);
	cv_destroy(&sc->hbm_cv);
	mtx_destroy(&sc->mtx);

	return (0);
}

/* ================================================================
 * Exported KPI
 *
 * These functions are called by drm-kmod (intel_gsc_proxy.c) to
 * relay GSC proxy messages between the i915 GPU driver and the
 * CSME firmware.  The send/recv semantics match what the i915 GSC
 * proxy code expects from its component ops:
 *   send() - push a message to CSME, return bytes sent or -errno
 *   recv() - block until CSME replies, return bytes received or -errno
 *
 * Both functions are synchronous and block with the MEI mutex held.
 * Flow control is managed via HBM FLOW_CONTROL credits from ME.
 * ================================================================ */
device_t
mei_find_gsc_proxy_device(void)
{
	return (mei_gsc_dev);
}

int
mei_gsc_proxy_send(device_t dev, const void *buf, size_t size)
{
	struct mei_softc *sc;
	uint32_t hdr;
	size_t offset, frag, avail;
	int empty, error;
	bool complete;

	sc = device_get_softc(dev);

	mtx_lock(&sc->mtx);

	if (!sc->gsc_connected) {
		mtx_unlock(&sc->mtx);
		return (-ENODEV);
	}

	if (size > sc->gsc_mtu) {
		mtx_unlock(&sc->mtx);
		return (-EFBIG);
	}

	/* Wait for TX flow control credit */
	while (sc->tx_credits == 0 && sc->gsc_connected) {
		error = mei_cv_timedwait(sc, &sc->tx_cv, MEI_CLIENT_TIMEOUT);
		if (error != 0) {
			mtx_unlock(&sc->mtx);
			device_printf(sc->dev,
			    "timeout waiting for TX credit\n");
			return (-ETIMEDOUT);
		}
	}

	if (!sc->gsc_connected) {
		mtx_unlock(&sc->mtx);
		return (-ENODEV);
	}

	sc->tx_credits--;

	/*
	 * Write the message, fragmenting as needed.  A single hardware
	 * message is limited to 511 bytes (9-bit header length field) and
	 * by the free space in the host circular buffer, while GSC proxy
	 * messages can be tens of KB.  Each fragment except the last is
	 * sent with MSG_COMPLETE clear; ME reassembles on its side.  One
	 * flow control credit covers the complete message.
	 *
	 * Between fragments we wait for ME to drain the circular buffer;
	 * the interrupt handler broadcasts tx_cv when that happens.  The
	 * timed wait doubles as a polling fallback in case draining does
	 * not raise an interrupt.
	 */
	offset = 0;
	while (offset < size) {
		/* Need room for the header plus at least one payload slot */
		while ((empty = mei_hbuf_empty_slots(sc)) < 2 &&
		    sc->gsc_connected) {
			error = mei_cv_timedwait(sc, &sc->tx_cv,
			    MEI_CLIENT_TIMEOUT);
			if (error != 0) {
				mtx_unlock(&sc->mtx);
				device_printf(sc->dev,
				    "timeout waiting for host buffer space\n");
				return (-ETIMEDOUT);
			}
		}
		if (!sc->gsc_connected) {
			mtx_unlock(&sc->mtx);
			return (-ENODEV);
		}

		avail = (size_t)(empty - 1) * MEI_SLOT_SIZE;
		frag = MIN(size - offset, MIN(avail, MEI_FRAG_LENGTH_MAX));
		complete = (offset + frag == size);

		hdr = mei_hdr_pack(sc->gsc_me_addr, sc->gsc_host_addr, frag,
		    complete);
		mei_write(sc, MEI_H_CB_WW, hdr);
		mei_write_slots(sc, (const uint8_t *)buf + offset, frag);
		mei_hcsr_set_hig(sc);

		offset += frag;
	}

	mtx_unlock(&sc->mtx);

	return ((int)size);
}

int
mei_gsc_proxy_recv(device_t dev, void *buf, size_t size)
{
	struct mei_softc *sc;
	int len, error;

	sc = device_get_softc(dev);

	mtx_lock(&sc->mtx);

	if (!sc->gsc_connected) {
		mtx_unlock(&sc->mtx);
		return (-ENODEV);
	}

	/* Wait for data */
	while (!sc->rx_ready && sc->gsc_connected) {
		error = mei_cv_timedwait(sc, &sc->rx_cv, MEI_CLIENT_TIMEOUT);
		if (error != 0) {
			mtx_unlock(&sc->mtx);
			device_printf(sc->dev,
			    "timeout waiting for RX data\n");
			return (-ETIMEDOUT);
		}
	}

	if (!sc->gsc_connected) {
		mtx_unlock(&sc->mtx);
		return (-ENODEV);
	}

	len = (int)MIN(size, sc->rx_len);
	memcpy(buf, sc->rx_buf, len);
	sc->rx_ready = false;

	/* Grant flow control credit back to ME */
	mei_send_flow_control(sc);

	mtx_unlock(&sc->mtx);

	return (len);
}

/* ================================================================
 * Module Glue
 * ================================================================ */

static device_method_t mei_pci_methods[] = {
	DEVMETHOD(device_probe,		mei_pci_probe),
	DEVMETHOD(device_attach,	mei_pci_attach),
	DEVMETHOD(device_detach,	mei_pci_detach),
	DEVMETHOD_END
};

static driver_t mei_pci_driver = {
	"mei",
	mei_pci_methods,
	sizeof(struct mei_softc),
};

DRIVER_MODULE(mei, pci, mei_pci_driver, NULL, NULL);
MODULE_VERSION(mei, 1);
MODULE_PNP_INFO("U16:vendor;U16:device;D:#", pci, mei, mei_pci_ids,
    nitems(mei_pci_ids));
