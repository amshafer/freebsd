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

#ifndef _DEV_MEI_MEI_REG_H_
#define _DEV_MEI_MEI_REG_H_

/*
 * Intel Management Engine Interface (MEI) MMIO Registers.
 *
 * The MEI hardware uses a pair of circular buffers (host and ME) accessed
 * through MMIO registers at BAR0.  The host writes data to H_CB_WW and
 * reads data from ME_CB_RW.  Control/status is managed through H_CSR
 * (read-write) and ME_CSR_HA (read-only from host).
 */

/* MMIO register offsets from BAR0 */
#define MEI_H_CB_WW		0x00	/* Host Circular Buffer Write Window */
#define MEI_H_CSR		0x04	/* Host Control Status Register */
#define MEI_ME_CB_RW		0x08	/* ME Circular Buffer Read Window */
#define MEI_ME_CSR_HA		0x0C	/* ME Control Status Host Access */

/* H_CSR (Host Control Status Register) bits */
#define MEI_H_CSR_IE		0x00000001	/* Host Interrupt Enable */
#define MEI_H_CSR_IS		0x00000002	/* Host Interrupt Status */
#define MEI_H_CSR_IG		0x00000004	/* Host Interrupt Generate */
#define MEI_H_CSR_RDY		0x00000008	/* Host Ready */
#define MEI_H_CSR_RST		0x00000010	/* Host Reset */
#define MEI_H_CSR_CBRP_MASK	0x0000FF00	/* CB Read Pointer */
#define MEI_H_CSR_CBRP_SHIFT	8
#define MEI_H_CSR_CBWP_MASK	0x00FF0000	/* CB Write Pointer */
#define MEI_H_CSR_CBWP_SHIFT	16
#define MEI_H_CSR_CBD_MASK	0xFF000000	/* CB Depth */
#define MEI_H_CSR_CBD_SHIFT	24

/* ME_CSR_HA (ME Control Status Host Access) bits - read only from host */
#define MEI_ME_CSR_IE		0x00000001	/* ME Interrupt Enable */
#define MEI_ME_CSR_IS		0x00000002	/* ME Interrupt Status */
#define MEI_ME_CSR_IG		0x00000004	/* ME Interrupt Generate */
#define MEI_ME_CSR_RDY		0x00000008	/* ME Ready */
#define MEI_ME_CSR_RST		0x00000010	/* ME Reset */
#define MEI_ME_CSR_CBRP_MASK	0x0000FF00	/* ME CB Read Pointer */
#define MEI_ME_CSR_CBRP_SHIFT	8
#define MEI_ME_CSR_CBWP_MASK	0x00FF0000	/* ME CB Write Pointer */
#define MEI_ME_CSR_CBWP_SHIFT	16
#define MEI_ME_CSR_CBD_MASK	0xFF000000	/* ME CB Depth */
#define MEI_ME_CSR_CBD_SHIFT	24

/* PCI Config Space Firmware Status Registers */
#define MEI_PCI_HFS_1		0x40
#define MEI_PCI_HFS_2		0x48
#define MEI_PCI_HFS_3		0x60

/*
 * MEI Bus Interface Message Header.
 *
 * Every message (HBM or client data) on the circular buffers is prefixed
 * with this 4-byte header.  Packed as a single uint32_t.
 */
#define MEI_HDR_ME_ADDR_MASK	0x000000FF
#define MEI_HDR_ME_ADDR_SHIFT	0
#define MEI_HDR_HOST_ADDR_MASK	0x0000FF00
#define MEI_HDR_HOST_ADDR_SHIFT	8
#define MEI_HDR_LENGTH_MASK	0x01FF0000
#define MEI_HDR_LENGTH_SHIFT	16
/*
 * The 9-bit length field limits a single hardware message to 511 bytes;
 * larger client messages must be sent as multiple fragments with
 * MEI_HDR_MSG_COMPLETE set only on the last one.  Non-final fragments
 * must be DWORD-multiples so no pad bytes are inserted mid-message,
 * hence the slot-aligned maximum below.
 */
#define MEI_HDR_LENGTH_MAX	511
#define MEI_FRAG_LENGTH_MAX	(MEI_HDR_LENGTH_MAX & ~(MEI_SLOT_SIZE - 1))
/* bits 25-27 reserved */
#define MEI_HDR_EXTENDED	(1U << 28)
#define MEI_HDR_DMAR		(1U << 29)
#define MEI_HDR_INTERNAL	(1U << 30)
#define MEI_HDR_MSG_COMPLETE	(1U << 31)

static __inline uint32_t
mei_hdr_pack(uint8_t me_addr, uint8_t host_addr, uint16_t length,
    bool msg_complete)
{
	uint32_t hdr;

	hdr = ((uint32_t)me_addr << MEI_HDR_ME_ADDR_SHIFT) |
	    ((uint32_t)host_addr << MEI_HDR_HOST_ADDR_SHIFT) |
	    ((uint32_t)(length & 0x1FF) << MEI_HDR_LENGTH_SHIFT);
	if (msg_complete)
		hdr |= MEI_HDR_MSG_COMPLETE;
	return (hdr);
}

/*
 * Host Bus Message (HBM) Command Opcodes.
 *
 * HBM messages use me_addr=0, host_addr=0 in the MEI header.
 * Response opcodes have bit 7 set.
 */
#define MEI_HBM_CMD_RES_FLAG	0x80

#define MEI_HBM_HOST_START_REQ		0x01
#define MEI_HBM_HOST_START_RES		0x81
#define MEI_HBM_HOST_STOP_REQ		0x02
#define MEI_HBM_HOST_STOP_RES		0x82
#define MEI_HBM_HOST_ENUM_REQ		0x04
#define MEI_HBM_HOST_ENUM_RES		0x84
#define MEI_HBM_HOST_CLIENT_PROP_REQ	0x05
#define MEI_HBM_HOST_CLIENT_PROP_RES	0x85
#define MEI_HBM_CLIENT_CONNECT_REQ	0x06
#define MEI_HBM_CLIENT_CONNECT_RES	0x86
#define MEI_HBM_CLIENT_DISCONNECT_REQ	0x07
#define MEI_HBM_CLIENT_DISCONNECT_RES	0x87
#define MEI_HBM_FLOW_CONTROL		0x08

/* HBM version we advertise to firmware */
#define MEI_HBM_MAJOR_VERSION	2
#define MEI_HBM_MINOR_VERSION	0

/*
 * HBM Message Structures.
 * All HBM messages are sent/received as payload inside the mei_msg_hdr.
 */

struct mei_hbm_host_start_req {
	uint8_t		hbm_cmd;	/* MEI_HBM_HOST_START_REQ */
	uint8_t		reserved;
	uint8_t		minor_version;
	uint8_t		major_version;
} __packed;

struct mei_hbm_host_start_res {
	uint8_t		hbm_cmd;	/* MEI_HBM_HOST_START_RES */
	uint8_t		host_version_supported;
	uint8_t		minor_version;
	uint8_t		major_version;
} __packed;

struct mei_hbm_host_enum_req {
	uint8_t		hbm_cmd;	/* MEI_HBM_HOST_ENUM_REQ */
	uint8_t		flags;
	uint8_t		reserved[2];
} __packed;

struct mei_hbm_host_enum_res {
	uint8_t		hbm_cmd;	/* MEI_HBM_HOST_ENUM_RES */
	uint8_t		reserved[3];
	uint8_t		valid_addresses[32];
} __packed;

struct mei_client_properties {
	uint8_t		protocol_name[16];	/* UUID, little-endian */
	uint8_t		protocol_version;
	uint8_t		max_connections;
	uint8_t		fixed_address;
	uint8_t		flags;			/* bit 0: single_recv_buf */
	uint32_t	max_msg_length;
} __packed;

struct mei_hbm_client_prop_req {
	uint8_t		hbm_cmd;	/* MEI_HBM_HOST_CLIENT_PROP_REQ */
	uint8_t		me_addr;
	uint8_t		reserved[2];
} __packed;

struct mei_hbm_client_prop_res {
	uint8_t		hbm_cmd;	/* MEI_HBM_HOST_CLIENT_PROP_RES */
	uint8_t		me_addr;
	uint8_t		status;
	uint8_t		reserved;
	struct mei_client_properties props;
} __packed;

struct mei_hbm_client_connect_req {
	uint8_t		hbm_cmd;	/* MEI_HBM_CLIENT_CONNECT_REQ */
	uint8_t		me_addr;
	uint8_t		host_addr;
	uint8_t		reserved;
} __packed;

struct mei_hbm_client_connect_res {
	uint8_t		hbm_cmd;	/* MEI_HBM_CLIENT_CONNECT_RES */
	uint8_t		me_addr;
	uint8_t		host_addr;
	uint8_t		status;
} __packed;

struct mei_hbm_flow_control {
	uint8_t		hbm_cmd;	/* MEI_HBM_FLOW_CONTROL */
	uint8_t		me_addr;
	uint8_t		host_addr;
	uint8_t		reserved[5];
} __packed;

/* MEI PCI Device IDs */
#define PCI_VENDOR_INTEL	0x8086
#define MEI_DEV_ID_MTL_M	0x7E70	/* Meteor Lake Point M */
#define MEI_DEV_ID_ARL_S	0x7F68	/* Arrow Lake Point S */
#define MEI_DEV_ID_ARL_H	0x7770	/* Arrow Lake Point H */
#define MEI_DEV_ID_LNL_M	0xA870	/* Lunar Lake Point M */

/* GSC Proxy UUID (little-endian): {0f73db04-97ab-4125-b893-e904ad0d5464} */
#define MEI_UUID_GSC_PROXY	\
	{ 0x04, 0xdb, 0x73, 0x0f, 0xab, 0x97, 0x25, 0x41, \
	  0xb8, 0x93, 0xe9, 0x04, 0xad, 0x0d, 0x54, 0x64 }

/* Slot size for circular buffer operations */
#define MEI_SLOT_SIZE		4	/* bytes per slot (DWORD) */

/* Timeouts */
#define MEI_HW_READY_TIMEOUT	5	/* seconds to wait for ME_RDY */
#define MEI_HBM_TIMEOUT		5	/* seconds per HBM exchange */
#define MEI_CLIENT_TIMEOUT	10	/* seconds for send/recv */

#endif /* _DEV_MEI_MEI_REG_H_ */
