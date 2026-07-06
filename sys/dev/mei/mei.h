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

#ifndef _DEV_MEI_MEI_H_
#define _DEV_MEI_MEI_H_

#include <sys/types.h>
#include <sys/bus.h>

/*
 * Public KPI for the Intel MEI driver.
 *
 * This interface is used by drm-kmod (i915) to send and receive GSC proxy
 * messages to/from the Intel CSME firmware via the PCH MEI PCI device.
 */

/*
 * Find the first attached MEI device that has a connected GSC proxy client.
 * Returns the device_t or NULL if not found/not ready.
 */
device_t mei_find_gsc_proxy_device(void);

/*
 * Send a GSC proxy message to CSME.
 * Blocks until the message is sent (waits for TX flow control credit).
 *
 * Returns bytes sent on success, negative errno on failure.
 */
int mei_gsc_proxy_send(device_t dev, const void *buf, size_t size);

/*
 * Receive a GSC proxy message from CSME.
 * Blocks until a message is available.
 *
 * Returns bytes received on success, negative errno on failure.
 */
int mei_gsc_proxy_recv(device_t dev, void *buf, size_t size);

#endif /* _DEV_MEI_MEI_H_ */
