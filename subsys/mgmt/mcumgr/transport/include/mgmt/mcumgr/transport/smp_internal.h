/*
 * Copyright Runtime.io 2018. All rights reserved.
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MGMT_MCUMGR_SMP_INTERNAL_H_
#define MGMT_MCUMGR_SMP_INTERNAL_H_

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/mgmt/mcumgr/smp/smp.h>
#include <zephyr/mgmt/mcumgr/transport/smp.h>
#include <zcbor_encode.h>

#ifdef __cplusplus
extern "C" {
#endif

struct smp_hdr {
#ifdef CONFIG_LITTLE_ENDIAN
	uint8_t  nh_op:3;		/* MGMT_OP_[...] */
	uint8_t  nh_version:2;
	uint8_t  _res1:3;
#else
	uint8_t  _res1:3;
	uint8_t  nh_version:2;
	uint8_t  nh_op:3;		/* MGMT_OP_[...] */
#endif
	uint8_t  nh_flags;		/* Reserved for future flags */
	uint16_t nh_len;		/* Length of the payload */
	uint16_t nh_group;		/* MGMT_GROUP_ID_[...] */
	uint8_t  nh_seq;		/* Sequence number */
	uint8_t  nh_id;			/* Message ID within group */
} __packed;

struct smp_transport;
struct zephyr_smp_transport;

/* Not under CONFIG_MCUMGR_TRANSPORT_FORWARD_TREE: a leaf has no forward tree and
 * still has to recognise the flags to echo the routing trailer back to the node
 * that forwarded the request. See CONFIG_MCUMGR_SMP_ROUTING_TRAILER_ECHO.
 */
enum smp_hdr_flag {
	/** A routing trailer is appended after the payload. */
	SMP_HDR_FLAG_FORWARD_TREE = 0x80,
	/** The frame was originated by a forward tree node, not by the host. */
	SMP_HDR_FLAG_FT_NODE_ORIGIN = 0x40,
} __packed;

/** Header flags a response echoes back; anything else is cleared. */
#define SMP_HDR_FLAG_ROUTING_MASK					\
	((uint8_t)(SMP_HDR_FLAG_FORWARD_TREE | SMP_HDR_FLAG_FT_NODE_ORIGIN))

#if defined(CONFIG_MCUMGR_TRANSPORT_FORWARD_TREE)
/* The trailer is a path plus the position along it. `hop` counts the hops left to
 * travel and indexes the next port to take; `up` counts the hops already
 * travelled, so that a response can retrace them. `hop + up` is the path length
 * and never changes: going down `hop--, up++`, coming back up the reverse. A
 * frame is home when `up` reaches zero.
 */
struct smp_forward_tree {
#ifdef CONFIG_LITTLE_ENDIAN
	uint64_t port:56;
	uint64_t up:4;
	uint64_t hop:4;
#else
	uint64_t hop:4;
	uint64_t up:4;
	uint64_t port:56;
#endif
} __packed;

BUILD_ASSERT(sizeof(struct smp_forward_tree) == sizeof(uint64_t),
	     "The routing trailer must stay eight bytes wide");

/** Ports the 56-bit path can hold, one nibble each. */
#define SMP_FORWARD_TREE_MAX_HOPS 14

struct smp_forward_tree_transport {
	const struct device *const dev;
	enum smp_transport_type type;
};
#endif

/**
 * @brief Enqueues an incoming SMP request packet for processing.
 *
 * This function always consumes the supplied net_buf.
 *
 * @param smtp                  The transport to use to send the corresponding
 *                                  response(s).
 * @param nb                    The request packet to process.
 */
void smp_rx_req(struct smp_transport *smtp, struct net_buf *nb);

#ifdef CONFIG_SMP_CLIENT
/**
 * @brief Get work queue for SMP client.
 *
 * @return SMP work queue object.
 */
struct k_work_q *smp_get_wq(void);
#endif

/**
 * @brief Allocates a request buffer.
 *
 * @param arg		The streamer providing the callback.
 *
 * @return	Newly-allocated buffer on success
 *		NULL on failure.
 */
struct net_buf *smp_alloc_req(void *arg, void *priv);

/**
 * @brief Allocates a response buffer.
 *
 * If a source buf is provided, its user data is copied into the new buffer.
 *
 * @param req		An optional source buffer to copy user data from.
 * @param arg		The streamer providing the callback.
 *
 * @return	Newly-allocated buffer on success
 *		NULL on failure.
 */
void *smp_alloc_rsp(const void *req, void *arg);


/**
 * @brief Frees an allocated buffer.
 *
 * @param buf		The buffer to free.
 * @param arg		The streamer providing the callback.
 */
void smp_free_buf(void *buf, void *arg);

/**
 * @brief	Reeset a zcbor encoder to allow a new response.
 *
 * If a response has already been (partially) generated than this will allow resetting back to
 * the default state so that new response can be used (e.g. an error).
 *
 * @param streamer	The streamer providing the required SMP callbacks.
 *
 * @return	true on success, false on failure (memory error).
 */
static inline bool smp_mgmt_reset_zse(struct smp_streamer *streamer)
{
	zcbor_state_t *zse = streamer->writer->zs;

	/* Because there is already data in the buffer, it must be cleared first */
	net_buf_reset(streamer->writer->nb);
	streamer->writer->nb->len = sizeof(struct smp_hdr);
	zcbor_new_encode_state(zse, ARRAY_SIZE(streamer->writer->zs),
			       streamer->writer->nb->data + sizeof(struct smp_hdr),
			       net_buf_tailroom(streamer->writer->nb), 0);

	return zcbor_map_start_encode(zse, CONFIG_MCUMGR_SMP_CBOR_MAX_MAIN_MAP_ENTRIES);
}

#ifdef __cplusplus
}
#endif

#endif /* MGMT_MCUMGR_SMP_INTERNAL_H_ */
