/*
 * Copyright (c) 2018-2021 mcumgr authors
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief SMP - Simple Management Protocol.
 *
 * SMP is a basic protocol that sits on top of the mgmt layer.  SMP requests
 * and responses have the following format:
 *
 *	 [Offset 0]: Mgmt header
 *	 [Offset 8]: CBOR map of command-specific key-value pairs.
 *
 * SMP request packets may contain multiple concatenated requests.  Each
 * request must start at an offset that is a multiple of 4, so padding should
 * be inserted between requests as necessary.  Requests are processed
 * sequentially from the start of the packet to the end.  Each response is sent
 * individually in its own packet.  If a request elicits an error response,
 * processing of the packet is aborted.
 */

#ifndef H_SMP_
#define H_SMP_

#include <zephyr/net_buf.h>
#include <zephyr/mgmt/mcumgr/transport/smp.h>

#include <zcbor_common.h>

#ifdef __cplusplus
extern "C" {
#endif

/** SMP MCUmgr protocol version, part of the SMP header */
enum smp_mcumgr_version_t {
	/** Version 1: the original protocol */
	SMP_MCUMGR_VERSION_1 = 0,

	/** Version 2: adds more detailed error reporting capabilities */
	SMP_MCUMGR_VERSION_2,
};

struct cbor_nb_reader {
	struct net_buf *nb;
	/* CONFIG_MCUMGR_SMP_CBOR_MAX_DECODING_LEVELS + 2 translates to minimal
	 * zcbor backup states.
	 */
	zcbor_state_t zs[CONFIG_MCUMGR_SMP_CBOR_MAX_DECODING_LEVELS + 2];
};

struct cbor_nb_writer {
	struct net_buf *nb;
	zcbor_state_t zs[CONFIG_MCUMGR_SMP_CBOR_MAX_ENCODING_LEVELS + 2];

#if defined(CONFIG_MCUMGR_SMP_SUPPORT_ORIGINAL_PROTOCOL)
	uint16_t error_group;
	uint16_t error_ret;
#endif
};

/**
 * @brief Allocates a net_buf for holding an mcumgr request or response.
 *
 * @return      A newly-allocated buffer net_buf on success;
 *              NULL on failure.
 */
struct net_buf *smp_packet_alloc(void);

/**
 * @brief Frees an mcumgr net_buf
 *
 * @param nb    The net_buf to free.
 */
void smp_packet_free(struct net_buf *nb);

/**
 * @brief Decodes, encodes, and transmits SMP packets.
 */
struct smp_streamer {
	struct smp_transport *smpt;
	struct cbor_nb_reader *reader;
	struct cbor_nb_writer *writer;

#ifdef CONFIG_MCUMGR_SMP_VERBOSE_ERR_RESPONSE
	const char *rc_rsn;
#endif
};

/**
 * @brief Processes a single SMP request packet and sends all corresponding responses.
 *
 * Processes all SMP requests in an incoming packet.  Requests are processed
 * sequentially from the start of the packet to the end.  Each response is sent
 * individually in its own packet.  If a request elicits an error response,
 * processing of the packet is aborted.  This function consumes the supplied
 * request buffer regardless of the outcome.
 *
 * @param streamer	The streamer providing the required SMP callbacks.
 * @param req		The request packet to process.
 *
 * @return 0 on success, #mcumgr_err_t code on failure.
 */
int smp_process_request_packet(struct smp_streamer *streamer, void *req);

#if defined(CONFIG_MCUMGR_TRANSPORT_FORWARD_TREE)
int smp_ft_process_request_packet(struct smp_streamer *streamer, void *vreq);

/**
 * @brief Application hook: a frame arrived from a downstream transport.
 *
 * Called for every frame received from a downstream transport before it is
 * forwarded upstream. The default forwards everything; override it with a strong
 * definition to consume a frame locally instead - e.g. to keep a private
 * management group off the upstream host session.
 *
 * Runs on the MCUmgr transport work queue.
 *
 * @param group	Management group id, host-endian.
 * @param nb	The frame. Valid for the duration of the call; the forward tree's
 *		common cleanup releases it, so an implementation must not free it.
 *
 * @return true to consume the frame locally, false to forward it upstream.
 */
bool smp_ft_upstream_intercept(uint16_t group, struct net_buf *nb);

/**
 * @brief Application hook: a frame is about to go out to a downstream transport.
 *
 * Called for every frame this node forwards downstream, before the transport
 * takes it. Purely observational - it is how an application learns that a
 * downstream link is carrying host traffic, so that something originated locally
 * (a keepalive, say) can keep off a half-duplex wire that is already busy.
 *
 * Runs on the MCUmgr transport work queue.
 *
 * @param group	Management group id, host-endian.
 * @param nb	The frame, still owned by the caller. Do not modify or free it.
 */
void smp_ft_downstream_forwarded(uint16_t group, struct net_buf *nb);
#endif

/**
 * @brief Appends an "err" response
 *
 * This appends an err response to a pending outgoing response which contains a
 * result code for a specific group. Note that error codes are specific to the
 * command group they are emitted from).
 *
 * @param zse		The zcbor encoder to use.
 * @param group		The group which emitted the error.
 * @param ret		The command result code to add.
 *
 * @return true on success, false on failure (memory error).
 */
bool smp_add_cmd_err(zcbor_state_t *zse, uint16_t group, uint16_t ret);

#if defined(CONFIG_MCUMGR_SMP_SUPPORT_ORIGINAL_PROTOCOL)
/** @typedef	smp_translate_error_fn
 * @brief	Translates a SMP version 2 error response to a legacy SMP version 1 error code.
 *
 * @param ret	The SMP version 2 group error value.
 *
 * @return	#enum mcumgr_err_t Legacy SMP version 1 error code to return to client.
 */
typedef int (*smp_translate_error_fn)(uint16_t err);
#endif

#ifdef __cplusplus
}
#endif

#endif /* H_SMP_ */
