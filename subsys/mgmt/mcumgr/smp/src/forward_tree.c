/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** SMP Forward Tree - Simple Management Protocol Forward Tree. */

#include <zephyr/sys/byteorder.h>
#include <zephyr/net_buf.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/smp/smp.h>
#include <zephyr/mgmt/mcumgr/smp/smp_client.h>
#include <zephyr/mgmt/mcumgr/transport/smp.h>
#include <assert.h>
#include <string.h>

#include <zcbor_common.h>
#include <zcbor_decode.h>
#include <zcbor_encode.h>

#include <mgmt/mcumgr/transport/smp_internal.h>

#include <zephyr/toolchain.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mcumgr_forward, 4);

#define SMP_FORWARD_TREE_PORT_MASK 0x0f
#define SMP_FORWARD_TREE_PORT_BITS 0x04
#define SMP_FORWARD_TREE_MAX_PORTS 0x10

/*
 * Application hook. The default is inert; an application may override it with a
 * strong definition.
 */

/* Called for every frame this node forwards to a downstream transport, before
 * the transport takes it. Observational only - it exists so an application can
 * see that a downstream link is busy with host traffic. @p group is host-endian.
 */
__weak void smp_ft_downstream_forwarded(uint16_t group, struct net_buf *nb)
{
	ARG_UNUSED(group);
	ARG_UNUSED(nb);
}

#define DT_SMP_FORWARD_INST		DT_INST(0, zephyr_smpmgr_forward)
#define DT_SMP_UPSTREAM			DT_PHANDLE(DT_SMP_FORWARD_INST, upstream)
#define DT_SMP_UPSTREAM_TRANSPORT 	DT_PHANDLE(DT_SMP_UPSTREAM, transport)

struct smp_forward_tree_transport upstream_transport = {
	.dev = DEVICE_DT_GET(DT_SMP_UPSTREAM_TRANSPORT),
	.type = DT_ENUM_IDX(DT_SMP_UPSTREAM, type),
};

#define INST_DOWN_TRANSPORTS_ENTRY(node_id)					\
{										\
	.dev = DEVICE_DT_GET(DT_PHANDLE(node_id, transport)),			\
	.type = DT_ENUM_IDX(node_id, type),					\
},

#define INST_DOWN_TRANSPORTS(node_id, prop, idx)				\
	INST_DOWN_TRANSPORTS_ENTRY(DT_PHANDLE_BY_IDX(node_id, prop, idx))

struct smp_forward_tree_transport downstream_transport[] = {
	DT_FOREACH_PROP_ELEM(DT_SMP_FORWARD_INST, downstream, INST_DOWN_TRANSPORTS)
};

static int smp_read_hdr(const struct net_buf_simple *nb, struct smp_hdr *dst_hdr)
{
	if (nb->len < sizeof(*dst_hdr)) {
		return MGMT_ERR_EINVAL;
	}

	memcpy(dst_hdr, nb->data, sizeof(*dst_hdr));
	dst_hdr->nh_len = sys_be16_to_cpu(dst_hdr->nh_len);
	dst_hdr->nh_group = sys_be16_to_cpu(dst_hdr->nh_group);

	return 0;
}

static int smp_ft_read_fwd(const struct net_buf_simple *nb,
			   struct smp_forward_tree *dst_fwd)
{
	uint64_t tmp_ft;

	if (nb->len < sizeof(struct smp_forward_tree)) {
		return MGMT_ERR_EINVAL;
	}

	memcpy(&tmp_ft, nb->data + (nb->len - sizeof(uint64_t)), sizeof(uint64_t));
	tmp_ft = sys_be64_to_cpu(tmp_ft);
	memcpy(dst_fwd, &tmp_ft, sizeof(uint64_t));

	return 0;
}

/* Write the trailer back to the end of the frame. smp_ft_read_fwd() read it with
 * sys_be64_to_cpu(), so it goes back big-endian.
 */
static void smp_ft_write_fwd(struct net_buf *nb, const struct smp_forward_tree *fwd)
{
	uint64_t tmp_ft;

	memcpy(&tmp_ft, fwd, sizeof(uint64_t));
	tmp_ft = sys_cpu_to_be64(tmp_ft);
	memcpy(nb->data + (nb->len - sizeof(uint64_t)), &tmp_ft, sizeof(uint64_t));
}

/* The port at a given position along the path. Position 0 is the last hop, the one
 * nearest the destination; the path is not consumed as the frame travels, which is
 * what lets the trailer serve as a return address.
 */
static uint8_t smp_ft_path_port(const struct smp_forward_tree *fwd, uint8_t index)
{
	return (fwd->port >> (index * SMP_FORWARD_TREE_PORT_BITS)) & SMP_FORWARD_TREE_PORT_MASK;
}

/* Which downstream port a frame arrived on, or -1 for anything else (the upstream
 * transport, or a transport this node does not forward for).
 */
static int smp_ft_port_of(const struct device *dev)
{
	for (int i = 0; i < ARRAY_SIZE(downstream_transport); ++i) {
		if (downstream_transport[i].dev == dev) {
			return i;
		}
	}

	return -1;
}

/* Take the routing trailer off a frame that has reached its destination and clear
 * the routing flags, leaving a plain SMP frame for local processing.
 */
static void smp_ft_strip_trailer(struct net_buf *req, struct net_buf_simple *clone,
				 struct smp_hdr *req_hdr)
{
	net_buf_simple_remove_mem(clone, sizeof(struct smp_forward_tree));

	req_hdr->nh_flags &= ~SMP_HDR_FLAG_ROUTING_MASK;
	req_hdr->nh_len -= sizeof(struct smp_forward_tree);

	/* Back to big-endian for the wire/buffer. */
	req_hdr->nh_len = sys_cpu_to_be16(req_hdr->nh_len);
	req_hdr->nh_group = sys_cpu_to_be16(req_hdr->nh_group);

	net_buf_simple_push_mem(clone, req_hdr, sizeof(struct smp_hdr));

	/* Sync the modified length back to the original buffer. */
	req->len = clone->len;
}

int smp_ft_forward_downstream(struct smp_forward_tree *req_fwd, void *vreq)
{
	uint8_t port = smp_ft_path_port(req_fwd, req_fwd->hop - 1);
	struct smp_transport *smpt = NULL;

	LOG_DBG("port: %d", port);

	if (port >= SMP_FORWARD_TREE_MAX_PORTS
	||  port >= ARRAY_SIZE(downstream_transport)) {
		LOG_ERR("No downstream port [%d] on this node", port);
		smp_packet_free(vreq);
		return MGMT_ERR_EINVAL;
	}

	smpt = smp_get_smpt(downstream_transport[port].dev);
	if (smpt == NULL) {
		LOG_ERR("Transport index [%d] not recognized", port);
		for (int i = 0; i < ARRAY_SIZE(downstream_transport); ++i) {
			LOG_DBG("transport[%d]: %s", i, downstream_transport[i].dev->name);
		}

		smp_packet_free(vreq);
		return MGMT_ERR_EINVAL;
	}

	/* One hop spent, one hop to retrace. Their sum is the path length and does
	 * not change; the path itself is left alone.
	 */
	--req_fwd->hop;
	++req_fwd->up;

	smp_ft_write_fwd(vreq, req_fwd);

	return smpt->functions.output(smpt->dev, vreq);
}

/* Hand a frame that has arrived at its destination to the local SMP layer.
 *
 * Consumes the buffer only on success; on error it is left to the caller, which
 * is what the common cleanup at the end of smp_ft_process_request_packet() is
 * for. A response frame ends up in smp_client_single_response() from there, so a
 * node that originated a request needs no callback of its own to get the answer.
 */
static int smp_ft_process_local(struct smp_streamer *streamer, struct net_buf *req,
				bool *consumed)
{
	int rc = smp_process_request_packet(streamer, req);

	if (rc == 0) {
		*consumed = true;
	}

	return rc;
}

/* Rules for a frame that arrived from the upstream transport: it is travelling
 * away from the host, and either this node is on its way or it is the addressee.
 */
static int smp_ft_process_downward(struct smp_streamer *streamer, struct net_buf *req,
				   struct net_buf_simple *clone, struct smp_hdr *req_hdr,
				   bool *consumed)
{
	struct smp_forward_tree req_fwd = { 0 };

	if ((req_hdr->nh_flags & SMP_HDR_FLAG_FORWARD_TREE) == 0) {
		/* Not routed at all: the host is talking to this node directly. */
		return smp_ft_process_local(streamer, req, consumed);
	}

	if (smp_ft_read_fwd(clone, &req_fwd)) {
		LOG_ERR("Frame is flagged as routed but carries no trailer");
		return MGMT_ERR_ECORRUPT;
	}

	LOG_DBG("hop: %u, up: %u", req_fwd.hop, req_fwd.up);

	if (req_fwd.hop + req_fwd.up > SMP_FORWARD_TREE_MAX_HOPS) {
		LOG_ERR("Path of %u hops is longer than this trailer can address",
			req_fwd.hop + req_fwd.up);
		return MGMT_ERR_EINVAL;
	}

	if (req_fwd.hop > 0) {
		LOG_DBG("forward downstream");
		/* Before the transport takes the buffer, which consumes it either
		 * way.
		 */
		smp_ft_downstream_forwarded(req_hdr->nh_group, req);
		*consumed = true;
		return smp_ft_forward_downstream(&req_fwd, req);
	}

	/* This node is the addressee. Whether the trailer stays on depends on who
	 * has to route the response, and `up` is what says so: with `up` at zero
	 * nobody forwarded this frame, so the peer is the host and the response
	 * must look exactly like it always has. With `up` above zero the response
	 * has to climb back through the nodes that forwarded the request, and the
	 * trailer left in place is the address it climbs by - echoed back by
	 * smp.c under CONFIG_MCUMGR_SMP_ROUTING_TRAILER_ECHO, the same way a leaf
	 * does it.
	 */
	if (req_fwd.up == 0) {
		smp_ft_strip_trailer(req, clone, req_hdr);
	}

	return smp_ft_process_local(streamer, req, consumed);
}

/* Rules for a frame that arrived from a downstream port: it is travelling towards
 * the host, and the trailer says how far it still has to climb.
 */
static int smp_ft_process_upward(struct smp_streamer *streamer, struct net_buf *req,
				 struct net_buf_simple *clone, struct smp_hdr *req_hdr,
				 int arrival_port, bool *consumed)
{
	struct smp_forward_tree req_fwd = { 0 };
	struct smp_transport *smpt;

	if ((req_hdr->nh_flags & SMP_HDR_FLAG_FORWARD_TREE) == 0) {
		/* No return address, so nobody up here asked for this. The forward
		 * tree only sends up what it knows was requested.
		 */
		LOG_WRN("Dropping an unaddressed frame from port %d", arrival_port);
		return MGMT_ERR_EINVAL;
	}

	if (smp_ft_read_fwd(clone, &req_fwd)) {
		LOG_ERR("Frame is flagged as routed but carries no trailer");
		return MGMT_ERR_ECORRUPT;
	}

	LOG_DBG("hop: %u, up: %u, port: %d", req_fwd.hop, req_fwd.up, arrival_port);

	if (req_fwd.up == 0 || req_fwd.hop >= SMP_FORWARD_TREE_MAX_HOPS) {
		LOG_WRN("Dropping a frame with nothing left to retrace");
		return MGMT_ERR_EINVAL;
	}

	/* The path is checked against reality at every step up: the nibble for
	 * this hop must be the port the frame actually came in on. It is what
	 * stops a device downstream from making up a route through this node.
	 */
	if (smp_ft_path_port(&req_fwd, req_fwd.hop) != arrival_port) {
		LOG_WRN("Path says port %u, frame came in on port %d - dropping",
			smp_ft_path_port(&req_fwd, req_fwd.hop), arrival_port);
		return MGMT_ERR_EINVAL;
	}

	++req_fwd.hop;
	--req_fwd.up;
	smp_ft_write_fwd(req, &req_fwd);

	if (req_fwd.up > 0) {
		/* Still below the frame's origin: pass it on untouched. */
		smpt = smp_get_smpt(upstream_transport.dev);
		if (smpt == NULL) {
			LOG_ERR("No SMP transport bound to the upstream device");
			return MGMT_ERR_ECORRUPT;
		}

		LOG_DBG("forward upstream: %s", smpt->dev->name);
		*consumed = true;
		/* The transport's output() callback always consumes the buf. */
		return smpt->functions.output(smpt->dev, req);
	}

	/* Home. Either this node originated the frame, or the host did and this
	 * node is the root.
	 */
	if (req_hdr->nh_flags & SMP_HDR_FLAG_FT_NODE_ORIGIN) {
		LOG_DBG("locally originated frame is home");
		smp_ft_strip_trailer(req, clone, req_hdr);
		return smp_ft_process_local(streamer, req, consumed);
	}

	smpt = smp_get_smpt(upstream_transport.dev);
	if (smpt == NULL) {
		LOG_ERR("No SMP transport bound to the upstream device");
		return MGMT_ERR_ECORRUPT;
	}

	/* The host gets back exactly what it would get from a node with no
	 * forward tree at all: no trailer, no flags.
	 */
	smp_ft_strip_trailer(req, clone, req_hdr);

	LOG_DBG("deliver upstream: %s", smpt->dev->name);
	*consumed = true;
	return smpt->functions.output(smpt->dev, req);
}

/**
 * Routes one incoming SMP packet.
 *
 * The side the frame arrived on selects the rules. A frame from the upstream
 * transport is heading away from the host: this node either forwards it out the
 * port the trailer names, or is itself the addressee and processes it locally. A
 * frame from a downstream port is heading back: this node retraces one hop of the
 * path in the trailer and either passes it further up, keeps it because it
 * originated the request, or hands it to the host because it is the root.
 *
 * A frame from downstream with no trailer is dropped. The forward tree passes up
 * only what it can see was asked for.
 *
 * This function consumes the supplied request buffer regardless of the outcome.
 *
 * @param streamer	The streamer to use for reading, writing, and transmitting.
 * @param vreq		A buffer containing the request packet.
 *
 * @return 0 on success;
 *         MGMT_ERR_ECORRUPT if the buffer does not hold one complete SMP frame,
 *         or another MGMT_ERR_[...] code on failure.
 */
int smp_ft_process_request_packet(struct smp_streamer *streamer, void *vreq)
{
	struct smp_hdr req_hdr = { 0 };
	struct net_buf_simple clone = { 0 };
	struct net_buf *req = vreq;
	int arrival_port;
	int rc = 0;
	bool consumed = false;

	LOG_DBG("incoming forward request...");

	/*
	 * This clone will copy the size and max length. The pointers will
	 * reference the real data. This means that any change in the data in
	 * the cloned data will change the real data.
	 */
	net_buf_simple_clone(&req->b, &clone);

	do {
		/* Read the management header. */
		rc = smp_read_hdr(&clone, &req_hdr);
		if (rc != 0) {
			rc = MGMT_ERR_ECORRUPT;
			LOG_ERR("Frame is too short to hold an SMP header");
			break;
		}

		LOG_DBG("Group ID: %04x", req_hdr.nh_group);
		LOG_DBG("Seq Num:  %02x", req_hdr.nh_seq);
		LOG_DBG("CMD ID:   %02x", req_hdr.nh_id);
		LOG_DBG("OP:       %02x", req_hdr.nh_op);
		LOG_DBG("Flags:    %02x", req_hdr.nh_flags);
		LOG_DBG("Len:      %04x", req_hdr.nh_len);

		/* Does buffer contain only one message and it is complete? */
		net_buf_simple_pull(&clone, sizeof(struct smp_hdr));

		if (clone.len != req_hdr.nh_len) {
			rc = MGMT_ERR_ECORRUPT;
			LOG_ERR("Frame holds %u payload bytes, header says %u",
				clone.len, req_hdr.nh_len);
			break;
		}

		arrival_port = smp_ft_port_of(streamer->smpt->dev);
		if (streamer->smpt->dev == upstream_transport.dev) {
			rc = smp_ft_process_downward(streamer, req, &clone, &req_hdr,
						     &consumed);
		} else if (arrival_port >= 0) {
			rc = smp_ft_process_upward(streamer, req, &clone, &req_hdr,
						   arrival_port, &consumed);
		} else {
			LOG_ERR("Frame from %s, which is not a port of this node",
				streamer->smpt->dev->name);
			rc = MGMT_ERR_EINVAL;
		}
	} while (0);

	LOG_DBG("finish forward request...");

	if (!consumed) {
		smp_free_buf(req, streamer->smpt);
	}

	return rc;
}
