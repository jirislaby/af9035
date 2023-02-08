#include <assert.h>
#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <arpa/inet.h>

#include "common.h"

#define pr_cont(...)			do {		\
	if (verbosity >= 2)				\
		printf(__VA_ARGS__);			\
} while (0)
#define dev_dbg(dev, ...)		pr_cont(__VA_ARGS__)
#define dump_data_limitedX(P, B, C, L)	do {		\
	if (verbosity >= 2)				\
		dump_data_limited(P, B, C, L);		\
} while (0)

#define be32_to_cpu(x)		ntohl(x)
#define WARN_ON(x)		do { fflush(stdout); assert(!(x)); } while (0)

struct af9035_packet {
	union {
		struct {
			uint8_t FA;
			/*
			 * 4 bits: size_bot
			 * 4 bits: seq_bot
			 * 5 bits: seq_top
			 * 1 bits: FLIP
			 * 2 bits: size_top
			 * 1 bit: REAL
			 * 1 bit: AUDIO
			 * 1 bit: SYNC
			 * 1 bit: ??
			 * 4 bits: synch
			 */
			uint8_t res[3];
		};
		uint32_t val;
	};
	uint8_t data[];
} __attribute__((packed));

#define HEADER_FA(val)		(((val) & 0xff000000) >> 24)
#define HEADER_WORDS(val)	(((val) & 0x00f00000) >> 20)
#define HEADER_SEQ_BOT(val)	(((val) & 0x000f0000) >> 16)
#define  HEADER_SEQ_BOT_BITS	4
#define HEADER_SEQ_TOP(val)	(((val) & 0x0000f800) >> 11)
#define HEADER_FLIP(val)	(((val) & 0x00000400) >> 10)
#define HEADER_HISIZE(val)	(((val) & 0x00000300) >>  8)
#define HEADER_REAL(val)	(((val) & 0x00000080) >>  7)
#define HEADER_AUDIO(val)	(((val) & 0x00000040) >>  6)
#define HEADER_SYNC(val)	(((val) & 0x00000020) >>  5)
#define HEADER_unk1(val)	(((val) & 0x00000010) >>  4)
#define HEADER_XXX_MASK(val)	(((val) & 0x000000f0) >>  4)
#define HEADER_SY(val)		(((val) & 0x0000000f) >>  0)

static inline uint16_t HEADER_SEQ(const uint32_t val)
{
	uint16_t seq = HEADER_SEQ_TOP(val);

	seq <<= HEADER_SEQ_BOT_BITS;
	seq |= HEADER_SEQ_BOT(val);

	WARN_ON(seq & 1);

	return seq >> 1;
}

static inline unsigned HEADER_SIZE(const uint32_t val)
{
	return (HEADER_HISIZE(val) << 6) | (HEADER_WORDS(val) << 2);
}

static void copy_to_local(struct af9035 *af9035, const uint8_t *isoc_data,
			  unsigned to_copy)
{
	memmove(af9035->packet_buf + af9035->packet_buf_ptr, isoc_data,
		to_copy);
	af9035->packet_buf_ptr += to_copy;
}

static void demux_one(struct af9035 *af9035,
			  const struct af9035_packet *pkt)
{
	bool dump_video = false;
	uint32_t val = be32_to_cpu(pkt->val);
	unsigned to_read = HEADER_SIZE(val);

	if (HEADER_unk1(val)) {
		pr_cont(" U");
	} else if (HEADER_SYNC(val)) {
		af9035->ssize += to_read;
		pr_cont(" S");
		dump_video = true;
		if (!af9035->synced) {
			af9035->asize = 0;
			af9035->ssize = 0;
			af9035->vsize = 0;
		}
		af9035->synced = true;
	} else if (HEADER_AUDIO(val)) {
		pr_cont(" A");
		if (af9035->synced && audio_fd >= 0)
			write(audio_fd, pkt->data, to_read);
		af9035->asize += to_read;
	} else if (to_read == 180) {
		pr_cont(" V");
		if (af9035->synced && video_fd >= 0 &&
		    af9035->vsize + (size_t)to_read < sizeof(af9035->videobuf))
			memcpy(&af9035->videobuf[af9035->vsize],
			       pkt->data, to_read);
		af9035->vsize += to_read;
	} else
		pr_cont(" _");

	pr_cont(" vsize=%6u asize=%8u", af9035->vsize, af9035->asize);
	dump_data_limitedX("payl", af9035->packet_buf, to_read, 12);

	if (dump_video && af9035->vsize) {
		pr_cont("\n");
		dev_dbg(&af9035->intf->dev, " ");
		if (af9035->synced && video_fd >= 0) {
			pr_cont("dumping; ");
			write(video_fd, af9035->videobuf, sizeof(af9035->videobuf));
			memset(af9035->videobuf, 0, sizeof(af9035->videobuf));
		}
		pr_cont("vsize=%u ssize=%u\n", af9035->vsize, af9035->ssize);
		af9035->vsize = 0;
		af9035->ssize = 0;
	}
}

static unsigned demux_packet(struct af9035 *af9035, const uint8_t *isoc_data,
			     uint32_t len)
{
	const struct af9035_packet *pkt;
	unsigned to_read, had_in_buf = af9035->packet_buf_ptr;
	uint32_t val;

	dev_dbg(&af9035->intf->dev, " off=%8zx len=%4u buf_ptr=%3u",
		af9035->off, len, af9035->packet_buf_ptr);

	if (af9035->packet_buf_ptr) {
		bool done = true;
		pkt = (void *)af9035->packet_buf;
		to_read = HEADER_SIZE(be32_to_cpu(pkt->val)) + sizeof(*pkt) -
			af9035->packet_buf_ptr;

		if (len < to_read) {
			done = false;
			to_read = len;
		}

		copy_to_local(af9035, isoc_data, to_read);
		if (!done)
			return to_read;

		af9035->packet_buf_ptr = 0;
	} else
		pkt = (void *)isoc_data;

	if (len < sizeof(*pkt)) {
		copy_to_local(af9035, isoc_data, len);
		WARN_ON(1);
		return len;
	}

	if (pkt->FA != 0xfa) {
		pr_cont(" BAD(%.8x)\n", be32_to_cpu(pkt->val));
		return 4;
	}

	val = be32_to_cpu(pkt->val);
	to_read = HEADER_SIZE(val);
	pr_cont(" hdr=%.8x SEQ=%3u/%2x SY=%u %c%c%c%c%c len=%3u",
		val,
		HEADER_SEQ(val), HEADER_SEQ(val),
		HEADER_SY(val),
		HEADER_FLIP(val) ? 'F' : '_',
		HEADER_REAL(val) ? 'R' : '_',
		HEADER_AUDIO(val) ? 'A' : '_',
		HEADER_SYNC(val) ? 'S' : '_',
		HEADER_unk1(val) ? 'U' : '_',
		to_read);

	if (len < sizeof(*pkt) + to_read) {
		copy_to_local(af9035, isoc_data, len);
		pr_cont(" STRAY need=%zu\n", sizeof(*pkt) + to_read);
		return len;
	}

	if (to_read) {
		/* BUG in FW? */
		if (to_read == 4 && pkt->data[0] == 0xfa) {
			pr_cont(" BAD -- retrying\n");
			copy_to_local(af9035, pkt->data, 4);
		} else
			demux_one(af9035, pkt);
	}

	pr_cont(" sub=%3u advancing=%3zu\n", had_in_buf,
		sizeof(*pkt) + to_read - had_in_buf);

	return sizeof(*pkt) + to_read - had_in_buf;
}

void demux(struct af9035 *af9035, const uint8_t *isoc_data, uint32_t len)
{
	unsigned size;

	while (len) {
		size = demux_packet(af9035, isoc_data, len);
		af9035->off += size;
		isoc_data += size;
		len -= size;
	}
}
