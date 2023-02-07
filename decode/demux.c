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

#define be32_to_cpu(x)	ntohl(x)
#define pr_cont		printf

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

#define HEADER_FA(hdr)		(((hdr)->val & 0xff000000) >> 24)
#define HEADER_WORDS(hdr)	(((hdr)->val & 0x00f00000) >> 20)
#define HEADER_SEQ_BOT(hdr)	(((hdr)->val & 0x000f0000) >> 16)
#define  HEADER_SEQ_BOT_BITS	4
#define HEADER_SEQ_TOP(hdr)	(((hdr)->val & 0x0000f800) >> 11)
#define HEADER_FLIP(hdr)	(((hdr)->val & 0x00000400) >> 10)
#define HEADER_HISIZE(hdr)	(((hdr)->val & 0x00000300) >>  8)
#define HEADER_REAL(hdr)	(((hdr)->val & 0x00000080) >>  7)
#define HEADER_AUDIO(hdr)	(((hdr)->val & 0x00000040) >>  6)
#define HEADER_SYNC(hdr)	(((hdr)->val & 0x00000020) >>  5)
#define HEADER_unk1(hdr)	(((hdr)->val & 0x00000010) >>  4)
#define HEADER_XXX_MASK(hdr)	(((hdr)->val & 0x000000f0) >>  4)
#define HEADER_SY(hdr)		(((hdr)->val & 0x0000000f) >>  0)

static inline uint16_t HEADER_SEQ(const struct af9035_packet *hdr)
{
	uint16_t seq = HEADER_SEQ_TOP(hdr);

	seq <<= HEADER_SEQ_BOT_BITS;
	seq |= HEADER_SEQ_BOT(hdr);

	if (seq & 1)
		printf(" ODD SEQ");

	return seq >> 1;
}

static inline unsigned HEADER_SIZE(const struct af9035_packet *hdr)
{
	return (HEADER_HISIZE(hdr) << 6) | (HEADER_WORDS(hdr) << 2);
}

static void *get_data(struct af9035 *af9035, const uint8_t **isoc_data,
		      uint32_t *len, unsigned requested)
{
	unsigned to_copy;

	if (af9035->packet_buf_ptr >= requested) {
		return af9035->packet_buf;
	} else if (af9035->packet_buf_ptr) {
		to_copy = min(requested, *len);
		memcpy(af9035->packet_buf + af9035->packet_buf_ptr, *isoc_data,
		       to_copy);
		af9035->off += to_copy;
		af9035->packet_buf_ptr = 0;

		*len -= to_copy;
		*isoc_data += to_copy;

		return af9035->packet_buf;
	}

	return NULL;
}

static void demux_one(struct af9035 *af9035, struct af9035_packet *header)
{
	bool dump_video = false;
	ssize_t to_read;

	printf("off=%8zx", af9035->off);

	if (header->FA != 0xfa) {
		printf(" BAD(%.8x)\n", be32_to_cpu(header->val));
		return;
	}

retry:
	header->val = be32_to_cpu(header->val);

	to_read = HEADER_SIZE(header);
	pr_cont(" hdr=%.8x SEQ=%3u/%2x SY=%u %c%c%c%c%c len=%3zd",
		header->val,
		HEADER_SEQ(header), HEADER_SEQ(header),
		HEADER_SY(header),
		HEADER_FLIP(header) ? 'F' : '_',
		HEADER_REAL(header) ? 'R' : '_',
		HEADER_AUDIO(header) ? 'A' : '_',
		HEADER_SYNC(header) ? 'S' : '_',
		HEADER_unk1(header) ? 'U' : '_',
		to_read);

	if (to_read) {
		/* BUG in FW? */
		if (to_read == 4 && header->data[0] == 0xfa) {
			printf(" BAD -- retrying\noff=%8zx", af9035->off);
			memmove(header, af9035->packet_buf, sizeof(*header));
			goto retry;
		}

		if (HEADER_unk1(header)) {
			printf(" U");
		} else if (HEADER_SYNC(header)) {
			af9035->ssize += to_read;
			printf(" S");
			dump_video = true;
			if (!af9035->synced) {
				af9035->asize = 0;
				af9035->ssize = 0;
				af9035->vsize = 0;
			}
			af9035->synced = true;
		} else if (HEADER_AUDIO(header)) {
			printf(" A");
			if (af9035->synced && audio_fd >= 0)
				write(audio_fd, af9035->packet_buf, to_read);
			af9035->asize += to_read;
		} else if (to_read == 180) {
			printf(" V");
			if (af9035->synced && video_fd >= 0 &&
			    af9035->vsize + (size_t)to_read < sizeof(af9035->videobuf))
				memcpy(&af9035->videobuf[af9035->vsize],
				       af9035->packet_buf, to_read);
			af9035->vsize += to_read;
		} else
			printf(" _");

		printf(" vsize=%6u asize=%8u", af9035->vsize, af9035->asize);
		dump_data_limited("payl", af9035->packet_buf, to_read, 12);
	}

	pr_cont("\n");

	if (dump_video && af9035->vsize) {
		if (af9035->synced && video_fd >= 0) {
			printf("dumping; ");
			write(video_fd, af9035->videobuf, sizeof(af9035->videobuf));
			memset(af9035->videobuf, 0, sizeof(af9035->videobuf));
		}
		printf("vsize=%u ssize=%u\n", af9035->vsize, af9035->ssize);
		af9035->vsize = 0;
		af9035->ssize = 0;
	}
}

void demux(struct af9035 *af9035, const uint8_t *isoc_data, uint32_t len)
{
	struct af9035_packet *pkt;

	while ((pkt = get_data(af9035, &isoc_data, &len, sizeof(*af9035))))
		demux_one(af9035, pkt);
}
