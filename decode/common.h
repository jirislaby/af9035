#ifndef COMMON_H
#define COMMON_H

#include <stdbool.h>
#include <stdint.h>

#define PACKET_SIZE	184

#define ARRAY_SIZE(x)	(sizeof(x) / sizeof(*(x)))
#define min(a, b)	((a) < (b) ? (a) : (b))

struct af9035 {
	unsigned packet_buf_ptr;
	uint8_t packet_buf[PACKET_SIZE];
	uint8_t videobuf[720*576/2*2];

	unsigned vsize, asize, ssize;
	bool synced;

	unsigned pkt;
	off_t off;
};

extern int video_fd, audio_fd;
extern unsigned verbosity;

void dump_data(const char *prefix, const uint8_t *data, unsigned len);
void dump_data_limited(const char *prefix, const uint8_t *data, unsigned len,
		       unsigned limit);

void demux(struct af9035 *af9035, const uint8_t *isoc_data, uint32_t len);

#endif
