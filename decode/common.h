#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>

extern int video_fd, audio_fd;
extern unsigned pkt_lim;

void dump_data(const char *prefix, const uint8_t *data, unsigned len);
void dump_data_limited(const char *prefix, const uint8_t *data, unsigned len,
		       unsigned limit);

void demux(const uint8_t *isoc_data, uint32_t len);

#endif
