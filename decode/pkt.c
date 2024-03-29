#include <assert.h>
#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <pcap/pcap.h>

#include "common.h"

struct usb_pkt {
	uint64_t id;
	uint8_t type;
	uint8_t ttype;
	uint8_t ep_dir;
	uint8_t dev;
	uint16_t bus;
	uint8_t setup_req;
	uint8_t data_pres;
	uint64_t sec;
	uint32_t usec;
	uint32_t stat;
	uint32_t ulen;
	uint32_t dlen;
	uint32_t iso_err;
	uint32_t isoc_nr;
	uint32_t intval;
	uint32_t start_frm;
	uint32_t flags_copy;
	uint32_t isoc_nr2;
	unsigned char data[];
} __attribute__((packed));

struct isoc_desc {
	int32_t stat;
	uint32_t off;
	uint32_t len;
	uint32_t pad;
} __attribute__((packed));

struct af9035_hw {
	uint8_t len;
	union {
		struct {
			uint8_t mbox;
			uint8_t cmd;
			uint8_t seq;
			unsigned char data[];
		} wr;
		struct {
			uint8_t seq;
			uint8_t sta;
			unsigned char data[];
		} rd;
	};
} __attribute__((packed));

#define CMD_MEM_RD                  0x00
#define CMD_MEM_WR                  0x01
#define CMD_I2C_RD                  0x02
#define CMD_I2C_WR                  0x03
#define CMD_IR_GET                  0x18
#define CMD_FW_DL                   0x21
#define CMD_FW_QUERYINFO            0x22
#define CMD_FW_BOOT                 0x23
#define CMD_FW_DL_BEGIN             0x24
#define CMD_FW_DL_END               0x25
#define CMD_FW_SCATTER_WR           0x29
#define CMD_GENERIC_I2C_RD          0x2a
#define CMD_GENERIC_I2C_WR          0x2b

static const struct {
	uint8_t cmd;
	const char *desc;
} cmds[] = {
	{ CMD_MEM_RD        , "MEMRD" },
	{ CMD_MEM_WR        , "MEMWR" },
	{ CMD_I2C_RD        , "I2CRD" },
	{ CMD_I2C_WR        , "I2CWR" },
	{ CMD_IR_GET        , "IRGET" },
	{ CMD_FW_DL         , "FWDL" },
	{ CMD_FW_QUERYINFO  , "QRYFW" },
	{ CMD_FW_BOOT       , "BOOT" },
	{ CMD_FW_DL_BEGIN   , "DLBEG" },
	{ CMD_FW_DL_END     , "DLEND" },
	{ CMD_FW_SCATTER_WR , "SCWR" },
	{ CMD_GENERIC_I2C_RD, "I2CRD" },
	{ CMD_GENERIC_I2C_WR, "I2CWR" },
};

static int dump_isoc = -1, dump_fw = -1;
static bool hex_data;
unsigned verbosity;
int video_fd = -1, audio_fd = -1;

void dump_data(const char *prefix, const uint8_t *data, unsigned len)
{
	printf(" %s=", prefix);
	for (unsigned a = 0; a < len; a++)
		printf("%s%.2x", hex_data ? ", 0x" : " ", data[a]);
}

void dump_data_limited(const char *prefix, const uint8_t *data, unsigned len,
		       unsigned limit)
{
	dump_data(prefix, data, min(len, limit));
	if (len > limit)
		printf(" (%u more)", len - limit);
}

static const char *get_cmd_desc(uint8_t cmd)
{
	for (unsigned a = 0; a < ARRAY_SIZE(cmds); a++)
		if (cmds[a].cmd == cmd)
			return cmds[a].desc;

	return "UNK";
}

static const char *get_reg_name(uint32_t reg)
{
	static char buf[32];
	const char *reg_str;

	switch (reg) {
	case 0x1222:
		return "CHIP_VER";
	case 0x384f:
		return "PRECHIP_VER";
	case 0x417f:
		return "PRE_EN_CLK?";
	case 0x42f5 ... 0x42f5+256-1:
		reg_str = "EEPROM";
		reg -= 0x42f5;
		break;
	case 0xd800:
		return "DEMOD_CLOCK";
	case 0xd81a:
		return "EN_CLK";
	case 0xf103:
		return "I2Cm13_CLK_SPD";
	case 0xf641:
		return "TUNER_TYPE";
	case 0xf6a7:
		return "I2Cm2_CLK_SPD";
	case 0xd827 ... 0xd829:
	case 0xd830 ... 0xd832:
	case 0xd8fd:
		reg_str = "DEM_AF9033";
		break;
	case 0xd8b3 ... 0xd8e1:
		reg_str = "TUN_MXL";
		break;
	case 0xdd11:
		return "dual_mode??";
	case 0xdd13:
		return "af9035_init";
	case 0xdd88:
		return "frame_sizeL";
	case 0xdd0c:
		return "packet_size";
	case 0x800000 ... 0x80ffff:
		if (reg == 0x800000 || reg == 0x80004c || reg == 0x80fb24)
			reg_str = "SLEEP";
		else
			reg_str = "TUN_IT9135";
		reg -= 0x800000;
		break;
	default:
		reg_str = "UNK";
		break;
	}

	sprintf(buf, "%s(%.6x)", reg_str, reg);

	return buf;
}

static inline uint32_t get_reg(uint8_t mbox, const uint8_t reg_arr[static 2])
{
	uint32_t reg = mbox;

	reg <<= 8;
	reg |= reg_arr[0];
	reg <<= 8;
	reg |= reg_arr[1];

	return reg;
}

static bool handle_wr_cmd(const struct af9035_hw *af, const uint8_t *data,
		unsigned data_len)
{
	switch (af->wr.cmd) {
	case CMD_MEM_RD: {
		uint32_t reg = get_reg(af->wr.mbox, &data[4]);
		printf(" len=%2u reg=%.6x/%-20s", data[0], reg,
		       get_reg_name(reg));
		return true;
	}
	case CMD_MEM_WR: {
		uint32_t reg = get_reg(af->wr.mbox, &data[4]);
		printf(" len=%2u reg=%.6x/%-20s", data[0], reg,
		       get_reg_name(reg));
		dump_data("data", &data[6], data[0]);
		return true;
	}
	case CMD_GENERIC_I2C_RD:
		printf(" len=%2u bus=%.2x addr=%.2x%s", data[0], data[1],
				data[2] >> 1, data[2] & 1 ? "" : "!");
		return true;
	case CMD_GENERIC_I2C_WR:
		printf(" len=%2u bus=%.2x addr=%.2x%s", data[0], data[1],
				data[2] >> 1, data[2] & 1 ? "!" : "");
		dump_data("data", &data[3], data[0]);
		return true;

	case CMD_FW_QUERYINFO:
		printf(" start=%u", data[0]);
		return true;

	case CMD_FW_DL:
		printf(" len=%u", data_len);
		dump_data_limited("data", data, data_len, 20);
		if (dump_fw >= 0)
			write(dump_fw, data, data_len);
		return true;

	case CMD_FW_DL_BEGIN:
	case CMD_FW_DL_END:
		return true;
	};

	return false;
}

static void handle_af9035(uint32_t sec, uint32_t usec,
			  const struct af9035_hw *af,
			  bool in, uint8_t ep)
{
	const uint8_t *raw = (const void *)af;
	uint16_t csum = raw[af->len - 1];
	unsigned int start = 3;
	bool do_dump_data = true;

	csum <<= 8;
	csum |= raw[af->len];

	printf("\t");

	if (verbosity)
		printf("%4u.%.4u", sec, usec / 100);

	printf(" BULK ep=%u %s len=%2u ", ep, in ? " IN" : "OUT", af->len);

	if (verbosity)
		printf("csum=%4x", csum);

	if (in) {
		if (verbosity)
			printf(" seq=%3u", af->rd.seq);
		printf(" sta=%.2x", af->rd.sta);
	} else {
		start++;
		if (verbosity)
			printf(" seq=%3u", af->wr.seq);
		printf(" mbox=%.2x cmd=%5s", af->wr.mbox,
				get_cmd_desc(af->wr.cmd));
		do_dump_data = !handle_wr_cmd(af, &raw[start],
				af->len - 1 - start);
	}

	if (do_dump_data)
		dump_data("data", &raw[start], af->len - 1 - start);

	puts("");
}

static void handle_bulk(const struct usb_pkt *pkt, uint32_t sec)
{
	bool in = pkt->ep_dir & (1U << 7);
	uint8_t ep = pkt->ep_dir & 0xf;

	if (pkt->dlen)
		handle_af9035(sec, pkt->usec, (const void *)pkt->data, in, ep);
}

static void handle_ctrl(const struct usb_pkt *pkt, uint32_t sec)
{
#if 0
	printf("\t");

	if (verbosity)
		printf("%4u.%.4u", sec, pkt->usec / 100);

	printf(" CTRL");
	puts("");
#else
	(void)pkt; (void)sec;
#endif
}

static void handle_single_isoc(struct af9035 *af9035, const uint8_t *isoc_data,
			       uint32_t len)
{
	if (verbosity >= 2) {
		dump_data_limited("idata", isoc_data, len, 20);
		puts("");
	}
	if (len) {
		demux(af9035, isoc_data, len);
		if (dump_isoc >= 0)
			write(dump_isoc, isoc_data, len);
	}
}

static void handle_isoc(struct af9035 *af9035, const struct usb_pkt *pkt,
			uint32_t sec)
{
	const struct isoc_desc *isoc;
	bool in = pkt->ep_dir & (1U << 7);
	uint8_t ep = pkt->ep_dir & 0xf;
	unsigned a, valid = 0;

	for (a = 0, isoc = (const void *)pkt->data; a < pkt->isoc_nr; a++)
		if (isoc->stat == 0)
			valid++;

	printf("\t");

	if (verbosity)
		printf("%4u.%.4u", sec, pkt->usec / 100);

	printf(" ISOC ep=%u %s isoc_nr=%3u (valid=%u)\n",
			ep, in ? " IN" : "OUT",
			pkt->isoc_nr, valid);

	if (pkt->type == 'S')
		return;

	const void *isoc_data = isoc + pkt->isoc_nr;
	for (a = 0, (const void *)pkt->data; a < pkt->isoc_nr; a++, isoc++) {
		bool eol_needed = false;
		if ((verbosity >= 1 && isoc->stat == 0) || verbosity >= 2) {
			printf("\t\tIDESC %2u[%4d/%5u/%4u]", a, isoc->stat, isoc->off,
			       isoc->len);
			eol_needed = true;
		}
		if (isoc->stat == 0) {
			handle_single_isoc(af9035, isoc_data + isoc->off,
					   isoc->len);
			eol_needed = verbosity > 0 && verbosity < 2;
		}
		if (eol_needed)
			puts("");
	}
}

static const char *get_ttype(const struct usb_pkt *pkt)
{
	switch (pkt->ttype) {
	case 0x00:
		return "ISOC";
	case 0x01:
		return "IRQ ";
	case 0x02:
		return "CTRL";
	case 0x03:
		return "BULK";
	default:
		return "UNKN";
	}
}

static void handle_packet(struct af9035 *af9035, const struct usb_pkt *pkt)
{
	static uint64_t first_sec;

	if (!first_sec)
		first_sec = pkt->sec;

	if (verbosity >= 3) {
		printf("%c ttype=%s, dlen=%3u", pkt->type, get_ttype(pkt), pkt->dlen);
		dump_data_limited("data", pkt->data, pkt->dlen, 30);
		puts("");
	}

	switch (pkt->ttype) {
	case 0x00:
		handle_isoc(af9035, pkt, pkt->sec - first_sec);
		break;
	case 0x02:
		handle_ctrl(pkt, pkt->sec - first_sec);
		break;
	case 0x03:
		handle_bulk(pkt, pkt->sec - first_sec);
		break;
	}
}

static void pcap_cb(u_char *af9035, const struct pcap_pkthdr *,
		    const u_char *bytes)
{
	handle_packet((void *)af9035, (const void *)bytes);
}

int main(int argc, char **argv)
{
	assert(sizeof(struct usb_pkt) == 0x40);

	int o;

	while ((o = getopt(argc, argv, "adfsSvVx")) != -1) {
		switch (o) {
		case 'a':
			audio_fd = open("audio.raw", O_WRONLY | O_CREAT |
					O_TRUNC, 0644);
			break;
		case 'd':
			dump_isoc = open("isoc.raw",
					 O_WRONLY | O_CREAT | O_TRUNC, 0644);
			if (dump_isoc < 0)
				err(1, "open(isoc.raw)");
			break;
		case 'f':
			dump_fw = open("fw.raw",
				       O_WRONLY | O_CREAT | O_TRUNC, 0644);
			if (dump_fw < 0)
				err(1, "open(fw.raw)");
			break;
		case 's':
			verbosity = 1;
			break;
		case 'S':
			verbosity = 2;
			break;
		case 'v':
			verbosity = 3;
			break;
		case 'V':
			video_fd = open("video.raw", O_WRONLY | O_CREAT |
					O_TRUNC, 0644);
			break;
		case 'x':
			hex_data = true;
			break;
		default:
			errx(1, "bad options");
		}
	}

	char errbuf[PCAP_ERRBUF_SIZE];
	struct af9035 af9035 = {};

	if (pcap_init(0, errbuf) < 0)
		errx(1, "pcap_init: %s", errbuf);

	pcap_t *pcap = pcap_open_offline(argv[optind], errbuf);
	if (!pcap)
		errx(1, "pcap_open_offline: %s", errbuf);

	if (pcap_loop(pcap, -1, pcap_cb, (void *)&af9035) == PCAP_ERROR)
		errx(1, "pcap_loop: %s", pcap_geterr(pcap));


	pcap_close(pcap);
	close(dump_isoc);
	close(dump_fw);
	close(audio_fd);
	close(video_fd);

	return 0;
}
