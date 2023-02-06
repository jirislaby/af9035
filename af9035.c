// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2013 Lubomir Rintel
 * Copyright (C) 2009 Antti Palosaari <crope@iki.fi>
 * Copyright (C) 2012 Antti Palosaari <crope@iki.fi>
 * Copyright (C) 2023 Jiri Slaby
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/ratelimit.h>
#include <linux/slab.h>
#include <linux/usb.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

//#include "af9033.h"
#include "blob.h"

#define USB_VID_DEXATEK		0x1d19

#define CMD_MEM_RD		0x00
#define CMD_MEM_WR		0x01
#define CMD_I2C_RD		0x02
#define CMD_I2C_WR		0x03
#define CMD_IR_GET		0x18
#define CMD_FW_DL		0x21
#define CMD_FW_QUERYINFO	0x22
#define CMD_FW_BOOT		0x23
#define CMD_FW_DL_BEGIN		0x24
#define CMD_FW_DL_END		0x25
#define CMD_FW_SCATTER_WR	0x29
#define CMD_GENERIC_I2C_RD	0x2a
#define CMD_GENERIC_I2C_WR	0x2b

#define MAX_XFER_SIZE		64

#define WARM			0
#define COLD			1

/*
 * The I2C speed register is calculated with:
 *    I2C speed register = (1000000000 / (24.4 * 16 * I2C_speed))
 *
 * 0x7 is ~366 kbps
 * 0xd is ~197 kbps
 */
#define I2C_SPEED_366K 0x7
#define I2C_SPEED_197k 0xd

#define EEPROM_BASE_AF9035        0x42f5
#define EEPROM_SHIFT                0x10

#define EEPROM_IR_MODE              0x18
#define EEPROM_TS_MODE              0x31
#define EEPROM_2ND_DEMOD_ADDR       0x32
#define EEPROM_IR_TYPE              0x34
#define EEPROM_1_IF_L               0x38
#define EEPROM_1_IF_H               0x39
#define EEPROM_1_TUNER_ID           0x3c

#define AF9035_FIRMWARE "af9035.fw"

#define STREAM_BUFS		15
#define STREAM_BUFS_PER_URB	32
#define STREAM_BUF_SIZE		3072

#define PACKET_SIZE		184

static const u32 clock_lut_af9035[] = {
	20480000, /*      FPGA */
	16384000, /* 16.38 MHz */
	20480000, /* 20.48 MHz */
	36000000, /* 36.00 MHz */
	30000000, /* 30.00 MHz */
	26000000, /* 26.00 MHz */
	28000000, /* 28.00 MHz */
	32000000, /* 32.00 MHz */
	34000000, /* 34.00 MHz */
	24000000, /* 24.00 MHz */
	22000000, /* 22.00 MHz */
	12000000, /* 12.00 MHz */
};

struct af9035 {
#define BUF_LEN 64
	u8 buf[BUF_LEN];
	u8 seq; /* packet sequence number */

	u8 prechip_version;
	u8 chip_version;
	u16 chip_type;

	u8 eeprom[256];
	//u8 af9033_i2c_addr[2];

	//#define AF9035_I2C_CLIENT_MAX 4
	//struct i2c_client *i2c_client[AF9035_I2C_CLIENT_MAX];

	//struct af9033_config af9033_config;

	struct usb_interface *intf;

	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct vb2_queue vb2q;

	struct mutex vb2q_lock;
	struct mutex v4l2_lock;
	struct mutex usb_lock;

	void *stream_bufs[STREAM_BUFS];
	struct urb *urbs[STREAM_BUFS];
	bool urbs_submitted[STREAM_BUFS];

	/* List of videobuf2 buffers protected by a lock. */
	spinlock_t buflock;
	struct list_head bufs;

	u8 packet_buf[PACKET_SIZE];
	bool synced;
	unsigned sequence, vsize, asize;
};

static int af9035_sleep(struct af9035 *af9035);

/* =========================== FROM dvb_usb_urb.c =========================== */

static int af9035_rw(struct af9035 *af9035, u8 *wbuf, u16 wlen,
		     u8 *rbuf, u16 rlen)
{
	struct usb_device *udev = interface_to_usbdev(af9035->intf);
        int ret, actual_length;

        if (!wbuf || !wlen) {
                dev_dbg(&udev->dev, "%s: failed=%d\n", __func__, -EINVAL);
                return -EINVAL;
        }

        //dev_dbg(&udev->dev, "%s: >>> %*ph\n", __func__, wlen, wbuf);

        ret = usb_bulk_msg(udev, usb_sndbulkpipe(udev, 0x02), wbuf, wlen,
			   &actual_length, 2000);
        if (ret) {
                dev_err(&udev->dev, "%s: usb_bulk_msg() failed=%d\n",
                                KBUILD_MODNAME, ret);
                return ret;
        }
        if (actual_length != wlen) {
                dev_err(&udev->dev, "%s: usb_bulk_msg() write length=%d, actual=%d\n",
                        KBUILD_MODNAME, wlen, actual_length);
                return -EIO;
        }

        /* an answer is expected */
        if (rbuf && rlen) {
                ret = usb_bulk_msg(udev, usb_rcvbulkpipe(udev, 0x81),
				   rbuf, rlen, &actual_length, 2000);
                if (ret)
                        dev_err(&udev->dev,
                                        "%s: usb_bulk_msg() failed=%d\n",
                                        __func__, ret);

                /*dev_dbg(&udev->dev, "%s: <<< %*ph\n", __func__,
                                actual_length, rbuf);*/
        }

        return ret;
}

/* =========================== FROM af9035.c =========================== */

struct usb_req {
	u8  cmd;
	u8  mbox;
	u8  wlen;
	const u8  *wbuf;
	u8  rlen;
	u8  *rbuf;
};

static u16 af9035_checksum(const u8 *buf, size_t len)
{
	size_t i;
	u16 checksum = 0;

	for (i = 1; i < len; i++) {
		if (i % 2)
			checksum += buf[i] << 8;
		else
			checksum += buf[i];
	}
	checksum = ~checksum;

	return checksum;
}

static int af9035_ctrl_msg(struct af9035 *state, struct usb_req *req)
{
#define REQ_HDR_LEN 4 /* send header size */
#define ACK_HDR_LEN 3 /* rece header size */
#define CHECKSUM_LEN 2
#define USB_TIMEOUT 2000
	struct usb_interface *intf = state->intf;
	int ret, wlen, rlen;
	u16 checksum, tmp_checksum;

	mutex_lock(&state->usb_lock);

	/* buffer overflow check */
	if (req->wlen > (BUF_LEN - REQ_HDR_LEN - CHECKSUM_LEN) ||
			req->rlen > (BUF_LEN - ACK_HDR_LEN - CHECKSUM_LEN)) {
		dev_err(&intf->dev, "too much data wlen=%d rlen=%d\n",
			req->wlen, req->rlen);
		ret = -EINVAL;
		goto exit;
	}

	state->buf[0] = REQ_HDR_LEN + req->wlen + CHECKSUM_LEN - 1;
	state->buf[1] = req->mbox;
	state->buf[2] = req->cmd;
	state->buf[3] = state->seq++;
	memcpy(&state->buf[REQ_HDR_LEN], req->wbuf, req->wlen);

	wlen = REQ_HDR_LEN + req->wlen + CHECKSUM_LEN;
	rlen = ACK_HDR_LEN + req->rlen + CHECKSUM_LEN;

	/* calc and add checksum */
	checksum = af9035_checksum(state->buf, state->buf[0] - 1);
	state->buf[state->buf[0] - 1] = (checksum >> 8);
	state->buf[state->buf[0] - 0] = (checksum & 0xff);

	/* no ack for these packets */
	if (req->cmd == CMD_FW_DL)
		rlen = 0;

	ret = af9035_rw(state, state->buf, wlen, state->buf, rlen);
	if (ret)
		goto exit;

	/* no ack for those packets */
	if (req->cmd == CMD_FW_DL)
		goto exit;

	/* verify checksum */
	checksum = af9035_checksum(state->buf, rlen - 2);
	tmp_checksum = (state->buf[rlen - 2] << 8) | state->buf[rlen - 1];
	if (tmp_checksum != checksum) {
		dev_err(&intf->dev, "command=%02x checksum mismatch (%04x != %04x)\n",
			req->cmd, tmp_checksum, checksum);
		ret = -EIO;
		goto exit;
	}

	/* check status */
	if (state->buf[2]) {
		/* fw returns status 1 when IR code was not received */
		if (req->cmd == CMD_IR_GET || state->buf[2] == 1) {
			ret = 1;
			goto exit;
		}

		dev_dbg(&intf->dev, "command=%02x failed fw error=%d\n",
			req->cmd, state->buf[2]);
		ret = -EIO;
		goto exit;
	}

	/* read request, copy returned data to return buf */
	if (req->rlen)
		memcpy(req->rbuf, &state->buf[ACK_HDR_LEN], req->rlen);
exit:
	mutex_unlock(&state->usb_lock);
	return ret;
}

/* write multiple registers */
static int af9035_wr_regs(struct af9035 *state, u32 reg, const u8 *val, int len)
{
	struct usb_interface *intf = state->intf;
	u8 wbuf[MAX_XFER_SIZE];
	u8 mbox = (reg >> 16) & 0xff;
	struct usb_req req = { CMD_MEM_WR, mbox, 6 + len, wbuf, 0, NULL };

	if (6 + len > sizeof(wbuf)) {
		dev_warn(&intf->dev, "i2c wr: len=%d is too big!\n", len);
		return -EOPNOTSUPP;
	}

	wbuf[0] = len;
	wbuf[1] = 2;
	wbuf[2] = 0;
	wbuf[3] = 0;
	wbuf[4] = (reg >> 8) & 0xff;
	wbuf[5] = (reg >> 0) & 0xff;
	memcpy(&wbuf[6], val, len);

	return af9035_ctrl_msg(state, &req);
}

static int af9035_wr_i2c(struct af9035 *state, u8 mbox, u8 bus, u8 addr,
			 const u8 *val, int len)
{
	u8 buf[MAX_XFER_SIZE];
	struct usb_req req = { CMD_GENERIC_I2C_WR, 0, 3 + len, buf, 0, NULL };

	req.mbox = mbox;
	buf[0] = len;
	buf[1] = bus;
	buf[2] = addr << 1;
	memcpy(&buf[3], val, len);
	return af9035_ctrl_msg(state, &req);
}

/* read multiple registers */
static int af9035_rd_regs(struct af9035 *state, u32 reg, u8 *val, int len)
{
	u8 wbuf[] = { len, 2, 0, 0, (reg >> 8) & 0xff, reg & 0xff };
	u8 mbox = (reg >> 16) & 0xff;
	struct usb_req req = { CMD_MEM_RD, mbox, sizeof(wbuf), wbuf, len, val };

	return af9035_ctrl_msg(state, &req);
}

/* write single register */
static int af9035_wr_reg(struct af9035 *state, u32 reg, u8 val)
{
	return af9035_wr_regs(state, reg, &val, 1);
}

/* read single register */
static int af9035_rd_reg(struct af9035 *state, u32 reg, u8 *val)
{
	return af9035_rd_regs(state, reg, val, 1);
}

#if 0
/* write single register with mask */
static int af9035_wr_reg_mask(struct af9035 *state, u32 reg, u8 val,
		u8 mask)
{
	int ret;
	u8 tmp;

	/* no need for read if whole reg is written */
	if (mask != 0xff) {
		ret = af9035_rd_regs(state, reg, &tmp, 1);
		if (ret)
			return ret;

		val &= mask;
		tmp &= ~mask;
		val |= tmp;
	}

	return af9035_wr_regs(state, reg, &val, 1);
}
#endif

static int af9035_write_blob(struct af9035 *af9035,
			     const struct af9035_blob *blob, unsigned size)
{
	unsigned i;
	int ret;

	for (i = 0; i < size; i++) {
		if (blob[i].type == 0) {
			ret = af9035_wr_regs(af9035, blob[i].reg, blob[i].data,
						 blob[i].len);
			if (ret) {
				dev_err(&af9035->intf->dev,
					"%uth blob failed\n", i);
				return ret;
			}
		} else {
			ret = af9035_wr_i2c(af9035, 0, blob[i].bus,
					    blob[i].addr, blob[i].data,
					    blob[i].len);
			if (ret) {
				if (af9035->buf[2] == blob[i].sta) {
					dev_dbg(&af9035->intf->dev,
						"%uth blob [%.2x:%.2x] EXPECTEDLY failed\n",
						i, blob[i].bus, blob[i].addr);
				} else {
					dev_err(&af9035->intf->dev,
						"%uth blob failed\n", i);
					return ret;
				}
			}
		}
	}

	return 0;
}

static int af9035_identify_state(struct af9035 *state)
{
	struct usb_interface *intf = state->intf;
	int ret, i;
	u8 tmp;
	u8 wbuf[1] = { 1 };
	u8 rbuf[4];
	struct usb_req req = { CMD_FW_QUERYINFO, 0, sizeof(wbuf), wbuf,
			sizeof(rbuf), rbuf };

	ret = af9035_rd_regs(state, 0x1222, rbuf, 3);
	if (ret < 0)
		goto err;

	state->chip_version = rbuf[0];
	state->chip_type = rbuf[2] << 8 | rbuf[1] << 0;

	dev_dbg(&intf->dev, "chip_version=%02x chip_type=%04x\n",
		state->chip_version, state->chip_type);

	ret = af9035_rd_reg(state, 0x384f, &state->prechip_version);
	if (ret < 0)
		goto err;

	dev_info(&intf->dev, "prechip_version=%02x chip_version=%02x chip_type=%04x\n",
		 state->prechip_version, state->chip_version, state->chip_type);

	/* Read and store eeprom */
	for (i = 0; i < 256; i += 32) {
		ret = af9035_rd_regs(state, EEPROM_BASE_AF9035 + i,
				     &state->eeprom[i],
				     32);
		if (ret < 0)
			goto err;
	}

	print_hex_dump_bytes("ee: ", DUMP_PREFIX_OFFSET, state->eeprom, 256);

	/* check for dual tuner mode */
	tmp = state->eeprom[EEPROM_TS_MODE];
	if (tmp != 0)
		dev_warn(&intf->dev, "ts mode=%d not supported!", tmp);

	dev_dbg(&intf->dev, "ts mode=%d\n", tmp);

	ret = af9035_ctrl_msg(state, &req);
	if (ret < 0)
		goto err;

	dev_dbg(&intf->dev, "firmware version=%d.%d.%d.%d",
		rbuf[0], rbuf[1], rbuf[2], rbuf[3]);
	if (rbuf[0] || rbuf[1] || rbuf[2] || rbuf[3])
		ret = WARM;
	else
		ret = COLD;

	return ret;

err:
	dev_dbg(&intf->dev, "%s: failed=%d\n", __func__, ret);

	return ret;
}

static int af9035_download_firmware_old(struct af9035 *state,
					const struct firmware *fw)
{
	struct usb_interface *intf = state->intf;
	int ret, i, j, len;
	u8 wbuf[1];
	struct usb_req req = { 0, 0, 0, NULL, 0, NULL };
	struct usb_req req_fw_dl = { CMD_FW_DL, 0, 0, wbuf, 0, NULL };
	u8 hdr_core;
	u16 hdr_addr, hdr_data_len, hdr_checksum;
#define MAX_DATA 58
#define HDR_SIZE 7

	/*
	 * Thanks to Daniel Glöckner <daniel-gl@gmx.net> about that info!
	 *
	 * byte 0: MCS 51 core
	 *  There are two inside the AF9035 (1=Link and 2=OFDM) with separate
	 *  address spaces
	 * byte 1-2: Big endian destination address
	 * byte 3-4: Big endian number of data bytes following the header
	 * byte 5-6: Big endian header checksum, apparently ignored by the chip
	 *  Calculated as ~(h[0]*256+h[1]+h[2]*256+h[3]+h[4]*256)
	 */

	for (i = fw->size; i > HDR_SIZE;) {
		hdr_core = fw->data[fw->size - i + 0];
		hdr_addr = fw->data[fw->size - i + 1] << 8;
		hdr_addr |= fw->data[fw->size - i + 2] << 0;
		hdr_data_len = fw->data[fw->size - i + 3] << 8;
		hdr_data_len |= fw->data[fw->size - i + 4] << 0;
		hdr_checksum = fw->data[fw->size - i + 5] << 8;
		hdr_checksum |= fw->data[fw->size - i + 6] << 0;

		dev_dbg(&intf->dev, "core=%d addr=%04x data_len=%d checksum=%04x\n",
			hdr_core, hdr_addr, hdr_data_len, hdr_checksum);

		if (((hdr_core != 1) && (hdr_core != 2)) ||
				(hdr_data_len > i)) {
			dev_dbg(&intf->dev, "bad firmware\n");
			break;
		}

		/* download begin packet */
		req.cmd = CMD_FW_DL_BEGIN;
		ret = af9035_ctrl_msg(state, &req);
		if (ret < 0)
			goto err;

		/* download firmware packet(s) */
		for (j = HDR_SIZE + hdr_data_len; j > 0; j -= MAX_DATA) {
			len = j;
			if (len > MAX_DATA)
				len = MAX_DATA;
			req_fw_dl.wlen = len;
			req_fw_dl.wbuf = (u8 *) &fw->data[fw->size - i +
					HDR_SIZE + hdr_data_len - j];
			ret = af9035_ctrl_msg(state, &req_fw_dl);
			if (ret < 0)
				goto err;
		}

		/* download end packet */
		req.cmd = CMD_FW_DL_END;
		ret = af9035_ctrl_msg(state, &req);
		if (ret < 0)
			goto err;

		i -= hdr_data_len + HDR_SIZE;

		dev_dbg(&intf->dev, "data uploaded=%zu\n", fw->size - i);
	}

	/* print warn if firmware is bad, continue and see what happens */
	if (i)
		dev_warn(&intf->dev, "bad firmware\n");

	return 0;

err:
	dev_dbg(&intf->dev, "failed=%d\n", ret);

	return ret;
}

static int af9035_download_firmware(struct af9035 *state,
				    const struct firmware *fw)
{
	struct usb_interface *intf = state->intf;
	int ret;
	u8 wbuf[1];
	u8 rbuf[4];
	struct usb_req req = { 0, 0, 0, NULL, 0, NULL };
	struct usb_req req_fw_ver = { CMD_FW_QUERYINFO, 0, 1, wbuf, 4, rbuf };

	if (fw->data[0] != 0x01)
		return -EINVAL;

	ret = af9035_download_firmware_old(state, fw);
	if (ret < 0)
		goto err;

	/* firmware loaded, request boot */
	req.cmd = CMD_FW_BOOT;
	ret = af9035_ctrl_msg(state, &req);
	if (ret < 0)
		goto err;

	/* ensure firmware starts */
	wbuf[0] = 1;
	ret = af9035_ctrl_msg(state, &req_fw_ver);
	if (ret < 0)
		goto err;

	if (!(rbuf[0] || rbuf[1] || rbuf[2] || rbuf[3])) {
		dev_err(&intf->dev, "firmware did not run\n");
		ret = -ENODEV;
		goto err;
	}

	dev_info(&intf->dev, "firmware version=%d.%d.%d.%d\n",
		 rbuf[0], rbuf[1], rbuf[2], rbuf[3]);

	return 0;

err:
	dev_dbg(&intf->dev, "%s: failed=%d\n", __func__, ret);

	return ret;
}

static int af9035_read_config(struct af9035 *state)
{
	struct usb_interface *intf = state->intf;
	int ret;
	u8 tmp;
	u16 tmp16;

#if 0
	/* Demod I2C address */
	state->af9033_i2c_addr[0] = 0x1c;
	state->af9033_i2c_addr[1] = 0x1d;
	state->af9033_config.adc_multiplier = AF9033_ADC_MULTIPLIER_2X;
	state->af9033_config.ts_mode = AF9033_TS_MODE_USB;
#endif
	//state->it930x_addresses = 0;

	dev_dbg(&intf->dev, "ir=%02x/%02x\n", state->eeprom[EEPROM_IR_MODE],
		state->eeprom[EEPROM_IR_TYPE]);

	/* tuner */
	tmp = state->eeprom[EEPROM_1_TUNER_ID];
	if (tmp != 0) {
		dev_warn(&intf->dev, "tuner id=%02x not supported\n", tmp);
		return -EINVAL;
	}

	dev_dbg(&intf->dev, "tuner=%02x\n", tmp);

	//state->af9033_config.tuner = tmp;

	/* tuner IF frequency */
	tmp = state->eeprom[EEPROM_1_IF_L];
	tmp16 = tmp << 0;
	tmp = state->eeprom[EEPROM_1_IF_H];
	tmp16 |= tmp << 8;
	if (tmp16 != 0) {
		dev_warn(&intf->dev, "freq not zero (%u)\n", tmp16);
		return -EINVAL;
	}

	dev_dbg(&intf->dev, "IF=%d\n", tmp16);

	/* get demod clock */
	ret = af9035_rd_reg(state, 0x00d800, &tmp);
	if (ret < 0)
		goto err;

	tmp = (tmp >> 0) & 0x0f;

	if (tmp >= ARRAY_SIZE(clock_lut_af9035)) {
		dev_warn(&intf->dev, "invalid clock offset: %u\n", tmp16);
		return -EINVAL;
	}

	//state->af9033_config.clock = clock_lut_af9035[tmp];

	dev_dbg(&intf->dev, "demod clk=%u -> %u kHz\n", tmp,
		clock_lut_af9035[tmp] / 1000);

	return 0;

err:
	dev_dbg(&intf->dev, "%s: failed=%d\n", __func__, ret);

	return ret;
}

/* =========================== V4L =========================== */

struct af9035_buf {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

static int af9035_queue_setup(struct vb2_queue *vq,
			     unsigned int *nbuffers,
			     unsigned int *nplanes, unsigned int sizes[],
			     struct device *alloc_devs[])
{
	*nbuffers = 2;
	*nplanes = 1;
	/* half-size (interlacing) of 16bit (UYVY) 720x576 picture */
	sizes[0] = 720*576*2/2;

	return 0;
}

static void af9035_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct af9035 *af9035 = vb2_get_drv_priv(vb->vb2_queue);
	struct af9035_buf *buf = container_of(vbuf, struct af9035_buf, vb);
	unsigned long flags;

	if (af9035->intf == NULL) {
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		return;
	}

	spin_lock_irqsave(&af9035->buflock, flags);
	list_add_tail(&buf->list, &af9035->bufs);
	spin_unlock_irqrestore(&af9035->buflock, flags);
}

static void af9035_reclaim_buffers(struct af9035 *af9035,
				   enum vb2_buffer_state state)
{
	unsigned long flags;
	unsigned cnt = 0;

	spin_lock_irqsave(&af9035->buflock, flags);
	while (!list_empty(&af9035->bufs)) {
		struct af9035_buf *buf = list_first_entry(&af9035->bufs,
				struct af9035_buf, list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
		list_del(&buf->list);
		cnt++;
	}
	spin_unlock_irqrestore(&af9035->buflock, flags);
}

static void af9035_kill_urbs(struct af9035 *af9035)
{
        for (unsigned i = 0; i < STREAM_BUFS; i++) {
		if (!af9035->urbs_submitted[i])
			continue;
                dev_dbg(&af9035->intf->dev, "%s: kill urb=%2d/%p\n", __func__,
			i, af9035->urbs[i]);
                usb_kill_urb(af9035->urbs[i]);
		af9035->urbs_submitted[i] = false;
        }
}

static int af9035_submit_urbs(struct af9035 *af9035)
{
	int ret;

        for (unsigned i = 0; i < STREAM_BUFS; i++) {
                dev_dbg(&af9035->intf->dev, "%s: submit urb=%2d/%p\n", __func__,
			i, af9035->urbs[i]);
                ret = usb_submit_urb(af9035->urbs[i], GFP_KERNEL);
                if (ret) {
                        dev_err(&af9035->intf->dev,
                                        "%s: could not submit urb no. %d - get them all back\n",
                                        __func__, i);
                        af9035_kill_urbs(af9035);
                        return ret;
                }
		af9035->urbs_submitted[i] = true;
        }

	return 0;
}

static int af9035_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct af9035 *af9035 = vb2_get_drv_priv(vq);
	int ret;

	af9035->synced = false;
	af9035->sequence = 0;
	af9035->vsize = 0;
	af9035->asize = 0;

	if (af9035->intf == NULL) {
		ret = -ENODEV;
		goto err;
	}

	ret = af9035_write_blob(af9035, af9035_start_blob1,
				ARRAY_SIZE(af9035_start_blob1));
	if (ret) {
		dev_err(&af9035->intf->dev, "%s: 1st blob failed\n", __func__);
		goto err;
	}

	ret = af9035_submit_urbs(af9035);
	if (ret) {
		dev_err(&af9035->intf->dev, "%s: submit urbs failed\n",
			__func__);
		goto err;
	}

	ret = af9035_write_blob(af9035, af9035_start_blob2,
				ARRAY_SIZE(af9035_start_blob2));
	if (ret) {
		dev_err(&af9035->intf->dev, "%s: 2nd blob failed\n", __func__);
		goto err_urbs;
	}

	return 0;
err_urbs:
	af9035_kill_urbs(af9035);
err:
	af9035_reclaim_buffers(af9035, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void af9035_stop_streaming(struct vb2_queue *vq)
{
	struct af9035 *af9035 = vb2_get_drv_priv(vq);

	if (af9035->intf) {
		af9035_sleep(af9035);
		af9035_kill_urbs(af9035);
	}

	af9035_reclaim_buffers(af9035, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops af9035_vb2_ops = {
	.queue_setup = af9035_queue_setup,
	.buf_queue = af9035_buf_queue,
	.start_streaming = af9035_start_streaming,
	.stop_streaming = af9035_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static void af9035_release(struct v4l2_device *v4l2_dev)
{
	struct af9035 *af9035 = container_of(v4l2_dev, struct af9035, v4l2_dev);

	v4l2_device_unregister(&af9035->v4l2_dev);
	pr_info("%s: FREE\n", __func__);
	kfree(af9035);
}

static const struct v4l2_file_operations af9035_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.read = vb2_fop_read,
	.poll = vb2_fop_poll,
};

static int af9035_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
        struct af9035 *dev = video_drvdata(file);

        strscpy(cap->driver, "af9035", sizeof(cap->driver));
        strscpy(cap->card, "af9035", sizeof(cap->card));
        usb_make_path(interface_to_usbdev(dev->intf), cap->bus_info,
			sizeof(cap->bus_info));

        return 0;
}

static int af9035_enum_input(struct file *file, void *priv,
			     struct v4l2_input *i)
{
        struct af9035 *dev = video_drvdata(file);

	strscpy(i->name, "Composite", sizeof(i->name));
        i->type = V4L2_INPUT_TYPE_CAMERA;
        i->std = dev->vdev.tvnorms;

        return 0;
}

static int af9035_enum_fmt_vid_cap(struct file *file, void  *priv,
				   struct v4l2_fmtdesc *f)
{
        if (f->index > 0)
                return -EINVAL;

        f->pixelformat = V4L2_PIX_FMT_UYVY;

        return 0;
}

static int af9035_fmt_vid_cap(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	f->fmt.pix.width = 720;
	f->fmt.pix.height = 576;
	f->fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
	f->fmt.pix.field = V4L2_FIELD_SEQ_BT;
	f->fmt.pix.bytesperline = 720 * 2;
	f->fmt.pix.sizeimage = (f->fmt.pix.bytesperline * f->fmt.pix.height);
	f->fmt.pix.colorspace = V4L2_COLORSPACE_DEFAULT;

	return 0;
}

static int af9035_g_std(struct file *file, void *priv, v4l2_std_id *norm)
{
	*norm = V4L2_STD_625_50;

	return 0;
}

static const struct v4l2_ioctl_ops af9035_ioctl_ops = {
	.vidioc_querycap = af9035_querycap,
	.vidioc_enum_input = af9035_enum_input,
	.vidioc_enum_fmt_vid_cap = af9035_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = af9035_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = af9035_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = af9035_fmt_vid_cap,
	.vidioc_g_std = af9035_g_std,
//	.vidioc_s_std = af9035_s_std,
//	.vidioc_g_input = af9035_g_input,
//	.vidioc_s_input = af9035_s_input,

	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static int af9035_video_init(struct af9035 *af9035)
{
	struct usb_interface *intf = af9035->intf;
	int ret;

	/* videobuf2 structure */
	af9035->vb2q.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	af9035->vb2q.io_modes = VB2_MMAP | VB2_USERPTR | VB2_READ;
	af9035->vb2q.drv_priv = af9035;
	af9035->vb2q.buf_struct_size = sizeof(struct af9035_buf);
	af9035->vb2q.ops = &af9035_vb2_ops;
	af9035->vb2q.mem_ops = &vb2_vmalloc_memops;
	af9035->vb2q.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	af9035->vb2q.lock = &af9035->vb2q_lock;
	ret = vb2_queue_init(&af9035->vb2q);
	if (ret < 0) {
		dev_err(&intf->dev, "Could not initialize videobuf2 queue\n");
		goto err;
	}

	af9035->v4l2_dev.release = af9035_release;
	ret = v4l2_device_register(&intf->dev, &af9035->v4l2_dev);
	if (ret < 0) {
		dev_err(&intf->dev, "Could not register v4l2 device\n");
		goto err;
	}

	/* Video structure */
	strscpy(af9035->vdev.name, "af9035", sizeof(af9035->vdev.name));
	af9035->vdev.v4l2_dev = &af9035->v4l2_dev;
	af9035->vdev.release = video_device_release_empty;
	af9035->vdev.fops = &af9035_fops;
	af9035->vdev.ioctl_ops = &af9035_ioctl_ops;
	af9035->vdev.tvnorms = V4L2_STD_PAL_D;
	af9035->vdev.queue = &af9035->vb2q;
	af9035->vdev.lock = &af9035->v4l2_lock;
	af9035->vdev.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE |
		V4L2_CAP_STREAMING;
	video_set_drvdata(&af9035->vdev, af9035);
	ret = video_register_device(&af9035->vdev, VFL_TYPE_VIDEO, -1);
	if (ret < 0) {
		dev_warn(&intf->dev, "Could not register video device\n");
		goto err_v4l2;
	}

	return 0;
err_v4l2:
	v4l2_device_unregister(&af9035->v4l2_dev);
err:
	return ret;
}

static void af9035_video_free(struct af9035 *af9035)
{
	mutex_lock(&af9035->vb2q_lock);
	mutex_lock(&af9035->v4l2_lock);

	//af9035_stop(af9035);
	vb2_video_unregister_device(&af9035->vdev);
	v4l2_device_disconnect(&af9035->v4l2_dev);

	mutex_unlock(&af9035->v4l2_lock);
	mutex_unlock(&af9035->vb2q_lock);

	v4l2_device_put(&af9035->v4l2_dev);
}

/* =========================== FRAME decode =========================== */

struct header {
	union {
		struct {
			uint8_t FA;
			/*
			 * 4 bits: type
			 * 4 bits: seq_bot
			 * 5 bits: seq_top
			 * 3 bits: ???
			 * 1 bit: FINAL
			 * 1 bit: SPEC
			 * 1 bit: AUDIO
			 * 1 bit: REAL
			 * 4 bits: synch
			 */
			uint8_t type_seq;
			uint8_t seq_res;
			uint8_t res_stream;
		};
		uint32_t val;
	};
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

static inline uint16_t HEADER_SEQ(const struct header *hdr)
{
	uint16_t seq = HEADER_SEQ_TOP(hdr);

	seq <<= HEADER_SEQ_BOT_BITS;
	seq |= HEADER_SEQ_BOT(hdr);

	WARN_ON(seq & 1);

	return seq >> 1;
}

static inline unsigned HEADER_SIZE(const struct header *hdr)
{
	return (HEADER_HISIZE(hdr) << 6) | (HEADER_WORDS(hdr) << 2);
}

static int maincoon(struct af9035 *af9035)
{
	struct header header;
	ssize_t to_read;

	while (1) {
		bool dump_video = false;
		ssize_t rd = read((void *)&header, sizeof(header));
		if (!rd)
			break;

		if (rd != sizeof(header)) {
			dev_warn(&af9035->intf->dev, "read(header)\n");
			return -EIO;
		}

		if (header.FA != 0xfa) {
			dev_dbg(&af9035->intf->dev, " BAD(%.8x)\n",
					be32_to_cpu(header.val));
			continue;
		}

retry:
		header.val = be32_to_cpu(header.val);

		to_read = HEADER_SIZE(&header);
		dev_dbg(&af9035->intf->dev,
				"hdr=%.8x SEQ=%3u/%2x SY=%u %c%c%c%c%c len=%3zd",
		       header.val,
		       HEADER_SEQ(&header), HEADER_SEQ(&header),
		       HEADER_SY(&header),
		       HEADER_FLIP(&header) ? 'F' : '_',
		       HEADER_REAL(&header) ? 'R' : '_',
		       HEADER_AUDIO(&header) ? 'A' : '_',
		       HEADER_SYNC(&header) ? 'S' : '_',
		       HEADER_unk1(&header) ? 'U' : '_',
		       to_read);

		if (to_read) {
			rd = read(&af9035->packet_buf, to_read);
			if (rd < 0) {
				dev_warn(&af9035->intf->dev,
					 "read(data) at 0xx");
				return -EIO;
			}
			if (rd != to_read)
				break;

			/* BUG in FW? */
			if (to_read == 4 && af9035->packet_buf[0] == 0xfa) {
				pr_cont(" BAD -- retrying\n");
				memcpy(&header, af9035->packet_buf,
				       sizeof(header));
				goto retry;
			}

			if (HEADER_unk1(&header)) {
				pr_cont(" U");
			} else if (HEADER_SYNC(&header)) {
				pr_cont(" S");
				dump_video = true;
				if (!af9035->synced) {
					af9035->asize = 0;
					af9035->vsize = 0;
				}
				af9035->synced = true;
			} else if (HEADER_AUDIO(&header)) {
				pr_cont(" A");
#if 0
				if (synced && audio_fd >= 0)
					write(audio_fd, buf, rd);
				asize += rd;
#endif
			} else if (to_read == 180) {
				pr_cont(" V");
#if 0
				if (af9035->synced &&
				    af9035->vsize + (size_t)rd < sizeof(videobuf))
					memcpy(&videobuf[af9035->vsize],
					       &af9035->packet_buf[4], rd);
#endif
				af9035->vsize += rd;
			} else
				pr_cont(" _");

			pr_cont(" vsize=%6u asize=%8u", af9035->vsize,
				 af9035->asize);
			//dump_data_limited("payl", buf, rd, 12);
		}

		pr_cont("\n");

#if 0
		if (dump_video && vsize) {
			if (synced && video_fd >= 0) {
				printf("dumping; ");
				write(video_fd, videobuf, sizeof(videobuf));
				memset(videobuf, 0, sizeof(videobuf));
			}
			printf("vsize=%u\n", vsize);
			vsize = 0;
		}
#endif
	}

	pr_cont("\n");

	return 0;
}

static bool af9035_output_to_frame(struct af9035 *af9035, void *frame,
				   const u8 *data, unsigned len)
{
	maincoon(af9035);
	return false;
}

/* =========================== DEV =========================== */

static void af9035_complete(struct af9035 *af9035, const u8 *data, unsigned len)
{
	struct af9035_buf *buf;
	unsigned long flags;
	void *frame;

	//print_hex_dump_bytes("pkt: ", DUMP_PREFIX_NONE, data, min(len, 16U));

        spin_lock_irqsave(&af9035->buflock, flags);

        if (list_empty(&af9035->bufs)) {
                /* No free buffers. Userspace likely too slow. */
                spin_unlock_irqrestore(&af9035->buflock, flags);
                return;
        }

        /* First available buffer. */
        buf = list_first_entry(&af9035->bufs, struct af9035_buf, list);
        frame = vb2_plane_vaddr(&buf->vb.vb2_buf, 0);

	if (af9035_output_to_frame(af9035, frame, data, len)) {
		int size = vb2_plane_size(&buf->vb.vb2_buf, 0);

		buf->vb.field = V4L2_FIELD_SEQ_BT;
		buf->vb.sequence = af9035->sequence++;
		buf->vb.vb2_buf.timestamp = ktime_get_ns();
		vb2_set_plane_payload(&buf->vb.vb2_buf, 0, size);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
		list_del(&buf->list);
	}

        spin_unlock_irqrestore(&af9035->buflock, flags);
}

static void usb_urb_complete(struct urb *urb)
{
        struct af9035 *af9035 = urb->context;
	int ret;
	u8 *b;

/*	dev_dbg(&af9035->intf->dev, "%s: %s urb (%p) completion status=%d length=%d/%d pack_num=%d errors=%d\n",
		__func__,
		usb_pipetype(urb->pipe) == PIPE_ISOCHRONOUS ? "isoc" : "bulk",
		urb, urb->status, urb->actual_length,
		urb->transfer_buffer_length, urb->number_of_packets,
		urb->error_count);*/

	switch (urb->status) {
	case 0:			/* success */
	case -ETIMEDOUT:	/* NAK */
		break;
	case -ECONNRESET:	/* kill */
	case -ENOENT:
	case -ESHUTDOWN:
		return;
	default:        /* error */
		dev_dbg_ratelimited(&af9035->intf->dev,
				    "%s: urb completion failed=%d\n",
				    __func__, urb->status);
		break;
	}

	b = (u8 *)urb->transfer_buffer;
	switch (usb_pipetype(urb->pipe)) {
	case PIPE_ISOCHRONOUS:
		for (unsigned i = 0; i < urb->number_of_packets; i++) {
			/*dev_dbg(&af9035->intf->dev,
				"%s: iso frame descriptor %u state=%d len=%d\n",
				__func__,
				i, urb->iso_frame_desc[i].status,
				urb->iso_frame_desc[i].actual_length);*/
			if (urb->iso_frame_desc[i].status != 0)
				/*dev_dbg(&af9035->intf->dev,
					"%s: iso frame descriptor %u has an error=%d\n",
					__func__, i,
					urb->iso_frame_desc[i].status);*/
				;
			else if (urb->iso_frame_desc[i].actual_length > 0)
				af9035_complete(af9035,
						b + urb->iso_frame_desc[i].offset,
						urb->iso_frame_desc[i].actual_length);

			urb->iso_frame_desc[i].status = 0;
			urb->iso_frame_desc[i].actual_length = 0;
		}
		break;
	default:
		dev_err(&af9035->intf->dev,
			"unknown endpoint type in completion handler\n");
		return;
	}

	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if (ret)
		dev_dbg(&af9035->intf->dev, "%s: usb_submit_urb() failed=%d\n",
			__func__, ret);
	//dev_dbg(&af9035->intf->dev, "%s: urb (%p) submitted\n", __func__, urb);
}

static int af9035_load_fw(struct af9035 *af9035)
{
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, AF9035_FIRMWARE, &af9035->intf->dev);
	if (ret < 0)
		return ret;

	ret = af9035_download_firmware(af9035, fw);
	release_firmware(fw);
	if (ret < 0)
		dev_err(&af9035->intf->dev, "fw upload failed\n");

	return ret;
}

static void af9035_stream_free(struct af9035 *af9035)
{
	for (unsigned a = 0; a < ARRAY_SIZE(af9035->stream_bufs); a++) {
		kfree(af9035->stream_bufs[a]);
		af9035->stream_bufs[a] = NULL;
		usb_free_urb(af9035->urbs[a]);
		af9035->urbs[a] = NULL;
	}
}

static int af9035_stream_init(struct af9035 *af9035)
{
	/* 15 * 32 * 3072 */
	struct usb_host_endpoint *ep;
	struct usb_device *udev = interface_to_usbdev(af9035->intf);
	struct urb *urb;
	int ret;

	ret = -ENOMEM;
	for (unsigned i = 0; i < ARRAY_SIZE(af9035->stream_bufs); i++) {
		af9035->stream_bufs[i] = kcalloc(STREAM_BUFS_PER_URB,
						 STREAM_BUF_SIZE, GFP_KERNEL);
		if (!af9035->stream_bufs[i])
			goto err;

		af9035->urbs[i] = urb = usb_alloc_urb(STREAM_BUFS_PER_URB,
						      GFP_KERNEL);
		if (!urb)
			goto err;

		urb->dev = udev;
		urb->context = af9035;
		urb->complete = usb_urb_complete;
		urb->pipe = usb_rcvisocpipe(udev, 0x6);
		urb->transfer_flags = URB_ISO_ASAP;
		urb->interval = 1;
		urb->number_of_packets = STREAM_BUFS_PER_URB;
		urb->transfer_buffer_length = STREAM_BUFS_PER_URB *
			STREAM_BUF_SIZE;
		urb->transfer_buffer = af9035->stream_bufs[i];

		ep = usb_pipe_endpoint(udev, urb->pipe);
		/*dev_dbg(&af9035->intf->dev, "urb pipe=%.8x maxp=%d isoc=%d\n",
			urb->pipe, usb_endpoint_maxp(&ep->desc),
			usb_endpoint_xfer_isoc(&ep->desc));*/

		for (unsigned j = 0; j < STREAM_BUFS_PER_URB; j++) {
			urb->iso_frame_desc[j].offset = STREAM_BUF_SIZE * j;
			urb->iso_frame_desc[j].length = STREAM_BUF_SIZE;
		}
	}

	return 0;
err:
	af9035_stream_free(af9035);
	return ret;
}

static int af9035_sleep(struct af9035 *af9035)
{
	unsigned counter = 20;
	int ret;
	u8 tmp;

	ret = af9035_wr_reg(af9035, 0x80004c, 0x01);
	if (ret)
		return ret;
	ret = af9035_wr_reg(af9035, 0x800000, 0x00);
	if (ret)
		return ret;

	do {
		ret = af9035_rd_reg(af9035, 0x80004c, &tmp);
		if (ret)
			return ret;
		if (tmp == 0)
			break;
		msleep(5);
	} while (counter-- > 0);

	ret = af9035_rd_reg(af9035, 0x80fb24, &tmp);
	if (ret)
		return ret;
	tmp |= 0x08;
	ret = af9035_wr_reg(af9035, 0x80fb24, tmp);
	if (ret)
		return ret;

	return 0;
}

static int af9035_frontend_init(struct af9035 *af9035)
{
	static const u8 wbuf[4] = { 0x00, 0x08, 0x00, 0x00 };
	int ret;

	/* I2C master bus 1,3 clock speed 366k */
	ret = af9035_wr_reg(af9035, 0x00f103, I2C_SPEED_197k);
	if (ret < 0)
		return ret;

	/* tuner type */
	ret = af9035_wr_reg(af9035, 0xf641, 0x26);
	if (ret < 0)
		return ret;

	ret = af9035_write_blob(af9035, af9035_init_blob,
				ARRAY_SIZE(af9035_init_blob));
	if (ret < 0)
		return ret;

	/* likely unneeded as it (expectedly) fails */
	ret = af9035_wr_i2c(af9035, 0, 0x02, 0xc2, wbuf, sizeof(wbuf));
	if (!ret)
		return -EIO;

	if (af9035->buf[2] == 4)
		dev_dbg(&af9035->intf->dev,
				"write to 02:c2 EXPECTEDLY failed\n");
	else
		return -EIO;

	return af9035_sleep(af9035);
}

static int af9035_dev_init(struct af9035 *af9035)
{
	int ret;

	ret = af9035_identify_state(af9035);
	if (ret < 0)
		goto err;

	dev_info(&af9035->intf->dev, "%s: state=%s\n", __func__,
		 ret == COLD ? "COLD" : "WARM");

	if (ret == COLD) {
		ret = af9035_load_fw(af9035);
		if (ret < 0)
			goto err;
	}

	ret = af9035_read_config(af9035);
	if (ret < 0)
		goto err;

	ret = af9035_stream_init(af9035);
	if (ret < 0)
		goto err;

	ret = af9035_frontend_init(af9035);
	if (ret < 0)
		goto err;

	return 0;
err:
	return ret;
}

static void af9035_dev_exit(struct af9035 *af9035)
{
	af9035_stream_free(af9035);
}

static int af9035_probe(struct usb_interface *intf,
			const struct usb_device_id *id)
{
	struct af9035 *af9035;
	int ret;

	dev_info(&intf->dev, "%s: altset=%u\n", __func__,
		 intf->cur_altsetting->desc.bAlternateSetting);

	if (intf->cur_altsetting->desc.bAlternateSetting != 1) {
		dev_dbg(&intf->dev, "switching to altset 1\n");
		ret = usb_set_interface(interface_to_usbdev(intf), 0, 1);
		if (ret) {
			dev_info(&intf->dev, "%s: cannot set altset to 1\n",
				 __func__);
			return ret;
		}

		dev_info(&intf->dev, "%s: altset=%u\n", __func__,
			 intf->cur_altsetting->desc.bAlternateSetting);

		/* the device needs to set up itself after set_interface */
		msleep(250);
	}

	af9035 = kzalloc(sizeof(*af9035), GFP_KERNEL);
	if (!af9035)
		return -ENOMEM;

	mutex_init(&af9035->vb2q_lock);
	mutex_init(&af9035->v4l2_lock);
	mutex_init(&af9035->usb_lock);
	spin_lock_init(&af9035->buflock);
	INIT_LIST_HEAD(&af9035->bufs);

	af9035->intf = usb_get_intf(intf);

	ret = af9035_dev_init(af9035);
	if (ret)
		goto err_free;

	ret = af9035_video_init(af9035);
	if (ret)
		goto err_dev;

	usb_set_intfdata(intf, af9035);
	v4l2_device_get(&af9035->v4l2_dev);

	return 0;
err_dev:
	af9035_dev_exit(af9035);
err_free:
	usb_put_intf(af9035->intf);
	kfree(af9035);
	return ret;
}

static void af9035_disconnect(struct usb_interface *intf)
{
	struct af9035 *af9035 = usb_get_intfdata(intf);

	usb_set_intfdata(intf, NULL);

	dev_info(&intf->dev, "%s: %zu\n", __func__,
		 intf->cur_altsetting - intf->altsetting);

	af9035_video_free(af9035);

	af9035->intf = NULL;
	usb_put_intf(af9035->intf);

	af9035_dev_exit(af9035);
	v4l2_device_put(&af9035->v4l2_dev);
}

static const struct usb_device_id af9035_usb_ids[] = {
	{ USB_DEVICE(USB_VID_DEXATEK, 0x6105) },
	{}
};
MODULE_DEVICE_TABLE(usb, af9035_usb_ids);

static struct usb_driver af9035_usb_driver = {
	.name           = "AF9015",
	.probe          = af9035_probe,
	.disconnect     = af9035_disconnect,
	.id_table       = af9035_usb_ids
};

module_usb_driver(af9035_usb_driver);
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(AF9035_FIRMWARE);
