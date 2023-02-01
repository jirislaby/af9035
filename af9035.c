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

#include "af9033.h"

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
	u8 ir_mode;
	u8 ir_type;
	u8 af9033_i2c_addr[2];

	//#define AF9035_I2C_CLIENT_MAX 4
	//struct i2c_client *i2c_client[AF9035_I2C_CLIENT_MAX];

	struct af9033_config af9033_config;

	struct usb_interface *intf;

	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct vb2_queue vb2q;

	struct mutex vb2q_lock;
	struct mutex v4l2_lock;
	struct mutex usb_lock;

	/* List of videobuf2 buffers protected by a lock. */
	spinlock_t buflock;
	struct list_head bufs;
};


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
	struct af9035 *af9035 = vb2_get_drv_priv(vq);
#if 1
	dev_info(&af9035->intf->dev, "%s: just guessing\n", __func__);
	*nbuffers = 2;
	*nplanes = 1;
	sizes[0] = 720*576*3;

	return 0;
#else
	unsigned size = USBTV_CHUNK * af9035->n_chunks * 2 * sizeof(u32);

	if (vq->num_buffers + *nbuffers < 2)
		*nbuffers = 2 - vq->num_buffers;
	if (*nplanes)
		return sizes[0] < size ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = size;

	return 0;
#endif
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

static int af9035_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct af9035 *af9035 = vb2_get_drv_priv(vq);

	if (af9035->intf == NULL)
		goto err;
#if 1
	dev_warn(&af9035->intf->dev, "%s: unimplemented\n", __func__);

	return 0;
err:
	af9035_reclaim_buffers(af9035, VB2_BUF_STATE_QUEUED);
	return -ENODEV;
#else
	af9035->last_odd = 1;
	af9035->sequence = 0;
	return af9035_start(af9035);
#endif
}

static void af9035_stop_streaming(struct vb2_queue *vq)
{
	struct af9035 *af9035 = vb2_get_drv_priv(vq);

#if 1
	dev_warn(&af9035->intf->dev, "%s: unimplemented\n", __func__);
	af9035_reclaim_buffers(af9035, VB2_BUF_STATE_ERROR);
#else
	if (af9035->intf)
		af9035_stop(af9035);
#endif
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

static int af9035_video_init(struct af9035 *af9035, struct usb_interface *intf)
{
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
                                        "%s: 2nd usb_bulk_msg() failed=%d\n",
                                        KBUILD_MODNAME, ret);

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
	u8  *wbuf;
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

#if 0
/* write multiple registers */
static int af9035_wr_regs(struct af9035 *state, u32 reg, u8 *val, int len)
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
#endif

/* read multiple registers */
static int af9035_rd_regs(struct af9035 *state, u32 reg, u8 *val, int len)
{
	u8 wbuf[] = { len, 2, 0, 0, (reg >> 8) & 0xff, reg & 0xff };
	u8 mbox = (reg >> 16) & 0xff;
	struct usb_req req = { CMD_MEM_RD, mbox, sizeof(wbuf), wbuf, len, val };

	return af9035_ctrl_msg(state, &req);
}

#if 0
/* write single register */
static int af9035_wr_reg(struct af9035 *state, u32 reg, u8 val)
{
	return af9035_wr_regs(state, reg, &val, 1);
}
#endif

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
	dev_dbg(&intf->dev, "failed=%d\n", ret);

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
	dev_dbg(&intf->dev, "failed=%d\n", ret);

	return ret;
}

static int af9035_read_config(struct af9035 *state)
{
	struct usb_interface *intf = state->intf;
	int ret;
	u8 tmp;
	u16 tmp16;

	/* Demod I2C address */
	state->af9033_i2c_addr[0] = 0x1c;
	state->af9033_i2c_addr[1] = 0x1d;
	state->af9033_config.adc_multiplier = AF9033_ADC_MULTIPLIER_2X;
	state->af9033_config.ts_mode = AF9033_TS_MODE_USB;
	//state->it930x_addresses = 0;

	/* Remote controller */
	state->ir_mode = state->eeprom[EEPROM_IR_MODE];
	state->ir_type = state->eeprom[EEPROM_IR_TYPE];

	/* tuner */
	tmp = state->eeprom[EEPROM_1_TUNER_ID];
	if (tmp != 0) {
		dev_warn(&intf->dev, "tuner id=%02x not supported\n", tmp);
		return -EINVAL;
	}

	dev_dbg(&intf->dev, "tuner=%02x\n", tmp);

	state->af9033_config.tuner = tmp;

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

	state->af9033_config.clock = clock_lut_af9035[tmp];

	dev_dbg(&intf->dev, "demod clk=%u -> %u kHz\n", tmp,
		state->af9033_config.clock / 1000);

	return 0;

err:
	dev_dbg(&intf->dev, "failed=%d\n", ret);

	return ret;
}

/* =========================== DEV =========================== */

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

static int af9035_dev_init(struct af9035 *af9035, struct usb_interface *intf)
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

#if 0
	ret = af9035_i2c_init(af9035);
#endif

	return 0;
err:
	return ret;
}

static int af9035_probe(struct usb_interface *intf,
			const struct usb_device_id *id)
{
	struct af9035 *af9035;
	int ret;

	dev_info(&intf->dev, "%s: altset=%zu\n", __func__,
		 intf->cur_altsetting - intf->altsetting);

	af9035 = kzalloc(sizeof(*af9035), GFP_KERNEL);
	if (!af9035)
		return -ENOMEM;

	mutex_init(&af9035->vb2q_lock);
	mutex_init(&af9035->v4l2_lock);
	mutex_init(&af9035->usb_lock);
	spin_lock_init(&af9035->buflock);
	INIT_LIST_HEAD(&af9035->bufs);

	af9035->intf = usb_get_intf(intf);

	ret = af9035_dev_init(af9035, intf);
	if (ret)
		goto err_free;

	ret = af9035_video_init(af9035, intf);
	if (ret)
		goto err_dev;

	usb_set_intfdata(intf, af9035);
	v4l2_device_get(&af9035->v4l2_dev);

	return 0;
err_dev:
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
