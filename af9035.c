#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/ratelimit.h>
#include <linux/slab.h>
#include <linux/usb.h>

#include <media/v4l2-device.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#define USB_VID_DEXATEK                         0x1d19

struct af9035 {
	struct usb_device *udev;

	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct vb2_queue vb2q;

	struct mutex vb2q_lock;

	/* List of videobuf2 buffers protected by a lock. */
	spinlock_t buflock;
	struct list_head bufs;
};

struct af9035_buf {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

static int af9035_queue_setup(struct vb2_queue *vq,
			     unsigned int *nbuffers,
			     unsigned int *nplanes, unsigned int sizes[],
			     struct device *alloc_devs[])
{
#if 1
	WARN_ON(1);
	return -ENODEV;
#else
	struct af9035 *af9035 = vb2_get_drv_priv(vq);
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

	if (af9035->udev == NULL) {
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		return;
	}

	spin_lock_irqsave(&af9035->buflock, flags);
	list_add_tail(&buf->list, &af9035->bufs);
	spin_unlock_irqrestore(&af9035->buflock, flags);
}

static int af9035_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct af9035 *af9035 = vb2_get_drv_priv(vq);

	if (af9035->udev == NULL)
		return -ENODEV;
#if 1
	WARN_ON(1);
	return -ENODEV;
#else
	af9035->last_odd = 1;
	af9035->sequence = 0;
	return af9035_start(af9035);
#endif
}

static void af9035_stop_streaming(struct vb2_queue *vq)
{
#if 1
	WARN_ON(1);
#else
	struct af9035 *af9035 = vb2_get_drv_priv(vq);

	if (af9035->udev)
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
	kfree(af9035);
	dev_info(v4l2_dev->dev, "FREE\n");
}

static int af9035_probe(struct usb_interface *intf,
			const struct usb_device_id *id)
{
	struct af9035 *af9035;
	int ret;

	dev_info(&intf->dev, "altset=%zu\n",
		 intf->cur_altsetting - intf->altsetting);

	af9035 = kzalloc(sizeof(*af9035), GFP_KERNEL);
	if (!af9035)
		return -ENOMEM;

	mutex_init(&af9035->vb2q_lock);
	spin_lock_init(&af9035->buflock);

	af9035->udev = usb_get_dev(interface_to_usbdev(intf));

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
		goto free;
	}

	af9035->v4l2_dev.release = af9035_release;
	ret = v4l2_device_register(&intf->dev, &af9035->v4l2_dev);
	if (ret < 0) {
		dev_err(&intf->dev, "Could not register v4l2 device\n");
		goto free;
	}

	usb_set_intfdata(intf, af9035);
	v4l2_device_get(&af9035->v4l2_dev);

	return 0;
free:
	usb_put_dev(af9035->udev);
	kfree(af9035);
	return ret;
}

static void af9035_disconnect(struct usb_interface *intf)
{
	struct af9035 *af9035 = usb_get_intfdata(intf);

	dev_info(&intf->dev, "%zu\n", intf->cur_altsetting - intf->altsetting);

	//vb2_video_unregister_device(&af9035->vdev);
	v4l2_device_disconnect(&af9035->v4l2_dev);

	af9035->udev = NULL;
	usb_put_dev(af9035->udev);

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