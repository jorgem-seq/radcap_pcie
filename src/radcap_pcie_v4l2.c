// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * radcap_pcie_v4l2.c - Sonifex Radcap PCIe V4L2 Driver
 * Copyright (c) 2020 Jorge Maidana <jorgem.seq@gmail.com>
 *
 * This driver is a derivative of:
 *
 * Philips SAA7134 driver
 * Copyright (c) 2001-03 Gerd Knorr <kraxel@bytesex.org> [SuSE Labs]
 *
 * Radcap driver for Linux
 * Copyright (c) 2012 Jeff Pages <jeff@sonifex.com.au>
 * Copyright (c) 2012 Kevin Dawson
 *
 * v4l2-pci-skeleton.c
 * Copyright (c) 2014 Cisco Systems, Inc. and/or its affiliates. All rights reserved.
 */

#include <linux/pci.h>
#include <linux/videodev2.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>

#include "radcap_pcie.h"

#define RADCAP_V4L2_NAME	"Radcap PCIe V4L2"

#define NOPILOTVAL		GENMASK(7, 0)
#define FREQMASK		GENMASK(15, 0)
#define RFSIGMAX		GENMASK(15, 0)

/* Freq in kHz */
#define am_freq_to_val(freq)	(((freq) * 20480 / 441) & FREQMASK)
#define fm_freq_to_val(freq)	((((freq) - 97920) * 128 / 51) & FREQMASK)

/* The base for the Radcap_PCIe driver controls. Total of 16 controls are reserved
 * for this driver */
#define V4L2_CID_USER_RADCAP_BASE	(V4L2_CID_USER_BASE + 0x1f00)

enum {
	V4L2_CID_RADCAP_DUAL = (V4L2_CID_USER_RADCAP_BASE + 0),
	V4L2_CID_RADCAP_NARROW,
};

enum {
	TUNER_AM,
	TUNER_FM,
};

static const struct v4l2_tuner tuners[] = {
	[TUNER_AM] = {
		.capability	= V4L2_TUNER_CAP_LOW,
		.audmode	= V4L2_TUNER_MODE_MONO,
		.rxsubchans	= V4L2_TUNER_SUB_MONO,
		.rangelow	=  (500 * 16),
		.rangehigh	= (1710 * 16),
		.type		= V4L2_TUNER_RADIO,
	},
	[TUNER_FM] = {
		.capability	= (V4L2_TUNER_CAP_LOW |
#ifdef RADCAP_FM_RDS
				   V4L2_TUNER_CAP_RDS |
				   V4L2_TUNER_CAP_RDS_BLOCK_IO |
#endif /* RADCAP_FM_RDS */
				   V4L2_TUNER_CAP_STEREO), /* Stereo = auto */
		.audmode	= V4L2_TUNER_MODE_STEREO,
		.rxsubchans	= V4L2_TUNER_SUB_STEREO,
		.rangelow	=  (85000 * 16),
		.rangehigh	= (110500 * 16),
		.type		= V4L2_TUNER_RADIO,
	},
};

static void radcap_set_fm_audmode(struct radcap_dev *dev)
{
	/* Set all tuners to Mono in dual tuner mode regardless of fm_audmode_mask */
	if (dev->dual)
		ctrl_iowrite32(dev, RADCAP_FM_AUDIO_MODE_REG, RADCAP_FM_AUDIO_MONO);
	else
		ctrl_iowrite32(dev, RADCAP_FM_AUDIO_MODE_REG, dev->fm_audmode_mask);
}

void radcap_set_freq(struct radcap_dev *dev, uint32_t tuner)
{
	int32_t freq = dev->freq[tuner]; /* Signed */

	switch (dev->board) {
	case CARD_AM:
		freq = clamp_t(int32_t, freq, tuners[TUNER_AM].rangelow,
			       tuners[TUNER_AM].rangehigh);
		dev->tuner_cmd[tuner] = (tuner << 16) | am_freq_to_val(freq / 16);
		break;
	case CARD_FM:
		freq = clamp_t(int32_t, freq, tuners[TUNER_FM].rangelow,
			       tuners[TUNER_FM].rangehigh);
		dev->tuner_cmd[tuner] = (tuner << 16) | fm_freq_to_val(freq / 16);
		break;
	}
	dev->freq[tuner] = freq;
	ctrl_iowrite32(dev, RADCAP_TUNER_FREQ_SET_REG, dev->tuner_cmd[tuner]);
}

/* Last config step */
void radcap_set_demod(struct radcap_dev *dev, bool status)
{
	uint32_t cmd = RADCAP_DEMOD_STOP;

	if (status == RADCAP_DEMOD_OFF)
		goto write_cmd;

	switch (dev->board) {
	case CARD_AM:
		cmd += dev->narrow ? RADCAP_AM_AUDIO_NARROW :
				     RADCAP_AM_AUDIO_WIDE;
		cmd += dev->dual ? RADCAP_AM_TUNER_DUAL :
				   RADCAP_AM_TUNER_SINGLE;
		break;
	case CARD_FM: {
		uint32_t fm_bw = dev->narrow ? RADCAP_FM_AUDIO_NARROW :
					       RADCAP_FM_AUDIO_WIDE;

		radcap_set_fm_audmode(dev);
		ctrl_iowrite32(dev, RADCAP_FM_AUDIO_BW_REG, fm_bw);
		cmd += dev->dual ? RADCAP_FM_TUNER_DUAL : RADCAP_FM_TUNER_SINGLE;
		cmd += (dev->fm_deemph == V4L2_DEEMPHASIS_50_uS) ?
					  RADCAP_FM_DEEMPHASIS_50_uS :
					  RADCAP_FM_DEEMPHASIS_75_uS;
#ifdef RADCAP_FM_RDS
		cmd += dev->fm_rds ? RADCAP_FM_RDS_ENABLE : RADCAP_FM_RDS_DISABLE;
#endif /* RADCAP_FM_RDS */
		break;
	}
	}

write_cmd:
	mb(); /* Enforce ordering, especially during resume */
	ctrl_iowrite32(dev, RADCAP_DEMOD_SET_REG, cmd);
}

static int radio_querycap(struct file *file, void __always_unused *priv,
			  struct v4l2_capability *cap)
{
	struct radcap_dev *dev = video_drvdata(file);

	snprintf(cap->bus_info, sizeof(cap->bus_info), "PCI:%s", pci_name(dev->pdev));
	strscpy(cap->card, RADCAP_DRV_NAME, sizeof(cap->card));
	strscpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
	return 0;
}

static int radio_g_tuner(struct file *file, void __always_unused *priv,
			 struct v4l2_tuner *t)
{
	struct radcap_dev *dev = video_drvdata(file);
	u32 tuner = t->index;

	if (tuner >= dev->nodes)
		return -EINVAL;

	switch (dev->board) {
	case CARD_AM:
		*t = tuners[TUNER_AM];
		t->index = tuner;
		t->signal = min_t(u32, ctrl_ioread32(dev,
				  RADCAP_AM_RSSI_REG + tuner) * 15, RFSIGMAX);
		break;
	case CARD_FM:
		*t = tuners[TUNER_FM];
		t->index = tuner;
		t->signal = min_t(u32, ioread16((int16_t __iomem *)(dev->ctrl +
				  RADCAP_FM_RSSI_REG) + tuner) * 30, RFSIGMAX);
		t->rxsubchans = (ioread16((int16_t __iomem *)(dev->ctrl +
				 RADCAP_FM_PILOT_REG) + tuner) > NOPILOTVAL) ?
				 V4L2_TUNER_SUB_STEREO : V4L2_TUNER_SUB_MONO;
		if (dev->fm_audmode_set[tuner] == RADCAP_FM_AUDMODE_SET)
			t->audmode = dev->fm_audmode[tuner]; /* Restore tuner mode */
		break;
	}
	snprintf(t->name, sizeof(t->name), "tuner %u", t->index);
	v4l2_dbg(1, dev->debug, dev,
		 "%s, audmode = %u, signal = %d, rxsubchans = %u\n",
		 t->name, t->audmode, t->signal, t->rxsubchans);
	return 0;
}

static int radio_s_tuner(struct file *file, void __always_unused *priv,
			 const struct v4l2_tuner *t)
{
	struct radcap_dev *dev = video_drvdata(file);
	u32 tuner = t->index;

	if (tuner >= dev->nodes)
		return -EINVAL;

	if (dev->board == CARD_FM) {
		if (t->audmode == V4L2_TUNER_MODE_MONO) {
			dev->fm_audmode_mask |= BIT(tuner);
			dev->fm_audmode[tuner] = V4L2_TUNER_MODE_MONO; /* Save tuner mode */
		} else {
			dev->fm_audmode_mask &= ~(BIT(tuner));
			dev->fm_audmode[tuner] = V4L2_TUNER_MODE_STEREO; /* Save tuner mode */
		}
		radcap_set_fm_audmode(dev);
		dev->fm_audmode_set[tuner] = RADCAP_FM_AUDMODE_SET; /* Mark as changed */
	}
	return 0;
}

static int radio_g_frequency(struct file *file, void __always_unused *priv,
			     struct v4l2_frequency *f)
{
	struct radcap_dev *dev = video_drvdata(file);
	u32 tuner = f->tuner;

	if (tuner >= dev->nodes)
		return -EINVAL;

	f->frequency = dev->freq[tuner];
	v4l2_dbg(1, dev->debug, dev,
		 "tuner %u, freq = %u kHz, tuner_cmd = 0x%x\n",
		 tuner, f->frequency / 16, dev->tuner_cmd[tuner]);
	return 0;
}

static int radio_s_frequency(struct file *file, void __always_unused *priv,
			     const struct v4l2_frequency *f)
{
	struct radcap_dev *dev = video_drvdata(file);
	u32 tuner = f->tuner;

	if (tuner >= dev->nodes)
		return -EINVAL;

	dev->freq[tuner] = f->frequency;
	radcap_set_freq(dev, tuner);
	return 0;
}

static const struct v4l2_file_operations radcap_fops = {
	.owner		= THIS_MODULE,
	.open		= v4l2_fh_open,
	.release	= v4l2_fh_release,
	.poll		= v4l2_ctrl_poll,
	.unlocked_ioctl	= video_ioctl2,
};

static const struct v4l2_ioctl_ops radcap_ioctl_ops = {
	.vidioc_querycap		= radio_querycap,
	.vidioc_g_tuner			= radio_g_tuner,
	.vidioc_s_tuner			= radio_s_tuner,
	.vidioc_g_frequency		= radio_g_frequency,
	.vidioc_s_frequency		= radio_s_frequency,
	.vidioc_log_status		= v4l2_ctrl_log_status,
	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

const struct video_device radcap_radio_template = {
	.device_caps	= (V4L2_CAP_TUNER | V4L2_CAP_RADIO),
	.fops		= &radcap_fops,
	.ioctl_ops	= &radcap_ioctl_ops,
	.name		= RADCAP_V4L2_NAME,
	.release	= video_device_release_empty,
};

static int radio_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct radcap_dev *dev = container_of(ctrl->handler, struct radcap_dev,
					      ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_RADCAP_DUAL:
		dev->dual = ctrl->val;
		break;
	case V4L2_CID_RADCAP_NARROW:
		dev->narrow = ctrl->val;
		break;
	case V4L2_CID_TUNE_DEEMPHASIS:
		dev->fm_deemph = ctrl->val;
		break;
#ifdef RADCAP_FM_RDS
	case V4L2_CID_RDS_RECEPTION:
		dev->fm_rds = ctrl->val;
		break;
#endif /* RADCAP_FM_RDS */
	default:
		return -EINVAL;
	}
	radcap_set_demod(dev, RADCAP_DEMOD_ON);
	return 0;
}

const struct v4l2_ctrl_ops radcap_ctrl_ops = {
	.s_ctrl	= radio_s_ctrl,
};

const struct v4l2_ctrl_config radcap_ctrl_dual = {
	.ops	= &radcap_ctrl_ops,
	.id	= V4L2_CID_RADCAP_DUAL,
	.name	= "Dual Tuner",
	.type	= V4L2_CTRL_TYPE_BOOLEAN,
	.min	= 0,
	.max	= 1,
	.step	= 1,
	.def	= 0,
};

const struct v4l2_ctrl_config radcap_ctrl_narrow = {
	.ops	= &radcap_ctrl_ops,
	.id	= V4L2_CID_RADCAP_NARROW,
	.name	= "Narrow Audio",
	.type	= V4L2_CTRL_TYPE_BOOLEAN,
	.min	= 0,
	.max	= 1,
	.step	= 1,
	.def	= 0,
};
