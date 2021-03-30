/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * radcap_pcie.h - Sonifex Radcap PCIe Driver
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

#ifndef RADCAP_PCIE_H
#define RADCAP_PCIE_H

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <linux/io-64-nonatomic-lo-hi.h>

#define RADCAP_DRV_NAME			"Radcap PCIe"
#define RADCAP_MAX_CARDS		32

/* IO */
#define RADCAP_CTRL_BAR			0
#define RADCAP_SG_BAR			2

/* Tuners + ALSA Subdevices */
#define RADCAP_NODES_VAL_REG		0x00

#define RADCAP_MIN_NODES		6
#define RADCAP_MAX_NODES		32

/* AM */
#define RADCAP_AM_CTRL_REG		0x40
#define RADCAP_AM_VOL_REG		0x60
#define RADCAP_AM_FPGA_ID_REG		0x04
#define RADCAP_AM_KEY_REG		0x05
#define RADCAP_AM_RSSI_REG		0x60

#define RADCAP_AM_AUDIO_WIDE		0x11  /* Wide Audio */
#define RADCAP_AM_AUDIO_NARROW		0x15  /* Narrow Audio */
#define RADCAP_AM_TUNER_SINGLE		0x00  /* 1 Tuner per ALSA Subdevice 0, 1, 2... */
#define RADCAP_AM_TUNER_DUAL		0x08  /* 2 Tuners per ALSA Subdevice 0, 2, 4... */

/* FM */
#define RADCAP_FM_CTRL_REG		0x20
#define RADCAP_FM_VOL_REG		0x40
#define RADCAP_FM_FPGA_ID_REG		0x06
#define RADCAP_FM_KEY_REG		0x02
#define RADCAP_FM_RSSI_REG		0x10
#define RADCAP_FM_AUDIO_MODE_REG	0x06  /* Mono or Stereo */
#define RADCAP_FM_AUDIO_BW_REG		0x07  /* Narrow or Wide */
#define RADCAP_FM_PILOT_REG		0x70  /* Detect Stereo */

/*
 * A dedicated interrupt handler is required for RDS.
 * The initialization code is similar to snd_radcap_pcm_prepare,
 * using lo_hi_writeq(val, dev->ctrl + reg).
 */
//#define RADCAP_FM_RDS			      /* Unfinished RDS */
#undef RADCAP_FM_RDS
#ifdef RADCAP_FM_RDS
# define RADCAP_FM_RDS_REG		0x04  /* RDS Stream */
#endif /* RADCAP_FM_RDS */

#define RADCAP_FM_AUDIO_WIDE		0x00  /* Wide Audio */
#define RADCAP_FM_AUDIO_NARROW		(~0u) /* Narrow Audio */
#define RADCAP_FM_AUDIO_STEREO		0x00  /* Stereo Audio */
#define RADCAP_FM_AUDIO_MONO		(~0u) /* Mono Audio */
#define RADCAP_FM_AUDMODE_UNSET		0
#define RADCAP_FM_AUDMODE_SET		1     /* Save state */
#define RADCAP_FM_TUNER_SINGLE		0x00  /* 1 Tuner per ALSA Subdevice 0, 1, 2... */
#define RADCAP_FM_TUNER_DUAL		0x04  /* 2 Tuners per ALSA Subdevice 0, 2, 4... */
#define RADCAP_FM_DEEMPHASIS_50_uS	0x30  /* 50us */
#define RADCAP_FM_DEEMPHASIS_75_uS	0x70  /* 75us */
#ifdef RADCAP_FM_RDS
# define RADCAP_FM_RDS_ENABLE		0x03  /* Enable RDS Interrupts */
# define RADCAP_FM_RDS_DISABLE		0x00  /* Disable RDS Interrupts */
#endif /* RADCAP_FM_RDS */

/* ALSA */
#define RADCAP_PCM_RUN			0x03
#define RADCAP_PCM_STOP			0x00
//#define RADCAP_PCM_BUFFERS		      /* Not needed, preconfigured value is OK */
#undef RADCAP_PCM_BUFFERS
#ifdef RADCAP_PCM_BUFFERS
# define RADCAP_PCM_BUF_REG		0x04  /* Change the number of HW Buffers */
# define RADCAP_PCM_BUF_NUM		16    /* Buffers number */
#endif /* RADCAP_PCM_BUFFERS */

/* V4L2 */
#define RADCAP_DEMOD_SET_REG		0x00
#define RADCAP_TUNER_FREQ_SET_REG	0x01

#define RADCAP_DEMOD_STOP		0x00
#define RADCAP_DEMOD_OFF		0
#define RADCAP_DEMOD_ON			1
#define RADCAP_HW_RESUME		0
#define RADCAP_HW_START			1

/* IRQ */
#define RADCAP_IRQ_STATUS		0x01
#define RADCAP_IRQ_PCM			0x01

enum {
	CARD_AM,
	CARD_FM,
};

struct radcap_card {
	uint32_t			ctrl_reg;
	uint32_t			fpga_id_reg;
	uint32_t			fpga_key_reg;
	uint32_t			vol_reg;
	char				band[8];
};

struct radcap_dev {
	struct	list_head		devlist;
	struct	work_struct		request_module_wk;
	struct	radcap_card		card;
	uint64_t			fpga_id;
	uint32_t			card_nodes; /* Board val */
	int				nodes; /* User defined */
	int				board;
	int				debug;
	int				nr;
	char				name[32];

	/* IO */
	struct	pci_dev			*pdev;
	uint64_t	__iomem		*sg;
	uint32_t	__iomem		*ctrl;

	/* V4L2 */
	struct	v4l2_device		v4l2_dev;
	struct	v4l2_ctrl_handler	ctrl_handler;
	struct	video_device		rdev;
	struct	mutex			rdev_mlock;
	uint32_t			tuner_cmd[RADCAP_MAX_NODES];
	u32				freq[RADCAP_MAX_NODES]; /* V4L2 freq */
	bool				dual;
	bool				narrow;

	/* V4L2 FM */
	uint32_t			fm_audmode_mask; /* See radio_s_tuner */
	u32				fm_deemph;
	u32				fm_audmode[RADCAP_MAX_NODES]; /* V4L2 FM audio mode */
	bool				fm_audmode_set[RADCAP_MAX_NODES]; /* 1 after first change */
#ifdef RADCAP_FM_RDS
	bool				fm_rds;
#endif /* RADCAP_FM_RDS */
};

/* Helpers */

/* Card disconnection or bad host */
#define IOREAD32_ERR			(~0u)

static inline uint32_t ctrl_ioread32(struct radcap_dev *dev, uint32_t reg)
{
	return ioread32(dev->ctrl + (reg));
}

static inline void ctrl_iowrite32(struct radcap_dev *dev, uint32_t reg, uint32_t val)
{
	iowrite32((val), dev->ctrl + (reg));
	BUG_ON(ctrl_ioread32(dev, RADCAP_NODES_VAL_REG) == IOREAD32_ERR);
}

static inline void sg_iowrite64(struct radcap_dev *dev, uint64_t reg, uint64_t val)
{
	iowrite64((val), dev->sg + (reg));
}

#define radcap_pr_info(fmt, ...)	pr_info(KBUILD_MODNAME " %s: " fmt,		\
						pci_name(dev->pdev), ##__VA_ARGS__)
#define radcap_pr_err(fmt, ...)		pr_err(KBUILD_MODNAME " %s: " fmt,		\
						pci_name(dev->pdev), ##__VA_ARGS__)

/* radcap_pcie_core.c */
extern struct list_head radcap_devlist;
extern struct mutex radcap_devlist_lock;

/* radcap_pcie_v4l2.c */
extern const struct v4l2_ctrl_ops radcap_ctrl_ops;
extern const struct v4l2_ctrl_config radcap_ctrl_narrow;
extern const struct v4l2_ctrl_config radcap_ctrl_dual;
extern const struct video_device radcap_radio_template;

void radcap_set_demod(struct radcap_dev *dev, bool status);
void radcap_set_freq(struct radcap_dev *dev, uint32_t tuner);

#endif
