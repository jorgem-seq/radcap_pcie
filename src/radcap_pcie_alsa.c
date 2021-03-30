// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * radcap_pcie_alsa.c - Sonifex Radcap PCIe ALSA Driver
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

/*
 * Audio capture inputs are subdevices of card X, device 0:
 * hw:X,0,0, hw:X,0,1, hw:X,0,2... hw:X,0,31.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <sound/control.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/tlv.h>

#include "radcap_pcie.h"

MODULE_AUTHOR("Jorge Maidana <jorgem.seq@gmail.com>");
MODULE_DESCRIPTION("Sonifex Radcap PCIe ALSA Driver");
MODULE_LICENSE("GPL v2");

#define	RADCAP_ALSA_NAME		"Radcap PCIe ALSA"
#define	RADCAP_MIXER_NAME		"Radcap PCIe MIXER"
#define	RADCAP_PCM_NAME			"Radcap PCIe PCM"

#define RADCAP_BUFFER_SIZE		SZ_128K
#define RADCAP_PERIOD_BYTES_MIN		64

#define RADCAP_MIN_VOL			0
#define RADCAP_DEF_VOL			BIT(15)
#define RADCAP_MAX_VOL			GENMASK(15, 0)

#define SGTABLE_VAL			BIT(10) /* From first to last - 1 */
#define SGTABLE_END			GENMASK(11, 2) /* Last entry */

#define dprintk(level, fmt, ...)				\
	do {							\
		if (debug >= (level))				\
			radcap_pr_info(fmt, ##__VA_ARGS__);	\
	} while (0)

static struct snd_card *snd_radcap_cards[SNDRV_CARDS];

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX; /* Index 0-MAX */
module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "Index value for the audio card, comma separated for each card.");

static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR; /* ID for this card */
module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string for the audio card, comma separated for each card.");

static bool enable[SNDRV_CARDS] = SNDRV_DEFAULT_ENABLE_PNP; /* Enable switches */
module_param_array(enable, bool, NULL, 0444);
MODULE_PARM_DESC(enable, "Enable the audio card, comma separated for each card.");

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Set the debug level.");

struct snd_card_radcap {
	spinlock_t			slock;
	struct	mutex			mlock;
	struct	radcap_dev		*dev;
	struct	snd_card		*card;
	struct	pci_dev			*pdev;
	struct	snd_pcm_substream	*radcap_pcm_substream[RADCAP_MAX_NODES];
	unsigned int			irq;
	uint32_t			radcap_pcm_status[RADCAP_MAX_NODES];
	uint16_t			mixer_volume[RADCAP_MAX_NODES][2];
};

struct snd_card_radcap_pcm {
	struct	radcap_dev		*dev;
};

static const struct snd_pcm_hardware snd_radcap_hw = {
	.info			= (SNDRV_PCM_INFO_MMAP |
				   SNDRV_PCM_INFO_MMAP_VALID |
				   SNDRV_PCM_INFO_INTERLEAVED |
				   SNDRV_PCM_INFO_BLOCK_TRANSFER |
				   SNDRV_PCM_INFO_SYNC_START),
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.rates			= (SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_48000),
	.rate_min		= 22050,
	.rate_max		= 48000,
	.channels_min		= 2,
	.channels_max		= 2,
	.buffer_bytes_max	= RADCAP_BUFFER_SIZE,
	.period_bytes_min	= RADCAP_PERIOD_BYTES_MIN,
	.period_bytes_max	= RADCAP_BUFFER_SIZE,
	.periods_min		= 1,
	.periods_max		= (RADCAP_BUFFER_SIZE / RADCAP_PERIOD_BYTES_MIN),
	.fifo_size		= 0,
};

static void snd_radcap_pcm_runtime_free(struct snd_pcm_runtime *runtime)
{
	struct snd_card_radcap_pcm *pcm = runtime->private_data;

	kfree(pcm);
}

static int snd_radcap_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_radcap *chip = snd_pcm_substream_chip(substream);
	struct snd_card_radcap_pcm *pcm;
	struct radcap_dev *dev;
	int source = substream->number;
	unsigned long flags;

	if (!chip)
		return -ENODEV;

	dev = chip->dev;

	pcm = kzalloc(sizeof(*pcm), GFP_KERNEL);
	if (!pcm)
		return -ENOMEM;

	pcm->dev = dev;

	spin_lock_irqsave(&chip->slock, flags);
	chip->radcap_pcm_substream[source] = substream;
	spin_unlock_irqrestore(&chip->slock, flags);

	runtime->private_data = pcm;
	runtime->private_free = snd_radcap_pcm_runtime_free;
	runtime->hw = snd_radcap_hw;

	switch (dev->board) {
	case CARD_AM:
		runtime->hw.rates = SNDRV_PCM_RATE_22050;
		runtime->hw.rate_max = 22050;
		break;
	case CARD_FM:
		runtime->hw.rates = SNDRV_PCM_RATE_48000;
		runtime->hw.rate_min = 48000;
		break;
	}
	return 0;
}

static int snd_radcap_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_card_radcap *chip = snd_pcm_substream_chip(substream);
	int source = substream->number;
	unsigned long flags;

	spin_lock_irqsave(&chip->slock, flags);
	chip->radcap_pcm_substream[source] = NULL;
	spin_unlock_irqrestore(&chip->slock, flags);
	return 0;
}

static int snd_radcap_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_radcap *chip = snd_pcm_substream_chip(substream);
	struct radcap_dev *dev = chip->dev;
	int source = substream->number;
	size_t ofs, size = runtime->dma_bytes;

	/* Set vol, enable audio */
	mutex_lock(&chip->mlock);
	ctrl_iowrite32(dev, dev->card.vol_reg + source,
		       chip->mixer_volume[source][0] |
		       ((uint32_t)chip->mixer_volume[source][1] << 16));
	mutex_unlock(&chip->mlock);

	/* SGDMA configuration */
	for (ofs = 0; ofs * PAGE_SIZE < size; ofs++) {
		uint64_t sgval = snd_pcm_sgbuf_get_addr(substream, ofs * PAGE_SIZE);

		if (size - ofs * PAGE_SIZE > PAGE_SIZE)
			sgval |= SGTABLE_VAL;
		else
			sgval |= ((size - ofs * PAGE_SIZE) & SGTABLE_END) >> 2;

		sg_iowrite64(dev, (uint64_t)(ofs + source * 64), sgval);
		dprintk(2, "reg = 0x%llx, val = 0x%llx\n",
		       (u64)(ofs + source * 64), (u64)sgval);
	}
#ifdef RADCAP_PCM_BUFFERS
	ctrl_iowrite32(dev, RADCAP_PCM_BUF_REG, runtime->period_size * RADCAP_PCM_BUF_NUM);
#endif /* RADCAP_PCM_BUFFERS */
	dprintk(2, "Substream %d configured\n", source);
	return 0;
}

static int snd_radcap_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_card_radcap *chip = snd_pcm_substream_chip(substream);
	struct radcap_dev *dev = chip->dev;
	unsigned long flags;
	int source = substream->number;
	int ret = 0;

	spin_lock_irqsave(&chip->slock, flags);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		chip->radcap_pcm_status[source] = RADCAP_PCM_RUN;
		ctrl_iowrite32(dev, dev->card.ctrl_reg + source,
			       chip->radcap_pcm_status[source]);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		chip->radcap_pcm_status[source] = RADCAP_PCM_STOP;
		ctrl_iowrite32(dev, dev->card.ctrl_reg + source,
			       chip->radcap_pcm_status[source]);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	spin_unlock_irqrestore(&chip->slock, flags);
	return ret;
}

static snd_pcm_uframes_t
snd_radcap_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_card_radcap_pcm *pcm = runtime->private_data;
	struct radcap_dev *dev = pcm->dev;
	snd_pcm_uframes_t pointer = 0;
	int source = substream->number;

	pointer = ctrl_ioread32(dev, dev->card.ctrl_reg + source);
	BUG_ON(pointer == IOREAD32_ERR);
	pointer = bytes_to_frames(runtime, pointer);
	if (pointer >= runtime->buffer_size)
		pointer = 0;
	dprintk(3, "pointer = 0x%llx, buffer_size = %llu\n",
	       (u64)pointer, (u64)runtime->buffer_size);
	return pointer;
}

static const struct snd_pcm_ops snd_radcap_pcm_ops = {
	.open		= snd_radcap_pcm_open,
	.close		= snd_radcap_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.prepare	= snd_radcap_pcm_prepare,
	.trigger	= snd_radcap_pcm_trigger,
	.pointer	= snd_radcap_pcm_pointer,
};

static int snd_radcap_pcm_create(struct snd_card_radcap *chip)
{
	struct snd_pcm *pcm;
	int ret;

	ret = snd_pcm_new(chip->card, RADCAP_PCM_NAME, 0, 0, chip->dev->nodes, &pcm);
	if (ret < 0)
		return ret;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &snd_radcap_pcm_ops);
	pcm->private_data = chip;
	pcm->info_flags = 0;
	strscpy(pcm->name, RADCAP_PCM_NAME, sizeof(pcm->name));
	snd_pcm_set_managed_buffer_all(pcm, SNDRV_DMA_TYPE_DEV_SG,
				       &chip->pdev->dev,
				       RADCAP_BUFFER_SIZE, RADCAP_BUFFER_SIZE);
	return 0;
}

static irqreturn_t snd_radcap_irq_handler(int __always_unused irq, void *dev_id)
{
	struct snd_card_radcap *chip = dev_id;
	struct radcap_dev *dev = chip->dev;
	int source;
	uint32_t status;

	spin_lock(&chip->slock);
	status = ctrl_ioread32(dev, RADCAP_IRQ_STATUS);
	if (status != RADCAP_IRQ_PCM) {
		spin_unlock(&chip->slock);
		return IRQ_NONE;
	}

	for (source = 0; source < dev->nodes; source++) {
		if (chip->radcap_pcm_substream[source] &&
		   (chip->radcap_pcm_status[source] == RADCAP_PCM_RUN)) {
			spin_unlock(&chip->slock);
			snd_pcm_period_elapsed(chip->radcap_pcm_substream[source]);
			spin_lock(&chip->slock);
		}
	}
	spin_unlock(&chip->slock);
	return IRQ_HANDLED;
}

static int snd_radcap_mixer_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_card_radcap *chip = snd_kcontrol_chip(kcontrol);
	u32 source = kcontrol->id.index;

	mutex_lock(&chip->mlock);
	ucontrol->value.integer.value[0] = chip->mixer_volume[source][0];
	ucontrol->value.integer.value[1] = chip->mixer_volume[source][1];
	mutex_unlock(&chip->mlock);
	return 0;
}

static int snd_radcap_mixer_info(struct snd_kcontrol __always_unused *kcontrol,
				 struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = RADCAP_MIN_VOL;
	uinfo->value.integer.max = RADCAP_MAX_VOL;
	return 0;
}

static int snd_radcap_mixer_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_card_radcap *chip = snd_kcontrol_chip(kcontrol);
	struct radcap_dev *dev = chip->dev;
	uint16_t left, right;
	int change = 0;
	u32 source = kcontrol->id.index;

	left = clamp_t(uint16_t, ucontrol->value.integer.value[0],
		       RADCAP_MIN_VOL, RADCAP_MAX_VOL);
	right = clamp_t(uint16_t, ucontrol->value.integer.value[1],
			RADCAP_MIN_VOL, RADCAP_MAX_VOL);

	mutex_lock(&chip->mlock);
	if ((chip->mixer_volume[source][0] != left) ||
	    (chip->mixer_volume[source][1] != right)) {
		chip->mixer_volume[source][0] = left;
		chip->mixer_volume[source][1] = right;
		ctrl_iowrite32(dev, dev->card.vol_reg + source,
			       chip->mixer_volume[source][0] |
			       ((uint32_t)chip->mixer_volume[source][1] << 16));
		change = 1;
	}
	mutex_unlock(&chip->mlock);
	return change;
}

static const DECLARE_TLV_DB_LINEAR(snd_radcap_mixer_linear, TLV_DB_GAIN_MUTE, 600);

static struct snd_kcontrol_new snd_radcap_mixer_new = {
	.iface	= SNDRV_CTL_ELEM_IFACE_MIXER,
	.access	= (SNDRV_CTL_ELEM_ACCESS_READWRITE |
		   SNDRV_CTL_ELEM_ACCESS_TLV_READ),
	.name	= "RAD",
	.info	= snd_radcap_mixer_info,
	.get	= snd_radcap_mixer_get,
	.put	= snd_radcap_mixer_put,
	.tlv.p	= snd_radcap_mixer_linear,
};

static int snd_radcap_mixer_create(struct snd_card_radcap *chip)
{
	struct snd_card *card = chip->card;
	struct radcap_dev *dev = chip->dev;
	struct snd_kcontrol *kcontrol;
	int idx, ret;

	strscpy(card->mixername, RADCAP_MIXER_NAME, sizeof(card->mixername));

	for (idx = 0; idx < dev->nodes; idx++) {
		mutex_lock(&chip->mlock);
		chip->mixer_volume[idx][0] = RADCAP_DEF_VOL;
		chip->mixer_volume[idx][1] = RADCAP_DEF_VOL;
		mutex_unlock(&chip->mlock);
		snd_radcap_mixer_new.index = idx;
		kcontrol = snd_ctl_new1(&snd_radcap_mixer_new, chip);
		ret = snd_ctl_add(card, kcontrol);
		if (ret < 0)
			return ret;
	}
	return 0;
}

/* Free Card */
static void snd_radcap_card_free(struct snd_card *card)
{
	struct snd_card_radcap *chip = card->private_data;

	free_irq(chip->irq, chip);
}

/* Create Card */
static int snd_radcap_card_create(struct radcap_dev *dev, int devnum)
{
	struct snd_card *card;
	struct snd_card_radcap *chip;
	int ret;

	if (devnum >= SNDRV_CARDS)
		return -ENODEV;

	if (!enable[devnum]) {
		radcap_pr_info("audio disabled\n");
		return -ENODEV;
	}

	ret = snd_card_new(&dev->pdev->dev, index[devnum], id[devnum],
			   THIS_MODULE, sizeof(*chip), &card);
	if (ret < 0)
		return ret;

	card->private_free = snd_radcap_card_free;
	chip = card->private_data;
	chip->dev = dev;
	chip->card = card;
	chip->pdev = dev->pdev;

	spin_lock_init(&chip->slock);
	mutex_init(&chip->mlock);

	ret = request_irq(chip->pdev->irq, snd_radcap_irq_handler, IRQF_SHARED,
			  KBUILD_MODNAME, chip);
	if (ret) {
		radcap_pr_err("failed to request IRQ\n");
		goto free_snd;
	}

	chip->irq = chip->pdev->irq;
	synchronize_irq(chip->irq);

	ret = snd_radcap_mixer_create(chip);
	if (ret < 0)
		goto free_irq;

	ret = snd_radcap_pcm_create(chip);
	if (ret < 0)
		goto free_irq;

	strscpy(card->driver, KBUILD_MODNAME, sizeof(card->driver));
	strscpy(card->shortname, RADCAP_DRV_NAME, sizeof(card->shortname));
	snprintf(card->longname, sizeof(card->longname), "%s irq %u id %llx",
		 card->shortname, chip->pdev->irq, (u64)chip->dev->fpga_id);

	ret = snd_card_register(card);
	if (ret < 0)
		goto free_irq;

	snd_radcap_cards[devnum] = card;
	return 0;

free_irq:
	free_irq(chip->irq, chip);
free_snd:
	snd_card_free(card);
	return ret;
}

static int radcap_pcie_alsa_init(void)
{
	struct radcap_dev *dev = NULL;
	struct list_head *list;

	pr_info(KBUILD_MODNAME ": " RADCAP_ALSA_NAME " driver loaded\n");

	list_for_each(list, &radcap_devlist) {
		dev = list_entry(list, struct radcap_dev, devlist);
		snd_radcap_card_create(dev, dev->nr);
	}
	if (!dev)
		pr_err(KBUILD_MODNAME ": no sound cards found\n");
	return 0;
}

static void radcap_pcie_alsa_exit(void)
{
	int idx;

	for (idx = 0; idx < SNDRV_CARDS; idx++) {
		if (snd_radcap_cards[idx]) {
			snd_card_free(snd_radcap_cards[idx]);
			snd_radcap_cards[idx] = NULL;
		}
	}
	pr_info(KBUILD_MODNAME ": " RADCAP_ALSA_NAME " driver unloaded\n");
}

late_initcall(radcap_pcie_alsa_init);
module_exit(radcap_pcie_alsa_exit);
