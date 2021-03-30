// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * radcap_pcie_core.c - Sonifex Radcap PCIe V4L2 Driver
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
 * Only one V4L2 device node is created per card, /dev/radioX, with a
 * maximum of 32 simultaneous Tuners and ALSA Subdevices.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/videodev2.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>

#include "radcap_pcie.h"

MODULE_AUTHOR("Jorge Maidana <jorgem.seq@gmail.com>");
MODULE_DESCRIPTION("Sonifex Radcap PCIe V4L2 driver");
MODULE_LICENSE("GPL v2");

#define PCI_VENDOR_ID_SONIFEX			0x1bc9
#define PCI_DEVICE_ID_SONIFEX_RADCAP_PCIE_AM	0x3000
#define PCI_DEVICE_ID_SONIFEX_RADCAP_PCIE_FM	0x3101

LIST_HEAD(radcap_devlist);
EXPORT_SYMBOL(radcap_devlist);

DEFINE_MUTEX(radcap_devlist_lock);
EXPORT_SYMBOL(radcap_devlist_lock);

static int radcap_devcount;

/* Unlock more nodes (Tuner + ALSA Subdevice) */
static char *key[RADCAP_MAX_CARDS];
static int key_count;
module_param_array(key, charp, &key_count, 0);
MODULE_PARM_DESC(key, "'FPGA_ID:KEY' string, comma separated for each card.");

static int nodes[RADCAP_MAX_CARDS];
module_param_array(nodes, int, NULL, 0);
MODULE_PARM_DESC(nodes, "Set the number of enabled nodes, comma separated for each card.");

static int debug;
module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "Set the debug level.");

#if defined(CONFIG_MODULES) && defined(MODULE)
static void request_module_async(struct work_struct __always_unused *work)
{
	request_module("Radcap_PCIe_ALSA");
}

static void request_modules(struct radcap_dev *dev)
{
	INIT_WORK(&dev->request_module_wk, request_module_async);
	schedule_work(&dev->request_module_wk);
}

static void flush_request_modules(struct radcap_dev *dev)
{
	flush_work(&dev->request_module_wk);
}
#else
#define request_modules(dev)
#define flush_request_modules(dev)
#endif /* CONFIG_MODULES */

static const struct radcap_card radcap_cards[] = {
	[CARD_AM] = {
		.band		= "AM",
		.ctrl_reg	= RADCAP_AM_CTRL_REG,
		.fpga_id_reg	= RADCAP_AM_FPGA_ID_REG,
		.fpga_key_reg	= RADCAP_AM_KEY_REG,
		.vol_reg	= RADCAP_AM_VOL_REG,
	},
	[CARD_FM] = {
		.band		= "FM",
		.ctrl_reg	= RADCAP_FM_CTRL_REG,
		.fpga_id_reg	= RADCAP_FM_FPGA_ID_REG,
		.fpga_key_reg	= RADCAP_FM_KEY_REG,
		.vol_reg	= RADCAP_FM_VOL_REG,
	},
};

static void radcap_hw_write_key(struct radcap_dev *dev)
{
	int i;

	/* If fpga_id matches write fpga_key to enable more nodes */
	for (i = 0; i < key_count; i++) {
		int read;
		uint32_t fpga_key = 0;
		u64 fpga_id = 0;

		read = sscanf(key[i], "%15llx:%u", &fpga_id, &fpga_key);
		if ((read == 2) && (fpga_id == (u64)dev->fpga_id))
			ctrl_iowrite32(dev, dev->card.fpga_key_reg, fpga_key);
	}
}

static int radcap_hw_setup(struct radcap_dev *dev)
{
	int i;

	/* Get the Xilinx Spartan-6 FPGA Unique Device Identifier */
	for (i = 0; i < 32; i++) {
		dev->fpga_id = lo_hi_readq(dev->ctrl + dev->card.fpga_id_reg);
		udelay(1);
	}
	if ((dev->fpga_id >> 32) == IOREAD32_ERR)
		return -ENODEV;

	/* Write the key */
	if (key_count) {
		radcap_hw_write_key(dev);
		mb(); /* Ensure the key is written */
	}

	/* Wait for the value to be updated and get the number of nodes */
	for (i = 0; i < 32; i++) {
		dev->card_nodes = ctrl_ioread32(dev, RADCAP_NODES_VAL_REG);
		udelay(1);
	}
	if ((dev->card_nodes < RADCAP_MIN_NODES) || (dev->card_nodes > RADCAP_MAX_NODES))
		return -ENODEV;
	dev->nodes = dev->card_nodes;

	/* Define the number of nodes used */
	if ((nodes[dev->nr] > 0) && (nodes[dev->nr] <= dev->nodes))
		dev->nodes = nodes[dev->nr];
	return 0;
}

static void radcap_hw_reset(struct radcap_dev *dev, bool power)
{
	int node;

	for (node = 0; node < dev->nodes; node++) {
		ctrl_iowrite32(dev, dev->card.ctrl_reg + node, RADCAP_PCM_STOP);
		if (power == RADCAP_HW_START)
			dev->fm_audmode_set[node] = RADCAP_FM_AUDMODE_UNSET; /* Reset fm audmode on init */
		radcap_set_freq(dev, node);
		udelay(1);
	}
}

static int radcap_pcie_probe(struct pci_dev *pdev, const struct pci_device_id *pci_id)
{
	struct radcap_dev *dev;
	struct v4l2_device *v4l2_dev;
	struct v4l2_ctrl_handler *hdl;
	int ret;

	if (radcap_devcount >= RADCAP_MAX_CARDS)
		return -ENODEV;

	/* PCI */
	ret = pci_enable_device(pdev);
	if (ret)
		return ret;

	pci_set_master(pdev);

	if (pci_set_dma_mask(pdev, DMA_BIT_MASK(64))) {
		if (pci_set_dma_mask(pdev, DMA_BIT_MASK(32))) {
			dev_err(&pdev->dev, "no suitable DMA support available\n");
			ret = -EFAULT;
			goto disable_device;
		}
	}

	/* Allocate a new instance */
	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "can't allocate memory\n");
		goto disable_device;
	}

	dev->board = pci_id->driver_data;
	dev->card = radcap_cards[dev->board];
	dev->debug = debug;
	dev->nr = radcap_devcount;
	dev->pdev = pdev;

	ret = pci_request_regions(pdev, KBUILD_MODNAME);
	if (ret) {
		radcap_pr_err("can't request memory regions\n");
		goto disable_device;
	}

	dev->ctrl = pci_ioremap_bar(pdev, RADCAP_CTRL_BAR);
	if (!dev->ctrl) {
		ret = -ENODEV;
		radcap_pr_err("can't ioremap BAR %d\n", RADCAP_CTRL_BAR);
		goto free_pci_regions;
	}

	dev->sg = pci_ioremap_wc_bar(pdev, RADCAP_SG_BAR);
	if (!dev->sg) {
		ret = -ENODEV;
		radcap_pr_err("can't ioremap BAR %d\n", RADCAP_SG_BAR);
		goto free_ctrl_mmio;
	}

	/* Set initial values */
	dev->dual = 0; /* radcap_ctrl_dual */
	dev->narrow = 0; /* radcap_ctrl_narrow */
	dev->fm_deemph = V4L2_DEEMPHASIS_75_uS;
	dev->fm_audmode_mask = RADCAP_FM_AUDIO_STEREO; /* Set all tuners to Stereo */
#ifdef RADCAP_FM_RDS
	dev->fm_rds = 0;
#endif /* RADCAP_FM_RDS */
	snprintf(dev->name, sizeof(dev->name), RADCAP_DRV_NAME " [%d] [%s]",
		 dev->nr, dev->card.band);

	/* Get/Set nodes */
	ret = radcap_hw_setup(dev);
	if (ret) {
		radcap_pr_err("error when detecting nodes\n");
		goto free_sg_mmio;
	}

	/* Init nodes */
	radcap_hw_reset(dev, RADCAP_HW_START);
	radcap_set_demod(dev, RADCAP_DEMOD_ON);

	mutex_lock(&radcap_devlist_lock);
	list_add_tail(&dev->devlist, &radcap_devlist);
	mutex_unlock(&radcap_devlist_lock);

	/* Initialize the top-level structure */
	mutex_init(&dev->rdev_mlock);
	v4l2_dev = &dev->v4l2_dev;
	ret = v4l2_device_register(&pdev->dev, v4l2_dev);
	if (ret) {
		v4l2_err(v4l2_dev, "can't register V4L2 device\n");
		goto free_sg_mmio;
	}

	/* Add the controls */
	hdl = &dev->ctrl_handler;
	v4l2_ctrl_handler_init(hdl, 3);
	v4l2_ctrl_new_custom(hdl, &radcap_ctrl_narrow, NULL);

	/*
	 * Dual Tuner mode, 2 Tuners per ALSA Subdevice:
	 * tuner 0 -> left ch, tuner 1 -> right ch of hw:X,0,0, hw:X,0,1 muted,
	 * tuner 2 -> left ch, tuner 3 -> right ch of hw:X,0,2, hw:X,0,3 muted...
	 */
	if ((dev->nodes % 2) == 0)
		v4l2_ctrl_new_custom(hdl, &radcap_ctrl_dual, NULL);
	if (dev->board == CARD_FM) {
		v4l2_ctrl_new_std_menu(hdl, &radcap_ctrl_ops,
				       V4L2_CID_TUNE_DEEMPHASIS,
				       V4L2_DEEMPHASIS_75_uS, 1, dev->fm_deemph);
#ifdef RADCAP_FM_RDS
		v4l2_ctrl_new_std(hdl, &radcap_ctrl_ops,
				  V4L2_CID_RDS_RECEPTION, 0, 1, 1, dev->fm_rds);
#endif /* RADCAP_FM_RDS */
	}

	v4l2_dev->ctrl_handler = hdl;
	if (hdl->error) {
		ret = hdl->error;
		v4l2_err(v4l2_dev, "can't register V4L2 controls\n");
		goto free_v4l2;
	}

	/* Initialize the video_device structure */
	strscpy(v4l2_dev->name, dev->name, sizeof(v4l2_dev->name));
	dev->rdev = radcap_radio_template;
	dev->rdev.ctrl_handler = &dev->ctrl_handler;
	dev->rdev.lock = &dev->rdev_mlock;
	dev->rdev.v4l2_dev = v4l2_dev;
	video_set_drvdata(&dev->rdev, dev);

	ret = video_register_device(&dev->rdev, VFL_TYPE_RADIO, -1);
	if (ret)
		goto free_v4l2;

	radcap_pr_info("id: %llx (%u %s tuners detected)\n",
		      (u64)dev->fpga_id, dev->card_nodes, dev->card.band);
	if (debug > 0)
		radcap_pr_info("irq: %u, Ctrl MMIO: 0x%p, SGDMA MMIO: 0x%p\n",
			       pdev->irq, dev->ctrl, dev->sg);
	radcap_pr_info("registered as %s (%d %s tuners enabled)\n",
		       video_device_node_name(&dev->rdev),
		       dev->nodes, dev->card.band);

	radcap_devcount++;
	request_modules(dev);
	return 0;

free_v4l2:
	v4l2_ctrl_handler_free(hdl);
	v4l2_device_unregister(v4l2_dev);
free_sg_mmio:
	iounmap(dev->sg);
free_ctrl_mmio:
	iounmap(dev->ctrl);
free_pci_regions:
	pci_release_regions(pdev);
disable_device:
	pci_disable_device(pdev);
	return ret;
}

static void radcap_pcie_remove(struct pci_dev *pdev)
{
	struct v4l2_device *v4l2_dev = pci_get_drvdata(pdev);
	struct radcap_dev *dev = container_of(v4l2_dev, struct radcap_dev, v4l2_dev);

	/* Shutdown card */
	radcap_set_demod(dev, RADCAP_DEMOD_OFF);
	udelay(5);

	flush_request_modules(dev);
	radcap_pr_info("removing %s\n", video_device_node_name(&dev->rdev));

	mutex_lock(&radcap_devlist_lock);
	list_del(&dev->devlist);
	mutex_unlock(&radcap_devlist_lock);
	radcap_devcount--;

	video_unregister_device(&dev->rdev);
	v4l2_ctrl_handler_free(&dev->ctrl_handler);
	v4l2_device_unregister(&dev->v4l2_dev);

	/* Release resources */
	iounmap(dev->sg);
	iounmap(dev->ctrl);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

static int __maybe_unused radcap_pcie_suspend(struct device *dev_d)
{
	struct v4l2_device *v4l2_dev = dev_get_drvdata(dev_d);
	struct radcap_dev *dev = container_of(v4l2_dev, struct radcap_dev, v4l2_dev);

	/* Shutdown card */
	radcap_set_demod(dev, RADCAP_DEMOD_OFF);
	udelay(5);

	return 0;
}

static int __maybe_unused radcap_pcie_resume(struct device *dev_d)
{
	struct v4l2_device *v4l2_dev = dev_get_drvdata(dev_d);
	struct radcap_dev *dev = container_of(v4l2_dev, struct radcap_dev, v4l2_dev);

	/* Write the key */
	if (key_count) {
		radcap_hw_write_key(dev);
		mb(); /* Ensure the key is written */
	}

	/* Everything needs to be restarted */
	radcap_hw_reset(dev, RADCAP_HW_RESUME);
	radcap_set_demod(dev, RADCAP_DEMOD_ON);
	return 0;
}

static const struct pci_device_id radcap_pcie_tbl[] = {
	{ PCI_DEVICE_DATA(SONIFEX, RADCAP_PCIE_AM, CARD_AM) },
	{ PCI_DEVICE_DATA(SONIFEX, RADCAP_PCIE_FM, CARD_FM) },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, radcap_pcie_tbl);

static SIMPLE_DEV_PM_OPS(radcap_pcie_pm_ops, radcap_pcie_suspend, radcap_pcie_resume);

static struct pci_driver radcap_pcie_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= radcap_pcie_tbl,
	.probe		= radcap_pcie_probe,
	.remove		= radcap_pcie_remove,
	.driver.pm	= &radcap_pcie_pm_ops,
};

module_pci_driver(radcap_pcie_driver);
