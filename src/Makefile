# SPDX-License-Identifier: GPL-2.0
Radcap_PCIe-y := radcap_pcie_core.o radcap_pcie_v4l2.o
Radcap_PCIe_ALSA-y := radcap_pcie_alsa.o

obj-m += Radcap_PCIe.o
obj-m += Radcap_PCIe_ALSA.o

KVERSION = $(shell uname -r)
CURR_PWD = $(shell pwd)

all: module

module:
	make -C /lib/modules/$(KVERSION)/build M=$(CURR_PWD) modules

install:
	make -C /lib/modules/$(KVERSION)/build M=$(CURR_PWD) modules_install
	depmod -a

clean:
	make -C /lib/modules/$(KVERSION)/build M=$(CURR_PWD) clean
