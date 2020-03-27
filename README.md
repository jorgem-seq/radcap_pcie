## Sonifex Radcap PCIe V4L2 Driver for Linux 5.5+

### Loading the modules

Under ./src run:

```sh
$ make
$ sudo modprobe videodev
$ sudo insmod Radcap_PCIe.ko
$ sudo insmod Radcap_PCIe_ALSA.ko

```

### Unloading the modules

```sh
$ sudo rmmod -f Radcap_PCIe_ALSA
$ sudo rmmod -f Radcap_PCIe
```
