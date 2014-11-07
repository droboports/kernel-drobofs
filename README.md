kernel-drobofs
==============

Kernel source code for the DroboFS

## How to compile modules

In the `kernel` folder:

```
source ../crosscompile.sh
make mrproper ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi-
make mv78200_defconfig ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi-
make menuconfig ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi-
```

Pick the modules from the menu, and then:

```
make modules ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi- CFLAGS_MODULE="-DMODULE -mlong-calls"
find . -name "*.ko"
```

See the [releases](https://github.com/droboports/kernel-drobofs/releases) page for pre-compiled modules.
