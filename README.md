kernel-drobofs
==============

Kernel source code for the DroboFS.

Obtained from [here](http://support.drobo.com/app/answers/detail/a_id/434).

## How to compile modules

First make sure that you have a [working cross-compiling VM](https://github.com/droboports/droboports.github.io/wiki/Setting-up-a-VM).

Log in the VM, pick a temporary folder (e.g., `~/build`), and then do:

```
git clone https://github.com/droboports/kernel-drobofs.git
cd kernel-drobofs
source crosscompile.sh
cd kernel
make mrproper ARCH=arm CROSS_COMPILE=${HOST}-
make mv78200_defconfig ARCH=arm CROSS_COMPILE=${HOST}-
make menuconfig ARCH=arm CROSS_COMPILE=${HOST}-
```

Pick the modules from the menu, and then:

```
make modules ARCH=arm CROSS_COMPILE=${HOST}- CFLAGS_MODULE="-DMODULE -mlong-calls"
find . -name "*.ko"
```

See the [releases](https://github.com/droboports/kernel-drobofs/releases) page for pre-compiled modules.

<sub>**Disclaimer**</sub>

<sub><sub>Drobo, DroboShare, Drobo FS, Drobo 5N, DRI and all related trademarks are the property of [Data Robotics, Inc](http://www.drobo.com/). This site is not affiliated, endorsed or supported by DRI in any way. The use of information and software provided on this website may be used at your own risk. The information and software available on this website are provided as-is without any warranty or guarantee. By visiting this website you agree that: (1) We take no liability under any circumstance or legal theory for any DroboApp, software, error, omissions, loss of data or damage of any kind related to your use or exposure to any information provided on this site; (2) All software are made “AS AVAILABLE” and “AS IS” without any warranty or guarantee. All express and implied warranties are disclaimed. Some states do not allow limitations of incidental or consequential damages or on how long an implied warranty lasts, so the above may not apply to you.</sub></sub>
