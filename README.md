# Basic MAC

[Basic MAC](https://doc.sm.tc/mac) is a portable implementation of the LoRa™
Alliance's LoRaWAN™ specification in the C programming language. It is a fork
of IBM's LMiC library. It supports multiple regions, selectable at compile-
and/or run-time, and it can handle class A, class B, and class C devices.

#### Documentation

The full documentation is available at
[https://doc.sm.tc/mac](https://doc.sm.tc/mac).

## First Steps

The reference hardware platform for Basic MAC is the [B-L072Z-LRWAN1 STM32
LoRa™ Discovery
kit](https://www.st.com/en/evaluation-tools/b-l072z-lrwan1.html). The steps
below assume building for this platform.

### Prerequisites

* **Toolchain**

    It is recommended to use a recent Ubuntu distribution as build host. To
    build a Basic MAC project, you must have an appropriate cross-compile
    toolchain installed. For the STM32 target platform, we use
    `gcc-arm-embedded` from this PPA:
    <https://launchpad.net/~team-gcc-arm-embedded/+archive/ubuntu/ppa>

* **Python 3.6 or newer**
    
    The build environment uses a number of tools that are written in Python.
    An environment with a Python 3.6 interpreter or newer is required.
    Further, the following Python package must be installed (e.g. using `pip
    install`):
    
    - intelhex

* **OpenOCD**

    To load firmware onto actual hardware, a tool such as
    [OpenOCD](http://openocd.org/) must be used. On Ubuntu, you can install it
    using `sudo apt install openocd`.

### Cloning the Repository

Clone the repository from GitHub:

```
$ git clone https://github.com/lorabasics/basicmac.git $MAC_REPO
```

Basic MAC contains Git submodules that must be initialized:

```
$ git submodule update --init
```

### Build Process

#### Building the Bootloader

The Basic MAC HAL for STM32 employs a bootloader, *Basic Loader*, to load and
start the actual firmware. This bootloader also applies any firmware updates
and verifies the integrity of the current firmware before calling the
firmware's entrypoint.

To build Basic Loader, change to the target board's build directory and type
make:

```
$ cd $MAC_REPO/basicloader/build/boards/B-L072Z-LRWAN1
$ make
```

The output of the build process is a file called `bootloader.hex`.


#### Building a Project

Projects are built from their respective subfolder in the `projects` directory.
A simple example project is the *Join Example* project in
`$MAC_REPO/projects/ex-join`.

Build settings, such as target platform, compile time and configuration options
are specified in the project's `Makefile`.

To build a project, simply change into the project's directory and run _make_:
```
$ cd $MAC_REPO/projects/ex-join
$ make
```

Multiple variants of a project can be built, such as versions for different
regions. The *Join Example* project builds two variants: `eu868` and `us915`.

The output files of the build process are stored in the project's
`build-<variant>` folders, with the firmware hex file having the same name as
the project with a *.hex* extension; in this case `ex-join.hex`.


### Loading a Project

After a project is built, it can be loaded onto a device. Note that both the
bootloader and the firmware must be loaded, although since the bootloader
rarely changes, it is generally only loaded once.

> **Note:**
>   You may need to install udev rules to grant access permissions to regular
>   users for accessing the ST-LINK device. You can install these using the tar
>   file provided in the Basic MAC repository with following command: `sudo tar
>   xzvf $MAC_REPO/tools/openocd/stlink-rules.tgz -C /etc/udev/rules.d/`.

To load the bootloader, run `make loadbl`; to load the firmware, run `make
load`:

```
$ make loadbl       # if not already done previously
$ make load
```

If multiple variants are present, this will load the *default variant*, which
is generally the first variant specified in the project's Makefile.

### Personalization

The HAL for STM32 stores personalization information such as EUIs and keys for
LoRaWAN operation in EEPROM. This pre-release of Basic MAC does not contain the
tools necessary to generate and write this information to the device.

If no valid personalization information is found in EEPROM, the HAL will
create a device EUI from the MCU's Unique ID registers, and use a fixed Join
EUI and test key:

- Device EUI: `FF-FF-FF-AA-xx-xx-xx-xx`
- Join EUI: `FF-FF-FF-BB-00-00-00-00`
- Device Key: `404142434445464748494A4B4C4D4E4F`

### Viewing Debug Output

On the B-L072Z-LRWAN1, the firmware prints debug information to the UART that
is connected via the ST-LINK to the host computer. On Linux, this device
usually shows up as `/dev/ttyACM0`. Use a serial terminal application to
connect to that port, using 115200/8-N-1.

```
$ miniterm.py /dev/ttyACM0 115200
```
```text

--- Miniterm on /dev/ttyACM0  115200,8,N,1 ---
--- Quit: Ctrl+] | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H ---

============== DEBUG STARTED ==============
id: FF-FF-FF-AA-2B-05-01-41 | sn:  | hw: 0x000 | flash: 192K
bl: v256 | fw: ex-join eu868 0x00000000 0x8B27E203 | boot: normal
Hello World!
switching mode: normal
lwm: JOINING
lwm: TXSTART
TX[freq=868.1,sf=7,bw=125,len=23]: 0000000000BBFFFFFF4101052BAAFFFFFFECF2B6412021
lwm: TXDONE

...
```

# Release Notes

While in pre-release phase, Basic MAC may be missing features, and internal
components may still be undergoing significant changes. Some of these are:

- **New Peripherals API and Drivers**

    Previously, device drivers lived outside the scope of the stack and were
    the responsibility of the application developer. However, to facilitate
    development, a new cross-platform API for common peripherals, such as GPIOs,
    UART, SPI, I²C, LEDs, and more is being added to the core at
    `lmic/peripherals.h`. This API is in very early stages and does not yet
    support many devices.

- **New Region Architecture**

    Currently, a build of the stack supports a single region as defined in the
    LoRaWAN Regional Parameters specification. A major change in this
    architecture is underway that enables the simultaneous support of multiple
    regions by the same firmware. It is then possible to select the supported
    set of regional parameters at compile time, and select the effective region
    at run-time.

- **LoRaWAN 1.1 Support**

    When looking at the source code, you may find references to LoRaWAN version
    1.1. Please note that these do not implement the current release of the
    1.1 specification and that **LoRaWAN 1.1 is not currently supported**!
    There are multiple issues and inconsistencies in LoRaWAN 1.1 that are being
    addressed by the Technical Committee of the LoRa Alliance. Until these are
    rectified, Basic MAC will only support the 1.0.x series of LoRaWAN.

### Pre-release 1
07-Jan-2019

- Implements LoRaWAN 1.0.3; Class A; EU868, US915
