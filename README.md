# Bornhack Badge 2017

## About the hardware

The Bornhack 2017 badge is based around a SiLabs Happy Gecko microcontroller,
an OLED graphical display, a few buttons and a USB interface.

The microcontroller is a Cortex-M0+ with built in USB, specifically we are using
the EFM32HG322F64G and one of the 128x64 pixels OLED displays with an SSD1306
controller.

We have separated the hardware design files from the code and put them in the
[hardware branch][hardware] for you to have a look at and build on.

You can download the microcontroller [reference manual][manual] and
[datasheet][] directly from [SiliconLabs][silabs].

[hardware]: https://github.com/bornhack/badge2017/tree/hardware
[silabs]: https://www.silabs.com/
[manual]: https://www.silabs.com/documents/public/reference-manuals/EFM32HG-RM.pdf
[datasheet]: https://www.silabs.com/documents/public/data-sheets/EFM32HG322.pdf

## About the code

This repo is meant as a starting point for developing your own code
for the Bornhack 2017 badge.

Although *main.c* is quite long it is not meant to scare you. Rather it is
meant to show in all its gory details how to actually program microcontrollers
that doesn't run any operating system without hiding anything behind
a thick API. However to make things readable it does make use of the
[geckonator][] library. This is a very thin wrapper around the registers
of the Happy Gecko chip on the badge. It allows us to write something like
```c
clock_rtc_disable();
```
rather than the more cryptic
```c
CMU->LFACLKEN0 &= ~CMU_LFACLKEN0_RTC;
```
Don't worry. The compiler will sort it out and do exactly as if you'd written the
cryptic version.

Scroll to the bottom of *main.c* and you'll see an infinite loop.
This is the 'mainloop' of the program. It simply waits for events
such as button presses and reacts to those. When there are no
more events to be processed in the internal queue you'll get one last `EVENT_LAST`
event before the chip will sleep and wait for more events.

Events are added to the queue by the interrupt handler functions. You can recognize those
by the fact that they're public (ie. not marked by `static`) and their name will
end in `_IRQHandler`. Before entering the mainloop the chip is configured to
call these handlers on certain events such the voltage on a pin dropping from above 1.5V
to below 0.3V caused by the press of a button.

[geckonator]: https://github.com/flummer/geckonator

## Develop software online

URL to be announced..

1. Click the "Download binary" button to download the compiled code.
2. Rename the downloaded file to `code.bin`.
3. Connect the USB cable to the board and your computer.
4. Press the **BOOT** button on the badge.
5. Copy the downloaded `code.bin` to the `GECKOBOOT` USB stick that appeared on your computer.
6. Eject (or unmount) the USB stick, and watch your code run.

## Develop software locally

### 1. Install dependencies

##### Archlinux
```sh
pacman -S arm-none-eabi-gcc arm-none-eabi-newlib make
```

##### Debian/Ubuntu
```sh
apt-get install arm-none-eabi-gcc libnewlib-arm-none-eabi make
```
Unfortunately the toolchain in Ubuntu Trusty 14.04 is too old to work out of the box.

##### Fedora
```sh
dnf install arm-none-eabi-gcc arm-none-eabi-newlib make
```

##### OSX

Download an arm-none-eabi toolchain from ARM [here][arm-toolchain].
Unpack the tarball and update your PATH variable to point to the unpacked `bin` directory.

##### Windows
###### Option 1
Download the apropriate installer from ARM [here][arm-toolchain].
Install it and update your path.
You'll also need GNU Make installed.
A pre-built version can be downloaded [here](http://gnuwin32.sourceforge.net/packages/make.htm).

###### Option 2

Use Windows Subsystem for Linux with Ubuntu 16.04 Xenial or newer and proceed as on Ubuntu above.

### 2. Get the source code

If you already have git installed
```
git clone https://github.com/bornhack/badge2017.git
cd badge2017
```

Otherwise you can download a tarball or zip file from
[https://github.com/bornhack/badge2017](https://github.com/bornhack/badge2017)

### 3. Build the code
Simply type `make` in the downloaded directory.
If the build fails (fx. on Debian Jessie and Ubuntu Xenial)
try editing the `Makefile` and uncommenting the line which says `OLD=1`.

If there are no compilation errors this will result in a `code.bin` in the `out` directory.

[arm-toolchain]: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
