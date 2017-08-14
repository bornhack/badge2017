# Bornhack Badge 2017

## About the hardware

The Bornhack 2017 badge is based around a SiLabs Happy Gecko microcontroller,
an OLED graphical display, a few buttons and a USB interface.

The microcontroller is a Cortex-M0+ with built in USB, specifically we are using
the EFM32HG322F64G and one of the 128x64 pixels OLED displays with an SSD1306
controller.

The PCB has been designed in KiCad and uses a double sided PCB, with most of the
components on the front side (only the battery clips and the proto typing area are
on the back side).

## About the code

We have separated the hardware design files from the code and put the code in the
[master branch](https://github.com/bornhack/badge2017/tree/master) for you
to have a look at and extend.