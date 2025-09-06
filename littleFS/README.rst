Interface to Macronix MX25L51245GZ2I-08G IC FLASH 512MBIT SPI/QUAD 8WSON using LittleFS library imported from MAX32666 LittleFS library.

Overview
********

The example shows how to interface to the MX25L51245GZ2I-08G using SPI Mode 0 interface and configure LittleFS library using the same Flash format as what would be use on the MAX326666 LittleFS format.

Requirements
************

Your board must:

#. Have an LED connected via a GPIO pin (these are called "User LEDs" on many of
   Zephyr's :ref:`boards`).
#. Have the LED configured using the ``led0`` devicetree alias.

#. Connect ot MX25L51245GZ2I-08G via Pin 0.25 (SCK), Pin 0.23 (MOSI), Pins 0.24 (MISO), and Pin 0.22 (CS)

Building and Running
********************
Use the following Build Configuration:

* **SDK:** nRF Connect SDK v2.6.1

* **Toolchain:** nRF Connect SDK Toolchain v2.6.1

* **Board Target:** nrf52832_ev_bc832

* **Base configuration files (Kconfig fragments):** prj.conf

* **Extra Kconfig fargments:** NONE

* **Base Devicetree overlays:** boards/arm/nrf52832_ev_bc832/nrf52832_ev_bc832.overlay

* **Extra Devicetree overlays:** NONE

* **Snippets:** NONE

* **Optimization level (size, speed, or debugging):** Use Project default

* **Extra CMake arguments:** NONE

* **Build directory name:** build

* **Generate only:** NOT CHECKED

* **System build (sysbuild):** Build system default

