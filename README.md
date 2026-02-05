# MASN: Mobile Acoustic Sensor Node

The **MASN** is an advanced, low-power sensor node designed for high precission acoustic monitoring and long-range data transmission via NB-IoT. This repository contains the complete hardware design files, firmware, and configuration tools.

## Hardware Overview

The hardware architecture is optimized for power efficiency and modular operation, featuring a specialized dual-voltage design.

### Technical Specifications
* **Microcontroller:** STM32L452RET6P (Ultra-low-power ARM Cortex-M4).
* **Acoustic Sensing:** SPH0645LM4H MEMS microphone.
* **Positioning:** MAX-M10S-00B GNSS module.
* **Connectivity:** SIMCOM SIM7022 NB-IoT module.
* **Storage:** M95P32-IXMNT/E ultra-low-power EEPROM (32 Mbit).
* **Interface:** USB-C for parameter configuration, calibration, and data recovery.

### Architecture and PCB Design
The MASN is implemented on a double-sided printed circuit board (PCB) constructed from FR4-Standard TG 135-140 (1.6 mm thickness). The architecture utilizes two isolated voltage domains:

1.  **1.8 V Domain (Front Face):** Powers the MCU, GNSS, microphone, and EEPROM.
2.  **2.1 V Domain (Back Face):** Powers the NB-IoT communication components.

This facilitates **1.8 V offline operation**, allowing the node to store acoustic data in the EEPROM while communication modules are disabled, significantly extending autonomy.

### Bill of Materials (BOM Summary)

| Component | Description |
| :--- | :--- |
| **STM32L452RET6P** | Low-power Microcontroller |
| **SPH0645LM4H** | MEMS microphone |
| **MAX-M10S-00B** | GNSS Module |
| **SIMCOM SIM7022** | NB-IoT module |
| **M95P32-IXMNT/E** | Ultra low-power EEPROM |

> **Assembly Note:** Due to the high density of components and LGA/BGA packages, professional SMT assembly is required.

---

## Firmware & Software

### Firmware Description
The firmware is built on **FreeRTOS**, enabling concurrent execution of four primary tasks:
* **Sampling:** Continuous acquisition using DMA and I2S protocol at $F_s = 32$ kHz.
* **Signal Processing:** Frequency compensation and computation of $L_{Aeq}$ and OTOB indices ($L_{eq, f_c}$ from 63 Hz to 10 kHz).
* **Georeferencing:** Integration of Latitude, Longitude, satellite count, and timestamps into data blocks.
* **Transmission/Storage:** Data delivery via **CoAP protocol** over NB-IoT, with automatic failover to EEPROM storage during network outages.


### Configuration GUI
A Python-based Graphical User Interface (GUI) is provided to facilitate node management via USB-C.
* **Location:** `/Software/CONFIG_GUI`
* **Features:** Time synchronization, module enabling/disabling (GNSS/NB-IoT), data recovery from EEPROM, and sensor calibration.

---

## Licensing

This project follows a strong copyleft policy to ensure improvements remain available to the community.

### Hardware
Licensed under **CERN Open Hardware Licence Version 2 - Strongly Reciprocal (CERN-OHL-S)**.
* Full text: `docs/CERN-OHL-S.txt`

### Firmware & Software
Licensed under **GNU General Public License v3.0 (GPL-3.0)**.
* Full text: `docs/GNU GPL v3.txt`

---

## Repository Structure
* `/PCB`: Schematics, PCB design files, and BOM.
* `/Firmware`: Source code and build files for STM32.
* `/Software/CONFIG_GUI`: Python source code (`main.py`) and UI files for the configuration tool.
* `/docs`: Technical documentation, implementation images, and license files.