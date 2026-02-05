# MASN: Mobile Acoustic Sensor Node

The **MASN** is an advanced, low-power sensor node designed for high-fidelity acoustic monitoring and long-range data transmission via NB-IoT. This repository contains the complete hardware design files, firmware, and configuration tools.

## Project Status: Minimal Working Version (MWV)
**Note:** The current firmware implementation is a **minimal working version**. While the hardware is fully featured, the firmware does not yet implement all architectural capabilities described in the documentation. Users are encouraged to consult the **Roadmap** below for contribution opportunities.

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
The MASN is implemented on a double-sided printed circuit board (PCB). The architecture utilizes two isolated voltage domains:
1.  **1.8 V Domain (Front Face):** Powers the MCU, GNSS, microphone, and EEPROM.
2.  **2.1 V Domain (Back Face):** Powers the NB-IoT communication components.



---

## Firmware & Software

### Firmware Description (Current Implementation)
The firmware is built on **FreeRTOS**, following the architecture proposed in Quintero-Perez et al. (2026). It enables concurrent execution of four primary tasks:
* **Sampling:** Continuous audio acquisition using DMA and I2S protocol at $F_s = 32$ kHz.
* **Signal Processing:** Basic computation of $L_{Aeq}$ and OTOB indices ($L_{eq, f_c}$).
* **Georeferencing (GNSS):** Interfacing with the MAX-M10S module to retrieve latitude, longitude, satellite count (Sat#), and fix quality (Q). This metadata is integrated into the acoustic data blocks.
* **Transmission:** Support for network attachment and data delivery via **CoAP protocol** over NB-IoT.

### Configuration GUI
A Python-based GUI facilitates node management via USB-C:
* **Location:** `/Software/CONFIG_GUI`
* **Features:** Time synchronization, module management (GNSS/NB-IoT), and sensor calibration.

---

## Roadmap & Known Issues

### To-Do (Pending Features)
- [ ] **125 ms Sampling:** Implement granular chunking for high-resolution analysis.
- [ ] **IMEI Management:** Read IMEI once and store in non-volatile memory to reduce startup overhead.
- [ ] **Auto-Time Sync:** Automatic RTC configuration based on GNSS NMEA sentences.
- [ ] **EEPROM Integration:** Full implementation of data caching during network outages.

### Known Bugs & Technical Debt
- **Prescaler Drift:** Issues observed with prescaler stability over very long-term operation.
- **Communication Inefficiency:** Optimization required for both NB-IoT module and USB-C serial communication stacks.
- **Interrupt Overhead:** High processing load within `HAL_UART_RxCpltCallback`; logic needs to be migrated to task-level processing.

---

## Citation

If you use this hardware or software in your research, please cite the following paper:

> Quintero, G., Balastegui, A., & Romeu, J. (2026). **Autonomous mobile low-cost sensor node for high-resolution noise sampling in urban environments**. *Measurement*. (Accepted, pending publication).

---

## Licensing

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
* `/Software/CONFIG_GUI`: Python source code and UI files.
* `/docs`: Technical documentation, implementation images, and license files.