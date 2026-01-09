# Tercio FD Firmware

<div align="center">

# TERCIO FD
**High-Speed USB to CAN-FD Adapter | The Bridge to Your Ecosystem**

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Documentation](https://img.shields.io/badge/Docs-terciolabs.com-green)](https://docs.terciolabs.com)
[![Hardware](https://img.shields.io/badge/Hardware-TercioLabs-orange)](https://terciolabs.com)

</div>

---

## Hardware Availability

This repository contains the firmware source code for the Tercio FD Adapter.
**Official hardware is available for purchase at [terciolabs.com](https://terciolabs.com).**

---

## 🚀 Project Overview

**Tercio FD** is the critical link between your computer and the Tercio ecosystem. It serves as a high-performance USB-to-CAN-FD bridge, enabling your PC to communicate with Tercio S1 drivers and IMU modules in real-time.

Designed for low latency and high reliability, it is the required interface for using the **Tercio Web Control GUI**.

### Key Features
* **High-Speed CAN-FD:** Supports data rates up to 5 Mbps (optimized for 2.5 Mbps).
* **USB Connectivity:** Seamless plug-and-play connection to PC/Mac/Linux.
* **Low Latency:** Optimized packet forwarding for real-time robotics control.
* **Ecosystem Integration:** Natively supported by the Tercio Web GUI for motor configuration and tuning.

---

## 🛠️ Development & Flashing

### Hardware Requirements
To develop or flash firmware on the Tercio FD, you will need:
1.  **Tercio FD Adapter**
2.  **ST-Link V2/V3 Programmer** (for flashing via SWD)
3.  **Wiring:** SWD connections (SWDIO, SWCLK, GND, 3V3)

### Development Environment
You can build and flash this project using either **PlatformIO** (VSCode) or **STM32CubeIDE**.

#### Option A: PlatformIO (Recommended)
1.  Install [Visual Studio Code](https://code.visualstudio.com/) and the **PlatformIO** extension.
2.  Clone this repository:
    ```bash
    git clone [https://github.com/Kerol-Dev/Tercio-fdcan.git](https://github.com/Kerol-Dev/Tercio-fdcan.git)
    ```
3.  Open the project folder in PlatformIO.
4.  Connect your ST-Link to the Tercio FD board via the SWD headers.
5.  Click the **PlatformIO: Upload** button (arrow icon) to build and flash.

#### Option B: STM32CubeIDE
1.  Open **STM32CubeIDE**.
2.  Select `File > Open Projects from File System...` and navigate to the repository folder.
3.  Connect your ST-Link to the board.
4.  Build the project (Hammer icon).
5.  Run/Debug (Play/Bug icon) to flash the firmware.

---

## 📚 Documentation

For complete wiring diagrams, pinouts, and API references, please consult the official documentation:
**[👉 docs.terciolabs.com](https://docs.terciolabs.com)**

---

## 🤝 Contributing

We welcome contributions! If you have optimizations or feature additions for the CAN-FD handling:
1.  Fork the repository.
2.  Create a feature branch (`git checkout -b feature/BetterBuffering`).
3.  Commit your changes.
4.  Open a Pull Request.

---

## 📄 License

This project is licensed under the **Non-Commercial MIT License** - see the [LICENSE](LICENSE) file for details.

<div align="center">
  <sub>Copyright © Tercio Labs</sub>
</div>
