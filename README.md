# IoT-Enabled Smart Socket MCB
 
**Context:** This project is done as a part of my Embedded & IoT Internship at Gaibi Sahib Technologies (Feb 2026 – Mar 2026)

## 📖 About This Repository
This repository contains the embedded C firmware for an intelligent, socket-level Miniature Circuit Breaker (MCB). 

**What is inside:**
* `Core/Src/`: Main application logic, EXTI interrupt handlers, and timer synchronization routines.
* `Core/Inc/`: Header files, including `hlw8012_config.h` for energy calibration multipliers.
* `.ioc file`: Hardware configuration mapping for STM32CubeIDE.

**The Goal:** To design a digital circuit breaker capable of real-time active power metering and automated power cutoff during overload events, providing a safer alternative to traditional thermal-magnetic MCBs.

**The Outcome:** Developed a highly responsive system that interfaces an STM32 with an HLW8012 energy metering IC. The firmware utilizes microsecond-level EXTI (External Interrupt) logic to monitor pulse frequencies, calculating real-time voltage, current, and power to trigger a relay cutoff instantly when safety thresholds are breached.

## 🛠️ How to Use This Repository
*⚠️ DANGER: This project involves interfacing with mains AC voltage (110V/220V). Ensure strictly isolated power supplies and practice extreme caution.*

### 1. Hardware Prerequisites
* **Microcontroller:** STM32 (Blue Pill / Nucleo)
* **Energy IC:** HLW8012 Breakout Board
* **Actuator:** 5V High-Current Relay Module (Rated for AC loads)

### 2. Wiring Configuration
* `HLW8012 CF` (Active Power pulse) ➡️ STM32 EXTI Pin [Insert Pin]
* `HLW8012 CF1` (V/I pulse) ➡️ STM32 EXTI Pin [Insert Pin]
* `HLW8012 SEL` (Mode Select) ➡️ STM32 GPIO Pin [Insert Pin]
* `Relay IN` ➡️ STM32 GPIO Pin [Insert Pin]

### 3. Build & Calibration Instructions
1. Open the project in **STM32CubeIDE**.
2. Navigate to `Core/Inc/hlw8012_config.h`. 
3. **Calibration:** You must calibrate the IC for your specific AC environment. Connect a known resistive load (like a 60W bulb), read the raw pulse outputs, and adjust the multipliers:
   ```c
   #define CURRENT_MULTIPLIER  [Default_Value]
   #define VOLTAGE_MULTIPLIER  [Default_Value]
   #define POWER_MULTIPLIER    [Default_Value]
