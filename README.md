# ALAN — Autonomous Long Range Aquatic Navigation Vessel
## LoRa Communications Subsystem

ALAN is a senior capstone project implementing a long-range wireless communication link between a boat-mounted STM32H755 microcontroller and a shore-based SparkFun SAMD21 ground station, using LoRa radio modulation at 915 MHz.

This subsystem handles all bidirectional telemetry and command communication between the vessel and ground station. The boat transmits state information (LED status, servo angle) to the ground station at 1Hz, and the ground station operator can send real-time commands to control the vessel.

---

## Team

### Sensor Team
- Corbin Barney (u1066089)
- Charbel Salloum (u1446840)
- Derek Tinoco (u1382366)
- Connor Stevens (u1463295)

### Antenna Team
- Harrison LeTourneau (u1460207)
- Adam Welsh (u1456456)
- Jared Pratt (u1237327)
- Aliou Tippett (u1415075)

---

## Hardware Requirements

**Boat Side**
- STM32H755ZI-Q Nucleo-144 development board
- RFM95W LoRa transceiver module wired to SPI1
- 915 MHz antenna (correctly tuned)
- UART serial adapter for monitoring incoming/outgoing messages
- Servo and motor board with appropriate battery

**Ground Station**
- SparkFun SAMD21 Pro RF (has RFM95W onboard)
- 915 MHz antenna (correctly tuned)
- USB cable

---

## Software Requirements

- macOS (tested on Mac only)
- `arm-none-eabi-gcc` toolchain
- `cmake`, `ninja`, `openocd`
- Python 3 (for dashboard)
- flask pyserial
- PlatformIO (for SAMD21)

---

## Architecture

The STM32H755 is a dual-core processor. The **CM4 core** exclusively handles the LoRa communication subsystem described here. The **CM7 core** is used by a separate subsystem handling sensor integration and path planning.

The LoRa link is configured as follows:

| Parameter | Value |
|-----------|-------|
| Frequency | 915 MHz (US ISM band) |
| Spreading Factor | SF7 |
| Bandwidth | 125 kHz |
| Coding Rate | 4/5 |
| TX Power | 17 dBm |

At these settings the link achieves approximately 10-20 km range on open water with an expected airtime of ~70ms per packet. The US ISM band has no duty cycle restrictions so 1Hz transmission is well within legal limits.

---

## Scripts

### `./build.sh [-cm4|-cm7|-startup]`

The build script is used to set up the repo on first clone or compile individual processor `.elf` files.

- `-startup` — Run once after first cloning the repo. Installs tools, configures projects, and builds both processors.
- `-cm4` — Builds the CM4 `.elf` only, without performing the startup routine.
- `-cm7` — Builds the CM7 `.elf` only, without performing the startup routine.
- Both `-cm4` and `-cm7` can be declared at the same time to build both `.elf` files.

### `./flash.sh [-cm4|-cm7|-b]`

The flash script is used to build and flash the STM32 for a specific processor.

- `-cm4` — Flashes the CM4 processor.
- `-cm7` — Flashes the CM7 processor.
- `-b` — Builds the binary for the declared processors before flashing. If `-b` is omitted the script will flash only, no build.
- If both `-cm4` and `-cm7` are declared, the script will always flash CM7 first and then CM4.

> **Note:** The CM7 must always be flashed before the CM4. The dual-core boot sequence requires the CM7 to release a hardware semaphore before the CM4 will exit stop mode and begin executing.

---

## How To Operate

### Boat Side (STM32 CM4)

Connect the STM32 Nucleo board to your computer via USB. On first clone run the startup script:

```bash
./build.sh -startup
```

Then build and flash both cores — CM7 first, then CM4:

```bash
./flash.sh -b -cm7
./flash.sh -b -cm4
```

On subsequent builds you can use `-cm4` or `-cm7` individually without `-startup`. Once running, connect a UART serial adapter to monitor incoming commands and outgoing telemetry.

### Ground Station (SAMD21)

Switch to the ground station branch:

```bash
git checkout SAMD21_LoRa
```

Follow the instructions in the `SAMD21_LoRa` branch README to build and flash. Once running you have two options:

**Option 1 — Serial terminal direct control**

Open a serial monitor and send single character commands:

| Key | Action |
|-----|--------|
| `q` | Servo +20° |
| `e` | Servo -20° |
| `a` | Servo +10° |
| `d` | Servo -10° |
| `z` | Servo +5° |
| `c` | Servo -5° |
| `g` | Toggle green LED |
| `y` | Toggle yellow LED |
| `r` | Toggle red LED |

**Option 2 — Web dashboard**

```bash
python3 dashboard.py [port] [baud_rate]
```

This opens a browser-based GUI at `localhost` for monitoring vessel state and sending commands visually.

---

## Repository Structure

```
├── CM4/                  # STM32 CM4 LoRa firmware
│   └── Core/Src/
│       ├── main.c
│       ├── lora.c        # LoRa driver (SPI, register config, TX/RX)
│       └── lora.h
├── CM7/                  # STM32 CM7 firmware (sensor/path planning subsystem)
├── build.sh              # Build script for STM32
├── flash.sh              # Flash script for STM32
└── docs/                 # Project documentation
```