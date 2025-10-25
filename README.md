
# Zigbee Temperature Sensor - ESP32-C6

Battery-powered Zigbee End Device that reads up to 3 DS18B20 temperature sensors and reports battery voltage.

## Setup ESP-IDF Environment

Before building, you need to set up the ESP-IDF environment in each terminal session:

```bash
# Source ESP-IDF tools (adds idf.py to PATH and sets environment variables)
. ~/prototypes/tools/esp-idf/export.sh
```

**What export.sh does:** Sets up the ESP-IDF toolchain by adding build tools (idf.py, xtensa-gcc, etc.) to your PATH and configuring environment variables needed for ESP32 development.

**Optional:** Add to ~/.zshrc to make it automatic:
```bash
echo ". ~/prototypes/tools/esp-idf/export.sh" >> ~/.zshrc
```

## Build & Flash

To see active devices on ports and which to use: 
```bash
dmesg | grep tty
```

Ease flash  

    Every time you flash, ZHA sees it as a NEW device because:

    idf.py erase-flash wipes the IEEE address stored in NVS
    The ESP32-C6 generates a new random IEEE address on each flash erase
    ZHA identifies Zigbee devices by their IEEE address (like a MAC address)
    Different IEEE address = different device in ZHA

```bash
idf.py -p /dev/ttyACM0 erase-flash
```

Build & flash
```bash
idf.py -p /dev/ttyACM0 build flash
```

Montitor (aka run)
```bash
idf.py -p /dev/ttyACM0 monitor
```

**Recommended:** All in one full erase and reflash
```bash
./erase_build_flash_monitor.sh
```

Or manually:
```bash
idf.py -p /dev/ttyACM0 erase-flash
idf.py -p /dev/ttyACM0 build flash
idf.py -p /dev/ttyACM0 monitor
```

## Notes
- Use script to erase, build, flash & monitor: Sometimes had issues running monitor right after build/flash in one line, run separetly.
- If battery monitoring does not get identified, try removing the USB before it pairs/connects, then running it off the battery for pairing, helps with identifying the power source = battery != mains

