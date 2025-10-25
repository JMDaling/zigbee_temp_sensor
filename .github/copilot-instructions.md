# Zigbee Temperature Sensor - ESP32-C6

## Architecture Overview

This is a **battery-powered Zigbee End Device** (sleepy device) using ESP32-C6 that reads up to 3 DS18B20 temperature sensors and reports battery voltage. Key architectural decisions:

- **Multi-endpoint design**: Each DS18B20 sensor gets its own Zigbee endpoint (1, 2, 3) but battery monitoring is only on endpoint 1 (shared for entire device)
- **Sleepy device pattern**: Uses `esp_zb_set_rx_on_when_idle(false)` for battery optimization with 10-second keep-alive
- **Delta-based reporting**: Temperature changes < 0.1°C are not reported to reduce battery drain
- **ROM address sorting**: DS18B20 sensors are sorted by ROM address for consistent endpoint assignment across reboots

## Critical Development Workflows

### Build & Flash Commands
```bash
# Quick development cycle
idf.py -p /dev/ttyACM0 build flash monitor

# Full reset (recommended for Zigbee - generates new IEEE address)
./erase_build_flash_monitor.sh
# OR manually:
idf.py -p /dev/ttyACM0 erase-flash && idf.py -p /dev/ttyACM0 build flash monitor
```

**Important**: Always use `erase-flash` between deployments. ESP32-C6 generates new IEEE addresses after NVS wipe, which ZHA sees as a completely new device.

### Device Port Detection
```bash
dmesg | grep tty  # Find which /dev/ttyACM* port to use
```

## Project-Specific Patterns

### GPIO Configuration
- **GPIO15**: Debug status LED (visual feedback for network state, sensor readings, errors)
- **GPIO8**: DS18B20 OneWire data bus (supports up to 3 sensors)
- **GPIO0**: Battery voltage monitoring via ADC1_CH0 (internal, voltage divider assumed in hardware)

### LED Status Codes
- 1 quick blink: Task started or sensor reported data
- 2 quick blinks: Temperature task started successfully  
- 3 long blinks: All sensors failed (critical error)
- 1 long repeating blink: Critical initialization failure
- Slow breathing: Network searching/connection issues

### Zigbee Cluster Structure
```c
// Temperature measurement cluster on each endpoint
ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT -> ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID

// Power config cluster ONLY on endpoint 1
ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG -> {
    ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID,      // Units of 100mV
    ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID  // 0-200 = 0-100%
}
```

### Battery Monitoring Pattern
- Voltage read via ADC with 2x multiplier (assumes hardware voltage divider)
- Percentage calculated linearly: 3.0V=0%, 4.2V=100%
- Reports hourly or on first reading for ZHA device discovery
- **Only reported to endpoint 1** - never duplicate across endpoints

### Configuration Constants (zigbee_temp_sensor.h)
- `TEMP_DELTA_THRESHOLD 10` - Report threshold: 0.1°C in hundredths
- `TEMP_REPORT_INTERVAL_MS 60000` - 60 seconds between sensor readings
- `BATTERY_REPORT_INTERVAL_MS 3600000` - 1 hour between battery reports  
- `ED_KEEP_ALIVE 10000` - Wake every 10 seconds for Zigbee maintenance

## Integration Points

### ESP-IDF Dependencies
- **ESP Zigbee Library**: `espressif/esp-zigbee-lib ~1.6.0` (HA profile, clusters)
- **ESP-IDF-LIB**: External component for DS18B20 driver (`ds18x20.h`)
- **Component exclusions**: `pcf8574` excluded in CMakeLists.txt

### Hardware Assumptions
- 3.3V logic level for OneWire bus
- Hardware voltage divider on battery input (2:1 ratio assumed in software)
- External pull-up resistor on GPIO8 OneWire bus

### Zigbee Network Configuration
- **Channel 25** only (`ESP_ZB_PRIMARY_CHANNEL_MASK`)
- **No install code** policy for easier development pairing
- **Home Automation Profile** (0x0104) with Temperature Sensor device type (0x0302)

## Common Gotchas

1. **IEEE Address**: Every `erase-flash` creates new device identity in ZHA
2. **Endpoint Battery**: Battery attributes must ONLY exist on endpoint 1, never duplicate
3. **ROM Sorting**: DS18B20 sensors are sorted by ROM address - physical sensor order may differ from endpoint order
4. **Sleep Timing**: Device sleeps between readings - debug prints may be missed during sleep periods
5. **ADC Calibration**: Battery readings depend on proper ADC calibration - verify with known voltages