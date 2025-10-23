#ifndef ZIGBEE_TEMP_SENSOR_H
#define ZIGBEE_TEMP_SENSOR_H

#include "esp_zigbee_core.h"

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false    /* Disable install code for easier pairing */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   3000     /* 3000 millisecond */
#define HA_TEMP_SENSOR_ENDPOINT_BASE    10       /* Base endpoint (10, 11, 12 for 3 sensors) */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

/* DS18B20 configuration */
#define DS18B20_GPIO                    8        /* GPIO8 for DS18B20 data pin */
#define MAX_SENSORS                     3        /* Maximum number of DS18B20 sensors */
#define TEMP_REPORT_INTERVAL_MS         60000    /* Report temperature every 60 seconds */
#define TEMP_DELTA_THRESHOLD            10       /* Report when temperature changes by 0.1°C (10 hundredths) */
#define TEMP_SENSOR_MIN_VALUE           -20.0f   /* Minimum temperature in Celsius */
#define TEMP_SENSOR_MAX_VALUE           80.0f    /* Maximum temperature in Celsius */

/* Basic cluster attributes */
#define MANUFACTURER_NAME               "\x09""ESPRESSIF"
#define MODEL_IDENTIFIER                "\x07"CONFIG_IDF_TARGET

#endif // ZIGBEE_TEMP_SENSOR_H