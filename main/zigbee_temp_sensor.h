#ifndef ZIGBEE_TEMP_SENSOR_H
#define ZIGBEE_TEMP_SENSOR_H

#include "esp_zigbee_core.h"

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false    /* Disable install code for easier pairing */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   3000     /* 3000 millisecond */
#define HA_ESP_SENSOR_ENDPOINT          10       /* esp temperature sensor device endpoint */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK  /* Scan all channels */

/* DS18B20 configuration */
#define DS18B20_GPIO                    8        /* GPIO8 for DS18B20 data pin */
#define TEMP_REPORT_INTERVAL_MS         60000    /* Report temperature every 60 seconds */
#define TEMP_DELTA_THRESHOLD            10       /* Report when temperature changes by 0.1°C (10 hundredths) */
#define TEMP_SENSOR_MIN_VALUE           -20.0f   /* Minimum temperature value */
#define TEMP_SENSOR_MAX_VALUE           80.0f    /* Maximum temperature value */

/* Basic cluster attributes */
#define MANUFACTURER_NAME               "\x09""ESPRESSIF"
#define MODEL_IDENTIFIER                "\x07"CONFIG_IDF_TARGET

#endif // ZIGBEE_TEMP_SENSOR_H