#ifndef ZIGBEE_TEMP_SENSOR_H
#define ZIGBEE_TEMP_SENSOR_H

#include "esp_zigbee_core.h"

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false    /* Disable install code for easier pairing */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   10000    /* Wake up every 10 seconds */
#define HA_TEMP_SENSOR_ENDPOINT_BASE    10       /* Base endpoint (10, 11, 12 for 3 sensors) */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     (1 << 25) /* Channel 25 */

/* DS18B20 configuration */
#define DS18B20_GPIO                    8        /* GPIO8 for DS18B20 data pin */
#define MAX_SENSORS                     3        /* Maximum number of DS18B20 sensors */
#define TEMP_REPORT_INTERVAL_MS         60000    /* Report temperature every 60 seconds */
#define TEMP_DELTA_THRESHOLD            10       /* Report when temperature changes by 0.1°C (10 hundredths) */
#define TEMP_SENSOR_MIN_VALUE           -20.0f   /* Minimum temperature in Celsius */
#define TEMP_SENSOR_MAX_VALUE           80.0f    /* Maximum temperature in Celsius */

/* Battery monitoring configuration */
#define BATTERY_ADC_UNIT                ADC_UNIT_1
#define BATTERY_ADC_CHANNEL             ADC_CHANNEL_0   /* GPIO0 = ADC1_CH0 */
#define BATTERY_ADC_ATTEN               ADC_ATTEN_DB_12 /* 0-3.3V range */
#define BATTERY_MIN_VOLTAGE             3000    /* 3.0V = 0% (mV) */
#define BATTERY_MAX_VOLTAGE             4200    /* 4.2V = 100% (mV) */
#define BATTERY_REPORT_INTERVAL_MS      3600000 /* Report battery every 1 hour */

/* Basic cluster attributes */
#define MANUFACTURER_NAME               "\x09""ESPRESSIF"
#define MODEL_IDENTIFIER                "\x07"CONFIG_IDF_TARGET

#endif // ZIGBEE_TEMP_SENSOR_H