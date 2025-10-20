#ifndef ZIGBEE_TEMP_SENSOR_H
#define ZIGBEE_TEMP_SENSOR_H

#include "esp_zigbee_core.h"

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false    /* enable the install code policy for security */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   3000     /* 3000 millisecond */
#define HA_ESP_SENSOR_ENDPOINT          10       /* esp temperature sensor device endpoint */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK  /* Zigbee primary channel mask */

/* DS18B20 configuration */
#define DS18B20_GPIO                    8        /* GPIO8 for DS18B20 data pin */
#define TEMP_REPORT_INTERVAL_MS         30000    /* Report temperature every 30 seconds */
#define TEMP_SENSOR_MIN_VALUE           -20.0f   /* Minimum temperature value */
#define TEMP_SENSOR_MAX_VALUE           80.0f    /* Maximum temperature value */

/* Basic cluster attributes */
#define ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME      "Espressif"
#define ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER       "ESP32C6.TempSensor"
#define ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE           0x03  /* Battery */

/* Function declarations */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct);
void esp_zb_task(void *pvParameters);
void temp_sensor_task(void *pvParameters);

#endif // ZIGBEE_TEMP_SENSOR_H
