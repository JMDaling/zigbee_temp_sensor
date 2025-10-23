#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "ds18x20.h"
#include "zigbee_temp_sensor.h"
#include "driver/gpio.h"
#include "esp_pm.h"  // Add this at the top with other includes

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile sensor (End Device) source code.
#endif

static const char *TAG = "ESP_ZB_TEMP_SENSOR";

/* Debug LED */
#define DEBUG_LED_GPIO                 15        /* GPIO15 for status LED */

/* DS18B20 */
static ds18x20_addr_t ds18b20_addrs[MAX_SENSORS];
static size_t sensor_count = 0;
static float last_reported_temps[MAX_SENSORS] = {0};  // Track last reported temp for each sensor

static int16_t zb_temperature_to_s16(float temp)
{
    return (int16_t)(temp * 100);
}

static void esp_app_temp_sensor_handler(uint8_t sensor_index, float temperature)
{
    if (sensor_index >= sensor_count) {
        ESP_LOGW(TAG, "Invalid sensor index: %d", sensor_index);
        return;
    }
    
    int16_t measured_value = zb_temperature_to_s16(temperature);
    uint8_t endpoint = HA_TEMP_SENSOR_ENDPOINT_BASE + sensor_index;
    
    /* Update temperature sensor measured value */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(endpoint,
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &measured_value, false);
    esp_zb_lock_release();
}

static int compare_rom_addresses(const void *a, const void *b)
{
    ds18x20_addr_t addr_a = *(ds18x20_addr_t*)a;
    ds18x20_addr_t addr_b = *(ds18x20_addr_t*)b;
    
    if (addr_a < addr_b) return -1;
    if (addr_a > addr_b) return 1;
    return 0;
}

static esp_err_t ds18b20_init(void)
{
    ESP_LOGI(TAG, "Initializing DS18B20 on GPIO%d", DS18B20_GPIO);
    
    // ds18x20_scan_devices needs 4 arguments including 'found' output parameter
    esp_err_t res = ds18x20_scan_devices(DS18B20_GPIO, ds18b20_addrs, MAX_SENSORS, &sensor_count);
    
    if (res != ESP_OK || sensor_count == 0) {
        ESP_LOGE(TAG, "No DS18B20 sensors found!");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Found %d DS18B20 sensor(s)", sensor_count);

    // Sort sensors by ROM address for consistent indexing
    qsort(ds18b20_addrs, sensor_count, sizeof(ds18x20_addr_t), compare_rom_addresses);

    ESP_LOGI(TAG, "Sensors sorted by ROM address:");
    
    // Log each sensor's ROM address - use correct format specifiers
    for (size_t i = 0; i < sensor_count; i++) {
        ESP_LOGI(TAG, "Sensor %d ROM: %016llX", i, (unsigned long long)ds18b20_addrs[i]);
    }
    
    return ESP_OK;
}

static void debug_led_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DEBUG_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(DEBUG_LED_GPIO, 0);
}

static void led_blink(int times, uint32_t duration_ms, uint32_t pause_ms)
{
    for (int i = 0; i < times; i++) {
        gpio_set_level(DEBUG_LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        gpio_set_level(DEBUG_LED_GPIO, 0);
        if (i < times - 1) {
            vTaskDelay(pdMS_TO_TICKS(pause_ms));
        }
    }
}

static void led_blink_quick(int times)
{
    led_blink(times, 100, 100);  // 100ms on, 100ms off
}

// static void led_blink_normal(int times)
// {
//     led_blink(times, 200, 200);  // 200ms on, 200ms off
// }

static void led_blink_long(int times)
{
    led_blink(times, 800, 400);  // 800ms on, 400ms off
}

static void temp_sensor_task(void *pvParameters)
{
    bool first_reading[MAX_SENSORS] = {true, true, true};
    bool any_reported = false;  // ADD THIS LINE - declare the variable
    
    ESP_LOGI(TAG, "Temperature sensor task started (threshold: 0.1°C, interval: %dms)", 
             TEMP_REPORT_INTERVAL_MS);
    
    led_blink_quick(2);  // 2 quick blinks = task started successfully

    // Pause before sensor init (helps for debugging by splitting LED indications)
    vTaskDelay(pdMS_TO_TICKS(500));
    
    if (ds18b20_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize DS18B20");
        while(1) {
            led_blink_long(1);  // 1 long blink repeating = critical error
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        vTaskDelete(NULL);
        return;
    }
    
    // Single long blink = sensors found OK
    led_blink(1, 1000, 0);  // 1 second blink

    ESP_LOGI(TAG, "Starting temperature measurement loop, reporting every %d seconds", TEMP_REPORT_INTERVAL_MS / 1000);
    
    uint32_t error_count = 0;
    
    while (1) {
        bool any_read_success = false;
        any_reported = false;  // Reset at start of each cycle
        
        for (size_t i = 0; i < sensor_count; i++) {
            float temperature = 0.0f;
            esp_err_t ret = ds18x20_measure_and_read(DS18B20_GPIO, ds18b20_addrs[i], &temperature);
            
            if (ret == ESP_OK) {
                any_read_success = true;
                
                if (temperature >= TEMP_SENSOR_MIN_VALUE && temperature <= TEMP_SENSOR_MAX_VALUE) {
                    
                    float temp_diff = fabs(temperature - last_reported_temps[i]);
                    float threshold = TEMP_DELTA_THRESHOLD / 100.0f;

                    if (first_reading[i] || (temp_diff >= threshold)) {
                        esp_app_temp_sensor_handler(i, temperature);
                        last_reported_temps[i] = temperature;
                        first_reading[i] = false;
                        any_reported = true;
                        ESP_LOGI(TAG, "📤 Sensor %d REPORTED: %.2f°C (Δ%.2f°C)", 
                                 i, temperature, temp_diff);
                    } else {
                        ESP_LOGI(TAG, "📊 Sensor %d Read: %.2f°C (Δ%.2f°C) - no report (< 0.1°C)", 
                                 i, temperature, temp_diff);
                    }
                }
            } else {
                ESP_LOGW(TAG, "Failed to read temperature from sensor %d", i);
                error_count++;
            }
        }

        // Single quick blink if any sensor reported
        if (any_reported) {
            led_blink_quick(1);
        }

        // 3 long blinks if ALL sensors failed
        if (!any_read_success) {
            ESP_LOGE(TAG, "❌ All sensors failed!");
            led_blink_long(3);  // 3 long blinks = error
            error_count++;
        } else if (error_count > 0) {
            error_count = 0;  // Reset error counter on successful read
        }
        
        vTaskDelay(pdMS_TO_TICKS(TEMP_REPORT_INTERVAL_MS));
    }
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, ,
                        TAG, "Failed to start Zigbee bdb commissioning");
}

static esp_err_t deferred_driver_init(void)
{
    static bool is_inited = false;
    if (!is_inited) {
        /* Start temperature sensor task */
        xTaskCreate(temp_sensor_task, "temp_sensor", 4096, NULL, 4, NULL);
        is_inited = true;
    }
    return is_inited ? ESP_OK : ESP_FAIL;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p     = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "🔧 Initializing Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
        
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "🔍 Starting network steering (searching for networks)");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "♻️  Device reconnecting to existing network");
                ESP_LOGI(TAG, "Starting temperature sensor task (reconnect)");
                deferred_driver_init();
            }
        } else {
            ESP_LOGW(TAG, "⚠️  Device start failed, retrying...");
            // Slow blink while retrying
            gpio_set_level(DEBUG_LED_GPIO, 1);
            for (volatile int i = 0; i < 4000000; i++); // ~500ms on
            gpio_set_level(DEBUG_LED_GPIO, 0);
            
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
        
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "✅ JOINED NETWORK - PAN: 0x%04hx, Channel: %d, Addr: 0x%04hx",
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            
            // Three quick blinks to indicate successful join
            for (size_t i = 0; i < 3; i++)
            {
                gpio_set_level(DEBUG_LED_GPIO, 1);
                for (volatile int j = 0; j < 4000000; j++); // ~500ms on
                gpio_set_level(DEBUG_LED_GPIO, 0);
                for (volatile int j = 0; j < 4000000; j++); // ~500ms off
            }

            ESP_LOGI(TAG, "Starting temperature sensor task (new join)");
            deferred_driver_init();
            
        } else {
            ESP_LOGW(TAG, "🔄 Still searching for network...");

            // Slow blink while searching for network
            gpio_set_level(DEBUG_LED_GPIO, 1);
            for (volatile int i = 0; i < 12000000; i++); // ~1 second
            gpio_set_level(DEBUG_LED_GPIO, 0);
            
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, 
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
        
    default:
        if (sig_type != ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY) {
            ESP_LOGD(TAG, "ZDO signal: %s (0x%x)", esp_zb_zdo_signal_to_string(sig_type), sig_type);
        }
        break;
    }
}

static esp_zb_cluster_list_t *custom_temperature_sensor_clusters_create(esp_zb_temperature_sensor_cfg_t *temperature_sensor)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&(temperature_sensor->basic_cfg));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&(temperature_sensor->identify_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, esp_zb_temperature_meas_cluster_create(&(temperature_sensor->temp_meas_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    return cluster_list;
}

static void esp_zb_task(void *pvParameters)
{
    /* Initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,
        .nwk_cfg = {
            .zed_cfg = {
                .ed_timeout = ED_AGING_TIMEOUT,
                .keep_alive = ED_KEEP_ALIVE,
            },
        },
    };
    esp_zb_init(&zb_nwk_cfg);

    /* Create endpoint list with multiple temperature sensors */
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    
    /* Create temperature sensor endpoint for each potential sensor */
    for (uint8_t i = 0; i < MAX_SENSORS; i++) {
        esp_zb_temperature_sensor_cfg_t sensor_cfg = ESP_ZB_DEFAULT_TEMPERATURE_SENSOR_CONFIG();
        sensor_cfg.temp_meas_cfg.min_value = zb_temperature_to_s16(TEMP_SENSOR_MIN_VALUE);
        sensor_cfg.temp_meas_cfg.max_value = zb_temperature_to_s16(TEMP_SENSOR_MAX_VALUE);
        
        uint8_t endpoint_id = HA_TEMP_SENSOR_ENDPOINT_BASE + i;
        
        // Create clusters and endpoint config
        esp_zb_cluster_list_t *cluster_list = custom_temperature_sensor_clusters_create(&sensor_cfg);
        esp_zb_endpoint_config_t endpoint_config = {
            .endpoint = endpoint_id,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
            .app_device_version = 0
        };
        
        // Add endpoint to the list
        esp_zb_ep_list_add_ep(esp_zb_ep_list, cluster_list, endpoint_config);
    }

    /* Register the device */
    esp_zb_device_register(esp_zb_ep_list);

    /* Config the reporting info for all endpoints */
    for (uint8_t i = 0; i < MAX_SENSORS; i++) {
        esp_zb_zcl_reporting_info_t reporting_info = {
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
            .ep = HA_TEMP_SENSOR_ENDPOINT_BASE + i,
            .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
            .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .u.send_info.min_interval = 1,
            .u.send_info.max_interval = 0,
            .u.send_info.def_min_interval = 1,
            .u.send_info.def_max_interval = 0,
            .u.send_info.delta.u16 = TEMP_DELTA_THRESHOLD,
            .attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
            .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        };
        esp_zb_zcl_update_reporting_info(&reporting_info);
    }
    
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));

    esp_zb_stack_main_loop();
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = {.radio_mode = ZB_RADIO_MODE_NATIVE},
        .host_config = {.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE},
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    
    // Initialize LED early
    debug_led_init();
    led_blink_quick(1);
    
    ESP_LOGI("POWER", "Running at 80MHz with tickless idle for battery savings");
    
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    /* Start Zigbee stack task */
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}