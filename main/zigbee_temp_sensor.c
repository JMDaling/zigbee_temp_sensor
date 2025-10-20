#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_zigbee_core.h"
#include "ds18x20.h"
#include "zigbee_temp_sensor.h"

static const char *TAG = "ZIGBEE_TEMP_SENSOR";

/* DS18B20 */
static ds18x20_addr_t ds18b20_addrs[1];
static size_t sensor_count = 0;

/* Zigbee */
static bool connected = false;

/**
 * @brief Initialize DS18B20 temperature sensor
 */
static esp_err_t ds18b20_init(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Initializing DS18B20 on GPIO%d", DS18B20_GPIO);
    
    // Search for DS18B20 sensors
    ret = ds18x20_scan_devices(DS18B20_GPIO, ds18b20_addrs, 1, &sensor_count);
    if (ret != ESP_OK || sensor_count == 0) {
        ESP_LOGE(TAG, "No DS18B20 sensors found!");
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "Found %d DS18B20 sensor(s)", sensor_count);
    
    return ESP_OK;
}

/**
 * @brief Read temperature from DS18B20
 */
static esp_err_t read_temperature(float *temperature)
{
    esp_err_t ret;
    
    if (sensor_count == 0) {
        return ESP_ERR_NOT_FOUND;
    }
    
    // Trigger temperature conversion and read
    ret = ds18x20_measure_and_read(DS18B20_GPIO, ds18b20_addrs[0], temperature);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Temperature read failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

/**
 * @brief Update Zigbee temperature attribute
 */
static void update_zigbee_temperature(int16_t temperature_value)
{
    /* Update temperature measurement cluster */
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(
        HA_ESP_SENSOR_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        &temperature_value,
        false
    );
    esp_zb_lock_release();
    
    ESP_LOGI(TAG, "Updated Zigbee temperature attribute: %d (%.2f°C)", 
             temperature_value, temperature_value / 100.0f);
}

/**
 * @brief Temperature sensor task - reads DS18B20 and reports to Zigbee
 */
void temp_sensor_task(void *pvParameters)
{
    float temperature = 0.0f;
    int16_t zigbee_temp = 0;
    
    ESP_LOGI(TAG, "Temperature sensor task started");
    
    // Initialize DS18B20
    if (ds18b20_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize DS18B20, task will exit");
        vTaskDelete(NULL);
        return;
    }
    
    while (1) {
        // Read temperature
        if (read_temperature(&temperature) == ESP_OK) {
            // Validate temperature range
            if (temperature >= TEMP_SENSOR_MIN_VALUE && temperature <= TEMP_SENSOR_MAX_VALUE) {
                // Convert to Zigbee format (value * 100)
                zigbee_temp = (int16_t)(temperature * 100);
                
                ESP_LOGI(TAG, "Temperature: %.2f°C (Zigbee value: %d)", temperature, zigbee_temp);
                
                // Update Zigbee attribute if connected
                if (connected) {
                    update_zigbee_temperature(zigbee_temp);
                }
            } else {
                ESP_LOGW(TAG, "Temperature out of range: %.2f°C", temperature);
            }
        } else {
            ESP_LOGW(TAG, "Failed to read temperature");
        }
        
        // Wait before next reading
        vTaskDelay(pdMS_TO_TICKS(TEMP_REPORT_INTERVAL_MS));
    }
}

/**
 * @brief Zigbee signal handler
 */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
        
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", 
                     sig_type == ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START ? "" : "non");
            if (sig_type == ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
            }
        } else {
            ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
        
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
            connected = true;
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)esp_zb_bdb_start_top_level_commissioning, 
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
        
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", 
                 esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}

/**
 * @brief Create Zigbee temperature sensor endpoint
 */
static void create_temperature_sensor_ep(esp_zb_ep_list_t *ep_list)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    
    /* Basic cluster */
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(NULL);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, 
                                  (void *)ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME);
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, 
                                  (void *)ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER);
    uint8_t power_source = ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE;
    esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &power_source);
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    /* Identify cluster */
    esp_zb_attribute_list_t *identify_cluster = esp_zb_identify_cluster_create(NULL);
    esp_zb_cluster_list_add_identify_cluster(cluster_list, identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    /* Temperature measurement cluster */
    esp_zb_temperature_meas_cluster_cfg_t temp_cfg = {
        .measured_value = (int16_t)0x8000,  // Invalid value constant
        .min_value = (int16_t)(TEMP_SENSOR_MIN_VALUE * 100),
        .max_value = (int16_t)(TEMP_SENSOR_MAX_VALUE * 100),
    };
    esp_zb_attribute_list_t *temp_cluster = esp_zb_temperature_meas_cluster_create(&temp_cfg);
    esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, temp_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    /* Create endpoint */
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_ESP_SENSOR_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config);
}

/**
 * @brief Zigbee task
 */
void esp_zb_task(void *pvParameters)
{
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
    
    /* Create endpoint list */
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    create_temperature_sensor_ep(ep_list);
    esp_zb_device_register(ep_list);
    
    /* Configure reporting */
    esp_zb_zcl_reporting_info_t reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_SENSOR_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 0,
        .u.send_info.delta.u16 = 10,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 0,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_zcl_update_reporting_info(&reporting_info);
    
    /* Set commissioning settings */
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

/**
 * @brief Main application entry point
 */
void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = {.radio_mode = ZB_RADIO_MODE_NATIVE},
        .host_config = {.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE},
    };
    
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    
    ESP_LOGI(TAG, "Zigbee temperature sensor starting...");
    
    /* Start Zigbee stack task */
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
    
    /* Start temperature sensor task */
    xTaskCreate(temp_sensor_task, "temp_sensor", 4096, NULL, 4, NULL);
}
