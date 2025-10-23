idf_component_register(
    SRCS "zigbee_temp_sensor.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        nvs_flash
        esp-zigbee-lib
        onewire 
        ds18x20
        esp_adc
)
