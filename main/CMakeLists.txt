idf_component_register(
    SRCS "zigbee_temp_sensor.c"
    INCLUDE_DIRS "."
    REQUIRES 
        nvs_flash
        ds18x20
)
