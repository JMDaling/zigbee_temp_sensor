cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/../tools/esp-idf-lib/components"
)
set(EXCLUDE_COMPONENTS "pcf8574")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(zigbee_temp_sensor)
