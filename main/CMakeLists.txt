# For more information about building ESP-IDF components, please refer to
# https://docs.espressif.com/projects/esp-idf/en/latest/esp32/develop-manuals/build_system.html#building-components

idf_component_register(SRCS
    "main.cpp"
    "link_sync.cpp"
    "io_helpers.cpp"
    "input_handler.cpp"
    "effect_handler.cpp"
    "effect_arp.cpp"
    "effect_filter.cpp"
    "effect_sidechain.cpp"
    "synth_mininova.cpp"
    "synth_microkorg.cpp"
    "state_machine.cpp"
    "midi_file.cpp"
    REQUIRES
        # Required IDF components
        nvs_flash
        esp_netif
        esp_event
        esp_wifi
        esp_eth
        driver
        log
        freertos
        esp_adc
        # esp_driver_touch_sens # Moved to PRIV_REQUIRES
        # Required 3rd-party components
        link-esp
        protocol_examples_common
    PRIV_REQUIRES
        esp_driver_touch_sens
        spiffs
)

# Set C++ standard to C++17 for this component
set_property(TARGET __idf_main PROPERTY CXX_STANDARD 17)

# Add size optimization flags
target_compile_options(__idf_main PRIVATE -Os -ffunction-sections -fdata-sections)
# Add linker optimization flags to remove dead code
target_link_options(__idf_main PRIVATE -Wl,--gc-sections)

# Add explicit include directory for Ableton Link
target_include_directories(__idf_main PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/link-esp/link/include
)

# Add explicit include directory for Touch Sensor
target_include_directories(__idf_main PRIVATE
    $ENV{IDF_PATH}/components/esp_driver_touch_sens/include
)

# Generate SPIFFS image from data directory
spiffs_create_partition_image(storage ${CMAKE_CURRENT_SOURCE_DIR}/../data FLASH_IN_PROJECT)
