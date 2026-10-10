idf_component_register(SRCS
    "main.cpp"
    "link_sync.cpp"
    "io_helpers.cpp"
    "input_handler.cpp"
    "synth_mininova.cpp"
    "synth_microkorg.cpp"
    "state_machine.cpp"
    "bass_engine.cpp"
    "bassline_interpreter.cpp"
    "wifi_config.cpp"

    "network_midi.cpp"
    REQUIRES
        nvs_flash
        esp_netif
        esp_event
        esp_wifi
        esp_http_server
        esp_eth
        spiffs
        driver
        log
        freertos
        esp_adc

        esp_driver_gptimer
        link-esp
        protocol_examples_common
    PRIV_REQUIRES
        esp_driver_touch_sens
        esp_driver_gpio
        esp_driver_uart
        esp_driver_ledc
)

set_property(TARGET __idf_main PROPERTY CXX_STANDARD 17)

target_compile_options(__idf_main PRIVATE -Wno-error=cpp)
target_compile_options(__idf_main PRIVATE -Os -ffunction-sections -fdata-sections)
target_link_options(__idf_main PRIVATE -Wl,--gc-sections)

target_include_directories(__idf_main PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../components/link-esp/link/include
)

target_include_directories(__idf_main PRIVATE
    $ENV{IDF_PATH}/components/esp_driver_touch_sens/include
)

target_include_directories(__idf_main PRIVATE
    $ENV{IDF_PATH}/components/driver/include
    $ENV{IDF_PATH}/components/esp_driver_gptimer/include
)
