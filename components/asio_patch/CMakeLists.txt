idf_component_register(
    REQUIRES 
        esp_pm 
        sock_utils  # Make sock_utils available to all components using asio
)
 
# Add sock_utils to the public requires of asio
if(TARGET "espressif__asio")
    idf_component_get_property(asio_lib espressif__asio COMPONENT_LIB)
    target_link_libraries(${asio_lib} PUBLIC idf::sock_utils)
endif() 