idf_component_register( 
    SRCS 
        "src/app_events.c" 
        "src/queue_manager.c" 
        "src/protocol.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos log esp_event nvs_flash
)
