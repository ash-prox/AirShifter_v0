idf_component_register(SRCS "wifi_manager.c" "main.c" "gatt_svr.c"
                    PRIV_REQUIRES bt nvs_flash
                    INCLUDE_DIRS ".")
