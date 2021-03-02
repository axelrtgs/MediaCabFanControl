idf_component_register(
                        SRCS
                          "main.cpp"
                          "lib/pwm_control/max31790.cpp"
                          "lib/pwm_control/pwm_control.cpp"
                          "lib/pid/pid.cpp"
                          "lib/ota/ota.cpp"
                          "lib/httpd/httpd.cpp"
                          "lib/mqtt/mqtt.cpp"
                          "lib/config/config.cpp"
                          "lib/wifi_prov_mgr/wifi_prov_mgr.cpp"
                          # "lib/homekit/homekit.c"
                        INCLUDE_DIRS
                          "."
                          "lib/pwm_control"
                          "lib/pid"
                          "lib/ota"
                          "lib/httpd"
                          "lib/mqtt"
                          "lib/config"
                          "lib/wifi_prov_mgr"
                          # "lib/homekit"
                          "lib/util"
                        REQUIRES
                          nvs_flash
                          wifi_provisioning
                          homekit
                          esp32-owb
                          ds18b20
                          I2Cbus
                          mqtt
                          json
                          spiffs
                          esp_http_client
                          esp_http_server
                          app_update
                        EMBED_TXTFILES
                          ${project_dir}/data/ca_cert.pem)
                        )

target_compile_definitions(${COMPONENT_TARGET} PRIVATE
    "-DBLE2MQTT_VER=\"${PROJECT_VER}\"")
target_include_directories(${COMPONENT_LIB} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# Static files to be served by the web server
file(GLOB_RECURSE www_files RELATIVE ${PROJECT_DIR}/www ${PROJECT_DIR}/www/*)
foreach(file ${www_files})
    string(MAKE_C_IDENTIFIER ${file} c_name)
    string(APPEND static_files_list
        "{ \"/${file}\", ${c_name}_start, ${c_name}_end },\n    ")
    string(APPEND static_files_declerations
        "extern const char ${c_name}_start[] asm(\"_binary_${c_name}_start\");\n")
    string(APPEND static_files_declerations
        "extern const char ${c_name}_end[] asm(\"_binary_${c_name}_end\");\n")
    add_custom_command(OUTPUT ${build_dir}/www/${file}.gz
        COMMAND gzip -c ${PROJECT_DIR}/www/${file} > ${build_dir}/www/${file}.gz
        DEPENDS ${PROJECT_DIR}/www/${file})
    target_add_binary_data(${COMPONENT_LIB} ${build_dir}/www/${file}.gz
        BINARY RENAME_TO ${c_name})
endforeach()
configure_file(lib/httpd/httpd_static_files.h.in httpd_static_files.h)
