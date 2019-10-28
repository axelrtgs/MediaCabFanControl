set(COMPONENT_SRCS
        "main.cpp"
        "lib/pwm_control/max31790.cpp"
        "lib/pwm_control/pwm_control.cpp"
        "lib/pid/pid.cpp"
        "lib/blufi/blufi.cpp"
        "lib/homekit/homekit.c"
        )

set(COMPONENT_ADD_INCLUDEDIRS
        "."
        "lib/pwm_control"
        "lib/pid"
        "lib/blufi"
        "lib/homekit"
        "lib/util"
        )

set(COMPONENT_REQUIRES nvs_flash bt homekit esp32-owb ds18b20 I2Cbus)

register_component()

component_compile_options(-std=c++17)