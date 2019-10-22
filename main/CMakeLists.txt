set(COMPONENT_SRCS
        "main.cpp"
        "lib/I2Cbus/src/I2Cbus.cpp"
        "lib/MAX31790/MAX31790.cpp"
        "lib/FanControl/FanControl.cpp"
        "lib/PID/PID.cpp"
        "lib/blufi/blufi.cpp"
        "lib/homekit/homekit.c"
        )

set(COMPONENT_ADD_INCLUDEDIRS
        "."
        "lib/I2Cbus/include"
        "lib/MAX31790"
        "lib/FanControl"
        "lib/PID"
        "lib/Misc"
        "lib/blufi"
        "lib/homekit"
        "lib/util"
        )

set(COMPONENT_REQUIRES nvs_flash bt homekit owb ds18b20)

register_component()

#component_compile_options(-std=c++17)