set(COMPONENT_SRCS "main.cpp" "lib/I2Cbus/src/I2Cbus.cpp" "lib/MAX31790/MAX31790.cpp" "lib/FanControl/FanControl.cpp" "lib/PID/PID.cpp" "lib/blufi/blufi_security.c" "lib/blufi/blufi_wireless.c")
set(COMPONENT_ADD_INCLUDEDIRS "." "lib/I2Cbus/include" "lib/MAX31790" "lib/FanControl" "lib/PID" "lib/Misc" "lib/blufi")

register_component()