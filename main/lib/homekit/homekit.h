#pragma once
enum target_state_value {
    OFF = 0,
    HEAT = 1,
    COOL= 2,
    AUTO = 3,
};

typedef struct {
    float cur_temp;
    float target_temp;
    enum target_state_value mode;
} fan_kit;

fan_kit* extern_values;

void homekit_init(fan_kit* fanKit);