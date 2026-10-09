#pragma once

#include <Arduino.h>
#include <SparkFun_APDS9960.h>

enum class ApdsGesture : uint8_t {
    None,
    Up,
    Down,
    Left,
    Right
};

class ApdsSensor {
public:
    bool begin(uint8_t& chipId);
    bool readProximity(uint8_t& proximity);
    ApdsGesture readGesture();

private:
    SparkFun_APDS9960 sensor_;
};
