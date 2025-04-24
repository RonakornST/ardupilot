#pragma once

#include <stdint.h>
#include "AP_HAL_Linux.h"
#include "GPIO_JETSON_ORIN_NN.h"

namespace Linux {

/**
 * @brief Class for NVIDIA Jetson GPIO control
 *
 */
class GPIO_JETSON : public AP_HAL::GPIO {
public:
    GPIO_JETSON();
    void    init() override;
    void    pinMode(uint8_t pin, uint8_t output) override;
    void    pinMode(uint8_t pin, uint8_t output, uint8_t alt) override;
    uint8_t read(uint8_t pin) override;
    void    write(uint8_t pin, uint8_t value) override;
    void    toggle(uint8_t pin) override;

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n) override;

    /* return true if USB cable is connected */
    bool    usb_connected(void) override;

private:
    GPIO_JETSON_ORIN_NN* gpioDriver;
};

}
