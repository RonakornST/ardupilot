#pragma once

#include <stdint.h>
#include "AP_HAL_Linux.h"
#include "GPIO_ORIN_HAL.h"

/**
 * @brief Check for valid Jetson Orin pin range
 *
 * @tparam pin
 * @return uint8_t
 */
template <uint8_t pin> constexpr uint8_t ORIN_GPIO_()
{
    static_assert(pin >= 0 && pin < 256, "Invalid pin value."); // Adjust based on Orin GPIO range
    return pin;
}

namespace Linux {

/**
 * @brief Class for Jetson Orin GPIO control
 *
 */
class GPIO_ORIN : public AP_HAL::GPIO {
public:
    GPIO_ORIN();
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
    GPIO_ORIN_HAL* gpioDriver;
};

}
