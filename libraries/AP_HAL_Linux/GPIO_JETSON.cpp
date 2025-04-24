#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_JETSON_ORIN_NANO

#include "GPIO_JETSON.h"

using namespace Linux;

extern const AP_HAL::HAL& hal;

GPIO_JETSON::GPIO_JETSON()
{
    gpioDriver = new GPIO_JETSON_ORIN_NN();
}

void GPIO_JETSON::init()
{
    gpioDriver->init();
}

void GPIO_JETSON::pinMode(uint8_t pin, uint8_t output)
{
    gpioDriver->pinMode(pin, output);
}

void GPIO_JETSON::pinMode(uint8_t pin, uint8_t output, uint8_t alt)
{
    gpioDriver->pinMode(pin, output, alt);
}

uint8_t GPIO_JETSON::read(uint8_t pin)
{
    return gpioDriver->read(pin);
}

void GPIO_JETSON::write(uint8_t pin, uint8_t value)
{
    gpioDriver->write(pin, value);
}

void GPIO_JETSON::toggle(uint8_t pin)
{
    gpioDriver->toggle(pin);
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO_JETSON::channel(uint16_t n)
{
    return NEW_NOTHROW DigitalSource(n);
}

bool GPIO_JETSON::usb_connected(void)
{
    return false;
}

#endif // CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_JETSON_ORIN_NANO
