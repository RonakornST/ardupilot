#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ORIN_V1
//#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ORIN_V1

#include "GPIO.h"
#include "GPIO_ORIN.h"
#include "GPIO_ORIN_NANO.h"

extern const AP_HAL::HAL& hal;

using namespace Linux;

GPIO_ORIN::GPIO_ORIN() {}

void GPIO_ORIN::init()
{
    // Initialize GPIO using memory-mapped I/O (mmap)
    gpioDriver = NEW_NOTHROW GPIO_ORIN_NANO();
    if (!gpioDriver) {
        AP_HAL::panic("Failed to allocate GPIO_ORIN_NANO instance");
        return;
    }
    gpioDriver->init();
}

void GPIO_ORIN::pinMode(uint8_t pin, uint8_t output)
{
    gpioDriver->pinMode(pin, output);
}

void GPIO_ORIN::pinMode(uint8_t pin, uint8_t output, uint8_t alt)
{
    gpioDriver->pinMode(pin, output, alt);
}

uint8_t GPIO_ORIN::read(uint8_t pin)
{
    return gpioDriver->read(pin);
}

void GPIO_ORIN::write(uint8_t pin, uint8_t value)
{
    gpioDriver->write(pin, value);
}

void GPIO_ORIN::toggle(uint8_t pin)
{
    gpioDriver->toggle(pin);
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO_ORIN::channel(uint16_t n)
{
    return NEW_NOTHROW DigitalSource(n);
}

bool GPIO_ORIN::usb_connected(void)
{
    return false;
}

#endif
