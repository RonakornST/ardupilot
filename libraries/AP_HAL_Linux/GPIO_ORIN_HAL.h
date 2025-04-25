#pragma once
//#include <cstdint>
// include <cstdint> to make uint8_t available
class GPIO_ORIN_HAL {
public:
    GPIO_ORIN_HAL() {}
    virtual void    init() = 0;
    virtual void    pinMode(uint8_t pin, uint8_t output) = 0;
    virtual void    pinMode(uint8_t pin, uint8_t output, uint8_t alt) {};

    virtual uint8_t read(uint8_t pin) = 0;
    virtual void    write(uint8_t pin, uint8_t value) = 0;
    virtual void    toggle(uint8_t pin) = 0;
};
