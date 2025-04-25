#pragma once

#include <cstdint>
#include "GPIO_RPI_HAL.h"

namespace Linux {

/**
 * @brief Class for Jetson Orin Nano GPIO control
 *
 *  For more information:
 *    - Orin TRM
 *    - GPIO Controller: tegra234-gpio (base address 348) 
 */
class GPIO_ORIN_NANO : public GPIO_RPI_HAL {
public:
    GPIO_ORIN_NANO();
    void init() override;
    void pinMode(uint8_t pin, uint8_t mode) override;
    void pinMode(uint8_t pin, uint8_t mode, uint8_t alt) override;
    uint8_t read(uint8_t pin) override;
    void write(uint8_t pin, uint8_t value) override;
    void toggle(uint8_t pin) override;

    enum class PadsPull : uint8_t {
        Off = 0,
        Down = 1,
        Up = 2,
    };
    void set_pull(uint8_t pin, PadsPull mode);

private:
    static constexpr const char* PATH_DEV_GPIOMEM = "/dev/gpiochip0";
    static constexpr uint32_t BASE_ADDRESS = 348;
    static constexpr uint32_t MEM_SIZE = 0x10000; // Assuming a reasonable memory size
    static constexpr uint32_t REG_SIZE = 4; // 32-bit registers

    // Register Offsets: (These are placeholder values, you'll need to verify with the Orin TRM)
    static constexpr uint32_t GPIO_DIRECTION_OFFSET = 0x0000; 
    static constexpr uint32_t GPIO_OUTPUT_OFFSET = 0x0004; 
    static constexpr uint32_t GPIO_INPUT_OFFSET = 0x0008; 

    // GPIO Control from Orin TRM (You'll need to fill these with the correct values)
    static constexpr uint32_t CTRL_FUNCSEL_MASK = 0x001f; 
    static constexpr uint32_t CTRL_FUNCSEL_LSB = 0; 

    static constexpr uint32_t PADS_GPIO_OFFSET = 0x0010; // Placeholder
    static constexpr uint32_t PADS_PULL_MASK = 0x0c;
    static constexpr uint32_t PADS_PULL_LSB = 2; 
    static constexpr uint32_t PADS_IN_ENABLE_MASK = 0x40; 
    static constexpr uint32_t PADS_OUT_DISABLE_MASK = 0x80; 

    enum class FunctionSelect : uint8_t {
        Alt0 = 0,
        Alt1 = 1,
        Alt2 = 2,
        Alt3 = 3,
        Alt4 = 4,
        Alt5 = 5,
        Alt6 = 6,
        Alt7 = 7,
        Alt8 = 8,
        Null = 31
    };

    enum Mode {
        Input,
        Output,
        Alt0,
        Alt1,
        Alt2,
        Alt3,
        Alt4,
        Alt5,
        Alt6,
        Alt7,
        Alt8,
        Null
    };

    enum Bias {
        Off,
        PullDown,
        PullUp
    };

    volatile uint32_t* _gpio;
    int _system_memory_device;
    uint32_t _gpio_output_port_status = 0;

    bool openMemoryDevice();
    void closeMemoryDevice();
    volatile uint32_t* get_memory_pointer(uint32_t address, uint32_t range) const;

    uint32_t read_register(uint32_t offset) const;
    void write_register(uint32_t offset, uint32_t value);

    Mode direction(uint8_t pin) const;
    void set_direction(uint8_t pin, Mode mode);
    void input_enable(uint8_t pin);
    void input_disable(uint8_t pin);
    void output_enable(uint8_t pin);
    void output_disable(uint8_t pin);

    void set_mode(uint8_t pin, Mode mode);
};

}