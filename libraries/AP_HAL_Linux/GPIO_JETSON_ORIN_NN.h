#pragma once

#include <stdint.h>
#include "GPIO_RPI_HAL.h"

namespace Linux {

/**
 * @brief Class for NVIDIA Jetson Orin Nano GPIO control
 *
 * This implementation uses memory-mapped I/O to control the GPIO pins
 * on the Jetson Orin Nano.
 *
 * For more information:
 * - Jetson Orin Nano Technical Reference Manual
 * - NVIDIA Jetson Linux Driver Package Documentation
 */
class GPIO_JETSON_ORIN_NN : public GPIO_RPI_HAL {
public:
    GPIO_JETSON_ORIN_NN();
    void    init() override;
    void    pinMode(uint8_t pin, uint8_t output) override;
    void    pinMode(uint8_t pin, uint8_t output, uint8_t alt) override;
    uint8_t read(uint8_t pin) override;
    void    write(uint8_t pin, uint8_t value) override;
    void    toggle(uint8_t pin) override;

private:
    // Jetson Orin Nano GPIO domains
    enum class GPIODomain {
        NON_AON,  // Non-AON domain (CTL0-CTL5)
        AON,      // AON domain (AON)
        FSI       // FSI domain (FSI_CTL0, FSI_CTL1)
    };

    // Jetson Orin Nano GPIO controllers within each domain
    enum class GPIOController {
        // Non-AON domain controllers
        CTL0,
        CTL1,
        CTL2,
        CTL3,
        CTL4,
        CTL5,

        // AON domain controller
        AON,

        // FSI domain controllers
        FSI_CTL0,
        FSI_CTL1
    };

    // Domain base addresses (need to be verified from TRM)
    struct GPIODomainBase {
        static const uint32_t NON_AON = 0x2200000;  // Base address for Non-AON domain
        static const uint32_t AON     = 0x2440000;  // Base address for AON domain
        static const uint32_t FSI     = 0x2450000;  // Base address for FSI domain
    };

    // Controller offset within domain (4-KiB per controller)
    static const uint32_t CONTROLLER_SIZE = 0x1000;  // 4-KiB per controller

    // Port offset within controller
    static const uint32_t PORT_SIZE = 0x100;  // Size of each port's register space

    // Register offsets within a port
    struct GPIORegisterOffsets {
        // Based on the register information provided (GPIO_<iii>_OUTPUT_VALUE_<j>_0)
        static const uint32_t OUTPUT_VALUE = 0x10;  // Offset for OUTPUT_VALUE register
        static const uint32_t INPUT_VALUE  = 0x20;  // Placeholder - needs verification
        static const uint32_t ENABLE_CONFIG = 0x30; // Placeholder - needs verification
        static const uint32_t OUTPUT_CONTROL = 0x40; // Placeholder - needs verification
        static const uint32_t INPUT_CONTROL = 0x50;  // Placeholder - needs verification
    };

    /**
     * @brief Open memory device to allow gpio address access
     *  Should be used before get_memory_pointer calls in the initialization
     *
     * @return true on success, false on failure
     */
    bool openMemoryDevice();

    /**
     * @brief Close open memory device
     */
    void closeMemoryDevice();

    /**
     * @brief Return pointer to memory location with specific range access
     *
     * @param address Memory address to map
     * @param range Size of memory region to map
     * @return volatile uint32_t* Pointer to mapped memory region
     */
    volatile uint32_t* get_memory_pointer(uint32_t address, uint32_t range) const;

    /**
     * @brief Converts a pin number to domain, controller, port, and bit position
     *
     * @param pin GPIO pin number (physical pin)
     * @param domain Output: GPIO domain (NON_AON, AON, FSI)
     * @param controller Output: GPIO controller within the domain
     * @param port Output: Port number (0-7)
     * @param bit Output: Bit position within port (0-7)
     * @return true if conversion successful, false if pin is invalid
     */
    bool pin_to_domain_controller_port_bit(uint8_t pin, GPIODomain& domain, GPIOController& controller, uint8_t& port, uint8_t& bit) const;

    /**
     * @brief Set a specific GPIO as input
     *
     * @param domain GPIO domain (NON_AON, AON, FSI)
     * @param controller GPIO controller within the domain
     * @param port Port number (0-7)
     * @param bit Bit position within port (0-7)
     */
    void set_gpio_mode_in(GPIODomain domain, GPIOController controller, uint8_t port, uint8_t bit);

    /**
     * @brief Set a specific GPIO as output
     *
     * @param domain GPIO domain (NON_AON, AON, FSI)
     * @param controller GPIO controller within the domain
     * @param port Port number (0-7)
     * @param bit Bit position within port (0-7)
     */
    void set_gpio_mode_out(GPIODomain domain, GPIOController controller, uint8_t port, uint8_t bit);

    /**
     * @brief Set a specific GPIO to use an alternative function
     *
     * @param domain GPIO domain (NON_AON, AON, FSI)
     * @param controller GPIO controller within the domain
     * @param port Port number (0-7)
     * @param bit Bit position within port (0-7)
     * @param alternative Alternative function number
     */
    void set_gpio_mode_alt(GPIODomain domain, GPIOController controller, uint8_t port, uint8_t bit, uint8_t alternative);

    /**
     * @brief Set a specific GPIO pin to high state
     *
     * @param domain GPIO domain (NON_AON, AON, FSI)
     * @param controller GPIO controller within the domain
     * @param port Port number (0-7)
     * @param bit Bit position within port (0-7)
     */
    void set_gpio_high(GPIODomain domain, GPIOController controller, uint8_t port, uint8_t bit);

    /**
     * @brief Set a specific GPIO pin to low state
     *
     * @param domain GPIO domain (NON_AON, AON, FSI)
     * @param controller GPIO controller within the domain
     * @param port Port number (0-7)
     * @param bit Bit position within port (0-7)
     */
    void set_gpio_low(GPIODomain domain, GPIOController controller, uint8_t port, uint8_t bit);

    /**
     * @brief Read the current state of a GPIO pin
     *
     * @param domain GPIO domain (NON_AON, AON, FSI)
     * @param controller GPIO controller within the domain
     * @param port Port number (0-7)
     * @param bit Bit position within port (0-7)
     * @return true if pin is high, false if pin is low
     */
    bool get_gpio_logic_state(GPIODomain domain, GPIOController controller, uint8_t port, uint8_t bit);

    /**
     * @brief Get the register address for a specific GPIO port and register type
     *
     * @param domain GPIO domain (NON_AON, AON, FSI)
     * @param controller GPIO controller within the domain
     * @param port Port number (0-7)
     * @param reg_offset Register offset from GPIORegisterOffsets
     * @return uint32_t The full register address
     */
    uint32_t get_gpio_register_address(GPIODomain domain, GPIOController controller, uint8_t port, uint32_t reg_offset) const;

    // We'll use direct memory access to registers rather than a structure
    // since the GPIO registers have variable offsets

    // Array of base addresses for each domain
    volatile uint32_t* _gpio_domain_bases[3]; // NON_AON, AON, FSI

    // Memory range for mapping each domain
    static const uint32_t _gpio_domain_memory_range = 0x10000; // 64-KiB per domain

    // Path to memory device
    static const char* _system_memory_device_path;

    // File descriptor for the memory device file
    int _system_memory_device;

    // Track GPIO output status for each controller
    // Index is calculated as: (domain_index * 10 + controller_index)
    uint32_t _gpio_output_status[30] = {0};

    // Maximum number of GPIO pins
    static const uint8_t JETSON_ORIN_NANO_MAX_PINS = 200; // Based on gpio numbers seen in pinout

    // Pin mapping table - maps physical pin numbers to domain, controller, port, and bit
    // This will need to be populated based on the Jetson Orin Nano documentation
    struct PinMapping {
        GPIODomain domain;        // GPIO domain (NON_AON, AON, FSI)
        GPIOController controller; // GPIO controller within the domain
        uint8_t port;             // Port number (0-7)
        uint8_t bit;              // Bit position within port (0-7)
        bool valid;               // Whether this is a valid GPIO pin
    };

    // Pin mapping table - to be populated in constructor
    PinMapping _pin_mapping[JETSON_ORIN_NANO_MAX_PINS];

    // Initialize the pin mapping table
    void init_pin_mapping();
};

}
