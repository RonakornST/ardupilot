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
    // Jetson Orin Nano GPIO domains based on jetgpio.h
    enum class GPIODomain {
        NON_AON,  // Non-AON domain (pins 7,8,10,11,29,31,36,15,12,32,33,35,38,40,13,16,18,19,21,22,23,24,26,37)
        AON,      // AON domain (pins 3,5,27,28)
        PADCTL_A0, // PADCTL_A0 pad (pins 8,10,11,29,31,36)
        PADCTL_A4, // PADCTL_A4 pad (pins 12,32,33,35,38,40)
        PADCTL_A13, // PADCTL_A13 pad (pins 13,16,18,19,21,22,23,24,26,37)
        PADCTL_A14, // PADCTL_A14 pad (pins 3,5,27,28)
        PADCTL_A16, // PADCTL_A16 pad (pin 15)
        PADCTL_A24  // PADCTL_A24 pad (pin 7)
    };

    // Jetson Orin Nano GPIO controllers based on jetgpio.h
    enum class GPIOController {
        // Non-AON controllers
        G3,  // PADCTL_A0 pad
        G4,  // PADCTL_A4 pad
        G2,  // PADCTL_A13 pad
        G7,  // PADCTL_A24 pad
        EDP, // PADCTL_A16 pad

        // AON controllers
        AO   // PADCTL_A14 pad
    };

    // Domain base addresses from jetgpio.h
    struct GPIODomainBase {
        static const uint32_t NON_AON  = 0x02210000;  // Base address for Non-AON
        static const uint32_t AON      = 0x0c2f1000;  // Base address for AON
        static const uint32_t PADCTL_A0 = 0x02430000; // Pinmux_G3
        static const uint32_t PADCTL_A4 = 0x02434000; // Pinmux_G4
        static const uint32_t PADCTL_A13 = 0x0243d000; // Pinmux_G2
        static const uint32_t PADCTL_A14 = 0x0c302000; // Pinmux_AON
        static const uint32_t PADCTL_A16 = 0x02440000; // Pinmux_EDP
        static const uint32_t PADCTL_A24 = 0x02448000; // Pinmux_G7
    };

    // Register offsets for each pin from jetgpio.h
    struct GPIOPinOffset {
        // CNF registers
        static const uint32_t PIN_3_CNF  = 0x0640; // AO_GEN8_I2C_SDA_0
        static const uint32_t PIN_5_CNF  = 0x0620; // AO_GEN8_I2C_SCL_0
        static const uint32_t PIN_7_CNF  = 0x002c0; // G7_SOC_GPIO59_0
        static const uint32_t PIN_8_CNF  = 0x02840; // G3_UART1_TX_0
        static const uint32_t PIN_10_CNF = 0x02860; // G3_UART1_RX_0
        static const uint32_t PIN_11_CNF = 0x02880; // G3_UART1_RTS_0
        static const uint32_t PIN_12_CNF = 0x042e0; // G4_SOC_GPIO41_0
        static const uint32_t PIN_13_CNF = 0x01200; // G2_SPI3_SCK_0
        static const uint32_t PIN_15_CNF = 0x02220; // EDP_SOC_GPIO39_0
        static const uint32_t PIN_16_CNF = 0x01280; // G2_SPI3_CS1_0
        static const uint32_t PIN_18_CNF = 0x01260; // G2_SPI3_CS0_0
        static const uint32_t PIN_19_CNF = 0x014a0; // G2_SPI1_MOSI_0
        static const uint32_t PIN_21_CNF = 0x01480; // G2_SPI1_MISO_0
        static const uint32_t PIN_22_CNF = 0x01220; // G2_SPI3_MISO_0
        static const uint32_t PIN_23_CNF = 0x01460; // G2_SPI1_SCK_0
        static const uint32_t PIN_24_CNF = 0x014c0; // G2_SPI1_CS0_0
        static const uint32_t PIN_26_CNF = 0x014e0; // G2_SPI1_CS1_0
        static const uint32_t PIN_27_CNF = 0x0600; // AO_GEN2_I2C_SDA_0
        static const uint32_t PIN_28_CNF = 0x04e0; // AO_GEN2_I2C_SCL_0
        static const uint32_t PIN_29_CNF = 0x026a0; // G3_SOC_GPIO32_0
        static const uint32_t PIN_31_CNF = 0x026c0; // G3_SOC_GPIO33_0
        static const uint32_t PIN_32_CNF = 0x040c0; // G4_SOC_GPIO19_0
        static const uint32_t PIN_33_CNF = 0x04200; // G4_SOC_GPIO21_0
        static const uint32_t PIN_35_CNF = 0x04440; // G4_SOC_GPIO44_0
        static const uint32_t PIN_36_CNF = 0x028a0; // G3_UART1_CTS_0
        static const uint32_t PIN_37_CNF = 0x01240; // G2_SPI3_MOSI_0
        static const uint32_t PIN_38_CNF = 0x04420; // G4_SOC_GPIO43_0
        static const uint32_t PIN_40_CNF = 0x04400; // G4_SOC_GPIO42_0
    };

    // Register offsets for GPIO control
    struct GPIORegisterOffsets {
        static const uint32_t CNF = 0x00;      // Configuration register
        static const uint32_t OUT = 0x10;      // Output value register
        static const uint32_t IN  = 0x20;      // Input value register
        static const uint32_t INT_STA = 0x40;  // Interrupt status register
        static const uint32_t INT_ENB = 0x50;  // Interrupt enable register
        static const uint32_t INT_LVL = 0x60;  // Interrupt level register
        static const uint32_t INT_CLR = 0x70;  // Interrupt clear register
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
     * @brief Gets the domain, controller, and register offsets for a pin
     *
     * @param pin GPIO pin number (physical pin)
     * @param domain Output: GPIO domain
     * @param controller Output: GPIO controller
     * @param cnf_offset Output: Configuration register offset
     * @param pinmux_offset Output: Pinmux register offset
     * @return true if conversion successful, false if pin is invalid
     */
    bool pin_to_domain_controller_registers(uint8_t pin, GPIODomain& domain, GPIOController& controller,
                                           uint32_t& cnf_offset, uint32_t& pinmux_offset) const;

    /**
     * @brief Set a specific GPIO as input
     *
     * @param domain GPIO domain
     * @param controller GPIO controller
     * @param cnf_offset Configuration register offset
     * @param pinmux_offset Pinmux register offset
     */
    void set_gpio_mode_in(GPIODomain domain, GPIOController controller, uint32_t cnf_offset, uint32_t pinmux_offset);

    /**
     * @brief Set a specific GPIO as output
     *
     * @param domain GPIO domain
     * @param controller GPIO controller
     * @param cnf_offset Configuration register offset
     * @param pinmux_offset Pinmux register offset
     */
    void set_gpio_mode_out(GPIODomain domain, GPIOController controller, uint32_t cnf_offset, uint32_t pinmux_offset);

    /**
     * @brief Set a specific GPIO to use an alternative function
     *
     * @param domain GPIO domain
     * @param controller GPIO controller
     * @param cnf_offset Configuration register offset
     * @param pinmux_offset Pinmux register offset
     * @param alternative Alternative function number
     */
    void set_gpio_mode_alt(GPIODomain domain, GPIOController controller, uint32_t cnf_offset, uint32_t pinmux_offset, uint8_t alternative);

    /**
     * @brief Set a specific GPIO pin to high state
     *
     * @param domain GPIO domain
     * @param controller GPIO controller
     * @param cnf_offset Configuration register offset
     * @param pinmux_offset Pinmux register offset
     */
    void set_gpio_high(GPIODomain domain, GPIOController controller, uint32_t cnf_offset, uint32_t pinmux_offset);

    /**
     * @brief Set a specific GPIO pin to low state
     *
     * @param domain GPIO domain
     * @param controller GPIO controller
     * @param cnf_offset Configuration register offset
     * @param pinmux_offset Pinmux register offset
     */
    void set_gpio_low(GPIODomain domain, GPIOController controller, uint32_t cnf_offset, uint32_t pinmux_offset);

    /**
     * @brief Read the current state of a GPIO pin
     *
     * @param domain GPIO domain
     * @param controller GPIO controller
     * @param cnf_offset Configuration register offset
     * @param pinmux_offset Pinmux register offset
     * @return true if pin is high, false if pin is low
     */
    bool get_gpio_logic_state(GPIODomain domain, GPIOController controller, uint32_t cnf_offset, uint32_t pinmux_offset);

    /**
     * @brief Get the register address for a specific GPIO controller and register offset
     *
     * @param domain GPIO domain
     * @param controller GPIO controller
     * @param reg_offset Register offset
     * @return uint32_t The full register address
     */
    uint32_t get_gpio_register_address(GPIODomain domain, GPIOController controller, uint32_t reg_offset) const;

    // We'll use direct memory access to registers rather than a structure
    // since the GPIO registers have variable offsets

    // Array of base addresses for each domain
    volatile uint32_t* _gpio_domain_bases[4]; // NON_AON, AON, FSI_0, FSI_1

    // Memory range for mapping each domain
    static const uint32_t _gpio_domain_memory_range = 0x10000; // 64-KiB per domain

    // Path to memory device
    static const char* _system_memory_device_path;

    // File descriptor for the memory device file
    int _system_memory_device;

    // Track GPIO output status for each controller
    // Index is calculated as: (domain_index * 100 + controller_index)
    uint32_t _gpio_output_status[400] = {0};

    // Maximum number of GPIO pins
    static const uint8_t JETSON_ORIN_NANO_MAX_PINS = 200; // Based on gpio numbers seen in pinout

    // Pin mapping table - maps physical pin numbers to domain, controller, and register offset
    // Based on jetgpio.h information
    struct PinMapping {
        GPIODomain domain;        // GPIO domain (NON_AON, AON, PADCTL_*)
        GPIOController controller; // GPIO controller (G3, G4, G2, G7, EDP, AO)
        uint32_t cnf_offset;      // Configuration register offset
        uint32_t pinmux_offset;   // Pinmux register offset
        bool valid;               // Whether this is a valid GPIO pin
    };

    // Pin mapping table - to be populated in constructor
    PinMapping _pin_mapping[JETSON_ORIN_NANO_MAX_PINS];

    // Initialize the pin mapping table
    void init_pin_mapping();
};

}
