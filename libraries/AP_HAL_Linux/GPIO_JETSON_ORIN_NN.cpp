#include <AP_HAL/AP_HAL.h>

#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "GPIO.h"
#include "GPIO_JETSON_ORIN_NN.h"

using namespace Linux;

extern const AP_HAL::HAL& hal;

// Path to the memory device
const char* GPIO_JETSON_ORIN_NN::_system_memory_device_path = "/dev/mem";

GPIO_JETSON_ORIN_NN::GPIO_JETSON_ORIN_NN()
{
    // Initialize all GPIO domain base pointers to nullptr
    for (int i = 0; i < 3; i++) {
        _gpio_domain_bases[i] = nullptr;
    }

    // Initialize GPIO output status tracking
    for (int i = 0; i < 30; i++) {
        _gpio_output_status[i] = 0;
    }

    // Initialize pin mapping table
    init_pin_mapping();
}

void GPIO_JETSON_ORIN_NN::init_pin_mapping()
{
    // Initialize all pins as invalid
    for (int i = 0; i < JETSON_ORIN_NANO_MAX_PINS; i++) {
        _pin_mapping[i].valid = false;
    }

    // Populate the pin mapping table based on Jetson Orin Nano documentation
    // This is a placeholder and needs to be updated with actual pin mappings
    // from the Jetson Orin Nano Technical Reference Manual

    // Example mappings (these are placeholders and need to be verified):
    // Based on the register information provided, we need to map pins to the correct
    // domain (NON_AON, AON, FSI), controller (CTL0-CTL5, AON, FSI_CTL0-FSI_CTL1),
    // port (0-7), and bit (0-7)

    // Example: Pin 7 -> NON_AON domain, CTL0 controller, Port 0, Bit 4
    _pin_mapping[7].domain = GPIODomain::NON_AON;
    _pin_mapping[7].controller = GPIOController::CTL0;
    _pin_mapping[7].port = 0;
    _pin_mapping[7].bit = 4;
    _pin_mapping[7].valid = true;

    // Example: Pin 11 -> AON domain, AON controller, Port 1, Bit 6
    _pin_mapping[11].domain = GPIODomain::AON;
    _pin_mapping[11].controller = GPIOController::AON;
    _pin_mapping[11].port = 1;
    _pin_mapping[11].bit = 6;
    _pin_mapping[11].valid = true;

    // Example: Pin 15 -> FSI domain, FSI_CTL0 controller, Port 2, Bit 3
    _pin_mapping[15].domain = GPIODomain::FSI;
    _pin_mapping[15].controller = GPIOController::FSI_CTL0;
    _pin_mapping[15].port = 2;
    _pin_mapping[15].bit = 3;
    _pin_mapping[15].valid = true;

    // Add more pin mappings as needed based on the Jetson Orin Nano documentation
}

bool GPIO_JETSON_ORIN_NN::pin_to_domain_controller_port_bit(uint8_t pin, GPIODomain& domain, GPIOController& controller, uint8_t& port, uint8_t& bit) const
{
    if (pin >= JETSON_ORIN_NANO_MAX_PINS || !_pin_mapping[pin].valid) {
        return false;
    }

    domain = _pin_mapping[pin].domain;
    controller = _pin_mapping[pin].controller;
    port = _pin_mapping[pin].port;
    bit = _pin_mapping[pin].bit;

    return true;
}

uint32_t GPIO_JETSON_ORIN_NN::get_gpio_register_address(GPIODomain domain, GPIOController controller, uint8_t port, uint32_t reg_offset) const
{
    // Get the base address for the domain
    uint32_t domain_base;
    switch (domain) {
        case GPIODomain::NON_AON:
            domain_base = GPIODomainBase::NON_AON;
            break;
        case GPIODomain::AON:
            domain_base = GPIODomainBase::AON;
            break;
        case GPIODomain::FSI:
            domain_base = GPIODomainBase::FSI;
            break;
        default:
            return 0; // Invalid domain
    }

    // Calculate the controller offset within the domain
    uint32_t controller_offset;
    switch (domain) {
        case GPIODomain::NON_AON:
            // Controllers CTL0-CTL5
            controller_offset = static_cast<uint32_t>(controller) * CONTROLLER_SIZE;
            break;
        case GPIODomain::AON:
            // Only one controller (AON)
            controller_offset = 0;
            break;
        case GPIODomain::FSI:
            // Controllers FSI_CTL0-FSI_CTL1
            controller_offset = (static_cast<uint32_t>(controller) - static_cast<uint32_t>(GPIOController::FSI_CTL0)) * CONTROLLER_SIZE;
            break;
        default:
            return 0; // Invalid domain
    }

    // Calculate the port offset within the controller
    uint32_t port_offset = port * PORT_SIZE;

    // Calculate the full register address
    return domain_base + controller_offset + port_offset + reg_offset;
}

void GPIO_JETSON_ORIN_NN::set_gpio_mode_alt(GPIODomain domain, GPIOController controller, uint8_t port, uint8_t bit, uint8_t alternative)
{
    if (port >= 8 || _gpio_domain_bases[static_cast<int>(domain)] == nullptr) {
        return;
    }

    // Calculate the register address for the ENABLE_CONFIG register
    uint32_t reg_addr = get_gpio_register_address(domain, controller, port, GPIORegisterOffsets::ENABLE_CONFIG);
    uint32_t domain_base;
    switch (domain) {
        case GPIODomain::NON_AON:
            domain_base = GPIODomainBase::NON_AON;
            break;
        case GPIODomain::AON:
            domain_base = GPIODomainBase::AON;
            break;
        case GPIODomain::FSI:
            domain_base = GPIODomainBase::FSI;
            break;
        default:
            return; // Invalid domain
    }

    // Calculate the offset from the domain base
    uint32_t reg_addr_offset = reg_addr - domain_base;

    // Get a pointer to the register
    volatile uint32_t* reg_ptr = _gpio_domain_bases[static_cast<int>(domain)] +
                                (reg_addr_offset / sizeof(uint32_t));

    // Read current value
    uint32_t reg_value = *reg_ptr;

    // Set the configuration for alternative function
    // The exact bit pattern will depend on the Jetson Orin Nano GPIO controller
    // For now, we'll assume setting bit 0 enables the alternative function
    reg_value |= (1 << bit);

    // Write back
    *reg_ptr = reg_value;

    // Update our tracking of output state
    int controller_index = static_cast<int>(domain) * 10 + static_cast<int>(controller);
    _gpio_output_status[controller_index] |= (1 << ((port * 8) + bit));
}

void GPIO_JETSON_ORIN_NN::set_gpio_mode_in(GPIODomain domain, GPIOController controller, uint8_t port, uint8_t bit)
{
    if (port >= 8 || _gpio_domain_bases[static_cast<int>(domain)] == nullptr) {
        return;
    }

    // Calculate the register address for the ENABLE_CONFIG register (to disable output)
    uint32_t reg_addr = get_gpio_register_address(domain, controller, port, GPIORegisterOffsets::ENABLE_CONFIG);
    uint32_t domain_base;
    switch (domain) {
        case GPIODomain::NON_AON:
            domain_base = GPIODomainBase::NON_AON;
            break;
        case GPIODomain::AON:
            domain_base = GPIODomainBase::AON;
            break;
        case GPIODomain::FSI:
            domain_base = GPIODomainBase::FSI;
            break;
        default:
            return; // Invalid domain
    }

    // Calculate the offset from the domain base
    uint32_t reg_addr_offset = reg_addr - domain_base;

    // Get a pointer to the register
    volatile uint32_t* reg_ptr = _gpio_domain_bases[static_cast<int>(domain)] +
                                (reg_addr_offset / sizeof(uint32_t));

    // Read current value
    uint32_t reg_value = *reg_ptr;

    // Clear the output enable bit for this pin (set to input mode)
    // Based on the Orin TRM, bit 0 of GPIO_OUT_VAL is effective when GPIO_ENABLE is ENABLE and IN_OUT is OUT
    // So we need to set IN_OUT to IN (assuming bit 1 controls this)
    reg_value &= ~(1 << bit);

    // Write back
    *reg_ptr = reg_value;

    // Update our tracking of output state
    int controller_index = static_cast<int>(domain) * 10 + static_cast<int>(controller);
    _gpio_output_status[controller_index] &= ~(1 << ((port * 8) + bit));
}

void GPIO_JETSON_ORIN_NN::set_gpio_mode_out(GPIODomain domain, GPIOController controller, uint8_t port, uint8_t bit)
{
    if (port >= 8 || _gpio_domain_bases[static_cast<int>(domain)] == nullptr) {
        return;
    }

    // Calculate the register address for the ENABLE_CONFIG register (to enable output)
    uint32_t reg_addr = get_gpio_register_address(domain, controller, port, GPIORegisterOffsets::ENABLE_CONFIG);
    uint32_t domain_base;
    switch (domain) {
        case GPIODomain::NON_AON:
            domain_base = GPIODomainBase::NON_AON;
            break;
        case GPIODomain::AON:
            domain_base = GPIODomainBase::AON;
            break;
        case GPIODomain::FSI:
            domain_base = GPIODomainBase::FSI;
            break;
        default:
            return; // Invalid domain
    }

    // Calculate the offset from the domain base
    uint32_t reg_addr_offset = reg_addr - domain_base;

    // Get a pointer to the register
    volatile uint32_t* reg_ptr = _gpio_domain_bases[static_cast<int>(domain)] +
                                (reg_addr_offset / sizeof(uint32_t));

    // Read current value
    uint32_t reg_value = *reg_ptr;

    // Set the output enable bit for this pin
    // Based on the Orin TRM, bit 0 of GPIO_OUT_VAL is effective when GPIO_ENABLE is ENABLE and IN_OUT is OUT
    // So we need to set IN_OUT to OUT (assuming bit 1 controls this)
    reg_value |= (1 << bit);

    // Write back
    *reg_ptr = reg_value;

    // Update our tracking of output state
    int controller_index = static_cast<int>(domain) * 10 + static_cast<int>(controller);
    _gpio_output_status[controller_index] |= (1 << ((port * 8) + bit));
}

void GPIO_JETSON_ORIN_NN::set_gpio_high(GPIODomain domain, GPIOController controller, uint8_t port, uint8_t bit)
{
    if (port >= 8 || _gpio_domain_bases[static_cast<int>(domain)] == nullptr) {
        return;
    }

    // Calculate the register address for the OUTPUT_VALUE register
    uint32_t reg_addr = get_gpio_register_address(domain, controller, port, GPIORegisterOffsets::OUTPUT_VALUE);
    uint32_t domain_base;
    switch (domain) {
        case GPIODomain::NON_AON:
            domain_base = GPIODomainBase::NON_AON;
            break;
        case GPIODomain::AON:
            domain_base = GPIODomainBase::AON;
            break;
        case GPIODomain::FSI:
            domain_base = GPIODomainBase::FSI;
            break;
        default:
            return; // Invalid domain
    }

    // Calculate the offset from the domain base
    uint32_t reg_addr_offset = reg_addr - domain_base;

    // Get a pointer to the register
    volatile uint32_t* reg_ptr = _gpio_domain_bases[static_cast<int>(domain)] +
                                (reg_addr_offset / sizeof(uint32_t));

    // Read current value
    uint32_t reg_value = *reg_ptr;

    // Set the bit for this pin to high
    reg_value |= (1 << bit);

    // Write back
    *reg_ptr = reg_value;

    // Update our tracking of output state
    int controller_index = static_cast<int>(domain) * 10 + static_cast<int>(controller);
    _gpio_output_status[controller_index] |= (1 << ((port * 8) + bit));
}

void GPIO_JETSON_ORIN_NN::set_gpio_low(GPIODomain domain, GPIOController controller, uint8_t port, uint8_t bit)
{
    if (port >= 8 || _gpio_domain_bases[static_cast<int>(domain)] == nullptr) {
        return;
    }

    // Calculate the register address for the OUTPUT_VALUE register
    uint32_t reg_addr = get_gpio_register_address(domain, controller, port, GPIORegisterOffsets::OUTPUT_VALUE);
    uint32_t domain_base;
    switch (domain) {
        case GPIODomain::NON_AON:
            domain_base = GPIODomainBase::NON_AON;
            break;
        case GPIODomain::AON:
            domain_base = GPIODomainBase::AON;
            break;
        case GPIODomain::FSI:
            domain_base = GPIODomainBase::FSI;
            break;
        default:
            return; // Invalid domain
    }

    // Calculate the offset from the domain base
    uint32_t reg_addr_offset = reg_addr - domain_base;

    // Get a pointer to the register
    volatile uint32_t* reg_ptr = _gpio_domain_bases[static_cast<int>(domain)] +
                                (reg_addr_offset / sizeof(uint32_t));

    // Read current value
    uint32_t reg_value = *reg_ptr;

    // Clear the bit for this pin to set it low
    reg_value &= ~(1 << bit);

    // Write back
    *reg_ptr = reg_value;

    // Update our tracking of output state
    int controller_index = static_cast<int>(domain) * 10 + static_cast<int>(controller);
    _gpio_output_status[controller_index] &= ~(1 << ((port * 8) + bit));
}

bool GPIO_JETSON_ORIN_NN::get_gpio_logic_state(GPIODomain domain, GPIOController controller, uint8_t port, uint8_t bit)
{
    if (port >= 8 || _gpio_domain_bases[static_cast<int>(domain)] == nullptr) {
        return false;
    }

    // Calculate the register address for the INPUT_VALUE register
    uint32_t reg_addr = get_gpio_register_address(domain, controller, port, GPIORegisterOffsets::INPUT_VALUE);
    uint32_t domain_base;
    switch (domain) {
        case GPIODomain::NON_AON:
            domain_base = GPIODomainBase::NON_AON;
            break;
        case GPIODomain::AON:
            domain_base = GPIODomainBase::AON;
            break;
        case GPIODomain::FSI:
            domain_base = GPIODomainBase::FSI;
            break;
        default:
            return false; // Invalid domain
    }

    // Calculate the offset from the domain base
    uint32_t reg_addr_offset = reg_addr - domain_base;

    // Get a pointer to the register
    volatile uint32_t* reg_ptr = _gpio_domain_bases[static_cast<int>(domain)] +
                                (reg_addr_offset / sizeof(uint32_t));

    // Read the register value
    uint32_t reg_value = *reg_ptr;

    // Return the state of the specific bit
    return (reg_value & (1 << bit)) != 0;
}

volatile uint32_t* GPIO_JETSON_ORIN_NN::get_memory_pointer(uint32_t address, uint32_t range) const
{
    auto pointer = mmap(
        nullptr,                         // Any address in our space will do
        range,                           // Map length
        PROT_READ|PROT_WRITE|PROT_EXEC,  // Enable reading & writing to mapped memory
        MAP_SHARED|MAP_LOCKED,           // Shared with other processes
        _system_memory_device,           // File to map
        address                          // Offset to GPIO peripheral
    );

    if (pointer == MAP_FAILED) {
        return nullptr;
    }

    return static_cast<volatile uint32_t*>(pointer);
}

bool GPIO_JETSON_ORIN_NN::openMemoryDevice()
{
    _system_memory_device = open(_system_memory_device_path, O_RDWR|O_SYNC|O_CLOEXEC);
    if (_system_memory_device < 0) {
        AP_HAL::panic("Can't open %s", GPIO_JETSON_ORIN_NN::_system_memory_device_path);
        return false;
    }

    return true;
}

void GPIO_JETSON_ORIN_NN::closeMemoryDevice()
{
    close(_system_memory_device);
    // Invalidate device variable
    _system_memory_device = -1;
}

void GPIO_JETSON_ORIN_NN::init()
{
    if (!openMemoryDevice()) {
        AP_HAL::panic("Failed to initialize memory device.");
        return;
    }

    // Map memory for each GPIO domain (NON_AON, AON, FSI)
    uint32_t domain_addresses[] = {
        GPIODomainBase::NON_AON,
        GPIODomainBase::AON,
        GPIODomainBase::FSI
    };

    for (int i = 0; i < 3; i++) {
        uint32_t domain_address = domain_addresses[i];

        // Map the entire 64-KiB domain space
        void* mapped_ptr = mmap(
            nullptr,                         // Any address in our space will do
            _gpio_domain_memory_range,       // Map length (64-KiB)
            PROT_READ|PROT_WRITE|PROT_EXEC,  // Enable reading & writing to mapped memory
            MAP_SHARED|MAP_LOCKED,           // Shared with other processes
            _system_memory_device,           // File to map
            domain_address                   // Offset to GPIO domain
        );

        if (mapped_ptr == MAP_FAILED) {
            AP_HAL::panic("Failed to map GPIO domain %d", i);
        } else {
            _gpio_domain_bases[i] = static_cast<volatile uint32_t*>(mapped_ptr);
        }
    }

    // No need to keep mem_fd open after mmap
    closeMemoryDevice();
}

void GPIO_JETSON_ORIN_NN::pinMode(uint8_t pin, uint8_t output)
{
    GPIODomain domain;
    GPIOController controller;
    uint8_t port, bit;

    if (!pin_to_domain_controller_port_bit(pin, domain, controller, port, bit)) {
        return;
    }

    if (output == HAL_GPIO_INPUT) {
        set_gpio_mode_in(domain, controller, port, bit);
    } else {
        set_gpio_mode_in(domain, controller, port, bit);  // First set as input to avoid glitches
        set_gpio_mode_out(domain, controller, port, bit);  // Then set as output
    }
}

void GPIO_JETSON_ORIN_NN::pinMode(uint8_t pin, uint8_t output, uint8_t alt)
{
    GPIODomain domain;
    GPIOController controller;
    uint8_t port, bit;

    if (!pin_to_domain_controller_port_bit(pin, domain, controller, port, bit)) {
        return;
    }

    if (output == HAL_GPIO_INPUT) {
        set_gpio_mode_in(domain, controller, port, bit);
    } else if (output == HAL_GPIO_ALT) {
        set_gpio_mode_in(domain, controller, port, bit);  // First set as input to avoid glitches
        set_gpio_mode_alt(domain, controller, port, bit, alt);
    } else {
        set_gpio_mode_in(domain, controller, port, bit);  // First set as input to avoid glitches
        set_gpio_mode_out(domain, controller, port, bit);  // Then set as output
    }
}

uint8_t GPIO_JETSON_ORIN_NN::read(uint8_t pin)
{
    GPIODomain domain;
    GPIOController controller;
    uint8_t port, bit;

    if (!pin_to_domain_controller_port_bit(pin, domain, controller, port, bit)) {
        return 0;
    }

    return static_cast<uint8_t>(get_gpio_logic_state(domain, controller, port, bit));
}

void GPIO_JETSON_ORIN_NN::write(uint8_t pin, uint8_t value)
{
    GPIODomain domain;
    GPIOController controller;
    uint8_t port, bit;

    if (!pin_to_domain_controller_port_bit(pin, domain, controller, port, bit)) {
        return;
    }

    if (value != 0) {
        set_gpio_high(domain, controller, port, bit);
    } else {
        set_gpio_low(domain, controller, port, bit);
    }
}

void GPIO_JETSON_ORIN_NN::toggle(uint8_t pin)
{
    GPIODomain domain;
    GPIOController controller;
    uint8_t port, bit;

    if (!pin_to_domain_controller_port_bit(pin, domain, controller, port, bit)) {
        return;
    }

    int controller_index = static_cast<int>(domain) * 10 + static_cast<int>(controller);
    uint32_t pin_mask = 1 << ((port * 8) + bit);
    _gpio_output_status[controller_index] ^= pin_mask;

    if (_gpio_output_status[controller_index] & pin_mask) {
        set_gpio_high(domain, controller, port, bit);
    } else {
        set_gpio_low(domain, controller, port, bit);
    }
}
