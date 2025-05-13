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
    for (int i = 0; i < 4; i++) {
        _gpio_domain_bases[i] = nullptr;
    }

    // Initialize GPIO output status tracking
    for (int i = 0; i < 400; i++) {
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

    // Populate the pin mapping table based on jetgpio.h information

    // Pin 3 - AO_GEN8_I2C_SDA_0 (AON domain)
    _pin_mapping[3].domain = GPIODomain::PADCTL_A14;
    _pin_mapping[3].controller = GPIOController::AO;
    _pin_mapping[3].cnf_offset = GPIOPinOffset::PIN_3_CNF;
    _pin_mapping[3].pinmux_offset = 0x18; // PINMUXO_3
    _pin_mapping[3].valid = true;

    // Pin 5 - AO_GEN8_I2C_SCL_0 (AON domain)
    _pin_mapping[5].domain = GPIODomain::PADCTL_A14;
    _pin_mapping[5].controller = GPIOController::AO;
    _pin_mapping[5].cnf_offset = GPIOPinOffset::PIN_5_CNF;
    _pin_mapping[5].pinmux_offset = 0x20; // PINMUXO_5
    _pin_mapping[5].valid = true;

    // Pin 7 - G7_SOC_GPIO59_0 (PADCTL_A24 domain)
    _pin_mapping[7].domain = GPIODomain::PADCTL_A24;
    _pin_mapping[7].controller = GPIOController::G7;
    _pin_mapping[7].cnf_offset = GPIOPinOffset::PIN_7_CNF;
    _pin_mapping[7].pinmux_offset = 0x30; // PINMUXO_7
    _pin_mapping[7].valid = true;

    // Pin 8 - G3_UART1_TX_0 (PADCTL_A0 domain)
    _pin_mapping[8].domain = GPIODomain::PADCTL_A0;
    _pin_mapping[8].controller = GPIOController::G3;
    _pin_mapping[8].cnf_offset = GPIOPinOffset::PIN_8_CNF;
    _pin_mapping[8].pinmux_offset = 0xa8; // PINMUXO_8
    _pin_mapping[8].valid = true;

    // Pin 10 - G3_UART1_RX_0 (PADCTL_A0 domain)
    _pin_mapping[10].domain = GPIODomain::PADCTL_A0;
    _pin_mapping[10].controller = GPIOController::G3;
    _pin_mapping[10].cnf_offset = GPIOPinOffset::PIN_10_CNF;
    _pin_mapping[10].pinmux_offset = 0xa0; // PINMUXO_10
    _pin_mapping[10].valid = true;

    // Add more pin mappings as needed based on the jetgpio.h information
    // The pattern continues for all the pins (11-40) following the same structure
}

bool GPIO_JETSON_ORIN_NN::pin_to_domain_controller_registers(uint8_t pin, GPIODomain& domain, GPIOController& controller,
                                                   uint32_t& cnf_offset, uint32_t& pinmux_offset) const
{
    if (pin >= JETSON_ORIN_NANO_MAX_PINS || !_pin_mapping[pin].valid) {
        return false;
    }

    domain = _pin_mapping[pin].domain;
    controller = _pin_mapping[pin].controller;
    cnf_offset = _pin_mapping[pin].cnf_offset;
    pinmux_offset = _pin_mapping[pin].pinmux_offset;

    return true;
}

uint32_t GPIO_JETSON_ORIN_NN::get_gpio_register_address(GPIODomain domain, GPIOController controller, uint32_t reg_offset) const
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
        case GPIODomain::PADCTL_A0:
            domain_base = GPIODomainBase::PADCTL_A0;
            break;
        case GPIODomain::PADCTL_A4:
            domain_base = GPIODomainBase::PADCTL_A4;
            break;
        case GPIODomain::PADCTL_A13:
            domain_base = GPIODomainBase::PADCTL_A13;
            break;
        case GPIODomain::PADCTL_A14:
            domain_base = GPIODomainBase::PADCTL_A14;
            break;
        case GPIODomain::PADCTL_A16:
            domain_base = GPIODomainBase::PADCTL_A16;
            break;
        case GPIODomain::PADCTL_A24:
            domain_base = GPIODomainBase::PADCTL_A24;
            break;
        default:
            return 0; // Invalid domain
    }

    // Calculate the full register address
    // For the Jetson Orin Nano, we use the direct register offset from the base address
    return domain_base + reg_offset;
}

void GPIO_JETSON_ORIN_NN::set_gpio_mode_alt(GPIODomain domain, GPIOController controller, uint32_t cnf_offset, uint32_t pinmux_offset, uint8_t alternative)
{
    if (_gpio_domain_bases[static_cast<int>(domain)] == nullptr) {
        return;
    }

    // Calculate the register address for the CNF register
    uint32_t cnf_reg_addr = get_gpio_register_address(domain, controller, cnf_offset);

    // Get the domain base address
    uint32_t domain_base;
    switch (domain) {
        case GPIODomain::NON_AON:
            domain_base = GPIODomainBase::NON_AON;
            break;
        case GPIODomain::AON:
            domain_base = GPIODomainBase::AON;
            break;
        case GPIODomain::PADCTL_A0:
            domain_base = GPIODomainBase::PADCTL_A0;
            break;
        case GPIODomain::PADCTL_A4:
            domain_base = GPIODomainBase::PADCTL_A4;
            break;
        case GPIODomain::PADCTL_A13:
            domain_base = GPIODomainBase::PADCTL_A13;
            break;
        case GPIODomain::PADCTL_A14:
            domain_base = GPIODomainBase::PADCTL_A14;
            break;
        case GPIODomain::PADCTL_A16:
            domain_base = GPIODomainBase::PADCTL_A16;
            break;
        case GPIODomain::PADCTL_A24:
            domain_base = GPIODomainBase::PADCTL_A24;
            break;
        default:
            return; // Invalid domain
    }

    // Calculate the offset from the domain base
    uint32_t cnf_reg_offset = cnf_reg_addr - domain_base;

    // Get a pointer to the CNF register
    volatile uint32_t* cnf_reg_ptr = _gpio_domain_bases[static_cast<int>(domain)] +
                                   (cnf_reg_offset / sizeof(uint32_t));

    // Read current CNF value
    uint32_t cnf_reg_value = *cnf_reg_ptr;

    // Set the configuration for alternative function
    // Based on jetgpio.h, we need to set the CNF register to 0 for GPIO mode
    // and to a specific value for alternative function
    cnf_reg_value = alternative;

    // Write back
    *cnf_reg_ptr = cnf_reg_value;

    // Now handle the pinmux register if needed
    if (pinmux_offset != 0) {
        // Calculate the register address for the PINMUX register
        uint32_t pinmux_reg_addr = get_gpio_register_address(domain, controller, pinmux_offset);

        // Calculate the offset from the domain base
        uint32_t pinmux_reg_offset = pinmux_reg_addr - domain_base;

        // Get a pointer to the PINMUX register
        volatile uint32_t* pinmux_reg_ptr = _gpio_domain_bases[static_cast<int>(domain)] +
                                          (pinmux_reg_offset / sizeof(uint32_t));

        // Set the pinmux value for the alternative function
        // The exact value depends on the specific pin and function
        *pinmux_reg_ptr = alternative;
    }

    // Update our tracking of output state for this pin
    // We'll use the cnf_offset as a unique identifier for the pin
    int controller_index = static_cast<int>(domain) * 100 + static_cast<int>(controller);
    _gpio_output_status[controller_index] |= (1 << (cnf_offset & 0x1F)); // Use lower 5 bits of offset as bit index
}

void GPIO_JETSON_ORIN_NN::set_gpio_mode_in(GPIODomain domain, GPIOController controller, uint32_t cnf_offset, uint32_t pinmux_offset)
{
    if (_gpio_domain_bases[static_cast<int>(domain)] == nullptr) {
        return;
    }

    // Calculate the register address for the CNF register
    uint32_t cnf_reg_addr = get_gpio_register_address(domain, controller, cnf_offset);

    // Get the domain base address
    uint32_t domain_base;
    switch (domain) {
        case GPIODomain::NON_AON:
            domain_base = GPIODomainBase::NON_AON;
            break;
        case GPIODomain::AON:
            domain_base = GPIODomainBase::AON;
            break;
        case GPIODomain::PADCTL_A0:
            domain_base = GPIODomainBase::PADCTL_A0;
            break;
        case GPIODomain::PADCTL_A4:
            domain_base = GPIODomainBase::PADCTL_A4;
            break;
        case GPIODomain::PADCTL_A13:
            domain_base = GPIODomainBase::PADCTL_A13;
            break;
        case GPIODomain::PADCTL_A14:
            domain_base = GPIODomainBase::PADCTL_A14;
            break;
        case GPIODomain::PADCTL_A16:
            domain_base = GPIODomainBase::PADCTL_A16;
            break;
        case GPIODomain::PADCTL_A24:
            domain_base = GPIODomainBase::PADCTL_A24;
            break;
        default:
            return; // Invalid domain
    }

    // Calculate the offset from the domain base
    uint32_t cnf_reg_offset = cnf_reg_addr - domain_base;

    // Get a pointer to the CNF register
    volatile uint32_t* cnf_reg_ptr = _gpio_domain_bases[static_cast<int>(domain)] +
                                   (cnf_reg_offset / sizeof(uint32_t));

    // Based on jetgpio.h, we need to set the CNF register to 0 for GPIO mode
    // and set the direction to input
    *cnf_reg_ptr = 0;

    // Now handle the pinmux register if needed
    if (pinmux_offset != 0) {
        // Calculate the register address for the PINMUX register
        uint32_t pinmux_reg_addr = get_gpio_register_address(domain, controller, pinmux_offset);

        // Calculate the offset from the domain base
        uint32_t pinmux_reg_offset = pinmux_reg_addr - domain_base;

        // Get a pointer to the PINMUX register
        volatile uint32_t* pinmux_reg_ptr = _gpio_domain_bases[static_cast<int>(domain)] +
                                          (pinmux_reg_offset / sizeof(uint32_t));

        // Set the pinmux value for GPIO mode (typically 0)
        *pinmux_reg_ptr = 0;
    }

    // Update our tracking of output state for this pin
    // We'll use the cnf_offset as a unique identifier for the pin
    int controller_index = static_cast<int>(domain) * 100 + static_cast<int>(controller);
    _gpio_output_status[controller_index] &= ~(1 << (cnf_offset & 0x1F)); // Use lower 5 bits of offset as bit index
}

void GPIO_JETSON_ORIN_NN::set_gpio_mode_out(GPIODomain domain, GPIOController controller, uint32_t cnf_offset, uint32_t pinmux_offset)
{
    if (_gpio_domain_bases[static_cast<int>(domain)] == nullptr) {
        return;
    }

    // Calculate the register address for the CNF register
    uint32_t cnf_reg_addr = get_gpio_register_address(domain, controller, cnf_offset);

    // Get the domain base address
    uint32_t domain_base;
    switch (domain) {
        case GPIODomain::NON_AON:
            domain_base = GPIODomainBase::NON_AON;
            break;
        case GPIODomain::AON:
            domain_base = GPIODomainBase::AON;
            break;
        case GPIODomain::PADCTL_A0:
            domain_base = GPIODomainBase::PADCTL_A0;
            break;
        case GPIODomain::PADCTL_A4:
            domain_base = GPIODomainBase::PADCTL_A4;
            break;
        case GPIODomain::PADCTL_A13:
            domain_base = GPIODomainBase::PADCTL_A13;
            break;
        case GPIODomain::PADCTL_A14:
            domain_base = GPIODomainBase::PADCTL_A14;
            break;
        case GPIODomain::PADCTL_A16:
            domain_base = GPIODomainBase::PADCTL_A16;
            break;
        case GPIODomain::PADCTL_A24:
            domain_base = GPIODomainBase::PADCTL_A24;
            break;
        default:
            return; // Invalid domain
    }

    // Calculate the offset from the domain base
    uint32_t cnf_reg_offset = cnf_reg_addr - domain_base;

    // Get a pointer to the CNF register
    volatile uint32_t* cnf_reg_ptr = _gpio_domain_bases[static_cast<int>(domain)] +
                                   (cnf_reg_offset / sizeof(uint32_t));

    // Based on jetgpio.h, we need to set the CNF register to 0 for GPIO mode
    // and set the direction to output (typically by setting bit 0)
    *cnf_reg_ptr = 1; // Set bit 0 for output mode

    // Now handle the pinmux register if needed
    if (pinmux_offset != 0) {
        // Calculate the register address for the PINMUX register
        uint32_t pinmux_reg_addr = get_gpio_register_address(domain, controller, pinmux_offset);

        // Calculate the offset from the domain base
        uint32_t pinmux_reg_offset = pinmux_reg_addr - domain_base;

        // Get a pointer to the PINMUX register
        volatile uint32_t* pinmux_reg_ptr = _gpio_domain_bases[static_cast<int>(domain)] +
                                          (pinmux_reg_offset / sizeof(uint32_t));

        // Set the pinmux value for GPIO mode (typically 0)
        *pinmux_reg_ptr = 0;
    }

    // Update our tracking of output state for this pin
    // We'll use the cnf_offset as a unique identifier for the pin
    int controller_index = static_cast<int>(domain) * 100 + static_cast<int>(controller);
    _gpio_output_status[controller_index] |= (1 << (cnf_offset & 0x1F)); // Use lower 5 bits of offset as bit index
}

void GPIO_JETSON_ORIN_NN::set_gpio_high(GPIODomain domain, GPIOController controller, uint32_t cnf_offset, uint32_t pinmux_offset)
{
    if (_gpio_domain_bases[static_cast<int>(domain)] == nullptr) {
        return;
    }

    // Calculate the register address for the OUT register
    // For Jetson Orin Nano, the OUT register is at offset CNF + 0x10 (based on jetgpio.h)
    uint32_t out_reg_addr = get_gpio_register_address(domain, controller, cnf_offset + GPIORegisterOffsets::OUT);

    // Get the domain base address
    uint32_t domain_base;
    switch (domain) {
        case GPIODomain::NON_AON:
            domain_base = GPIODomainBase::NON_AON;
            break;
        case GPIODomain::AON:
            domain_base = GPIODomainBase::AON;
            break;
        case GPIODomain::PADCTL_A0:
            domain_base = GPIODomainBase::PADCTL_A0;
            break;
        case GPIODomain::PADCTL_A4:
            domain_base = GPIODomainBase::PADCTL_A4;
            break;
        case GPIODomain::PADCTL_A13:
            domain_base = GPIODomainBase::PADCTL_A13;
            break;
        case GPIODomain::PADCTL_A14:
            domain_base = GPIODomainBase::PADCTL_A14;
            break;
        case GPIODomain::PADCTL_A16:
            domain_base = GPIODomainBase::PADCTL_A16;
            break;
        case GPIODomain::PADCTL_A24:
            domain_base = GPIODomainBase::PADCTL_A24;
            break;
        default:
            return; // Invalid domain
    }

    // Calculate the offset from the domain base
    uint32_t out_reg_offset = out_reg_addr - domain_base;

    // Get a pointer to the OUT register
    volatile uint32_t* out_reg_ptr = _gpio_domain_bases[static_cast<int>(domain)] +
                                   (out_reg_offset / sizeof(uint32_t));

    // Set the output value to high (1)
    *out_reg_ptr = 1;

    // Update our tracking of output state for this pin
    // We'll use the cnf_offset as a unique identifier for the pin
    int controller_index = static_cast<int>(domain) * 100 + static_cast<int>(controller);
    _gpio_output_status[controller_index] |= (1 << (cnf_offset & 0x1F)); // Use lower 5 bits of offset as bit index
}

void GPIO_JETSON_ORIN_NN::set_gpio_low(GPIODomain domain, GPIOController controller, uint32_t cnf_offset, uint32_t pinmux_offset)
{
    if (_gpio_domain_bases[static_cast<int>(domain)] == nullptr) {
        return;
    }

    // Calculate the register address for the OUT register
    // For Jetson Orin Nano, the OUT register is at offset CNF + 0x10 (based on jetgpio.h)
    uint32_t out_reg_addr = get_gpio_register_address(domain, controller, cnf_offset + GPIORegisterOffsets::OUT);

    // Get the domain base address
    uint32_t domain_base;
    switch (domain) {
        case GPIODomain::NON_AON:
            domain_base = GPIODomainBase::NON_AON;
            break;
        case GPIODomain::AON:
            domain_base = GPIODomainBase::AON;
            break;
        case GPIODomain::PADCTL_A0:
            domain_base = GPIODomainBase::PADCTL_A0;
            break;
        case GPIODomain::PADCTL_A4:
            domain_base = GPIODomainBase::PADCTL_A4;
            break;
        case GPIODomain::PADCTL_A13:
            domain_base = GPIODomainBase::PADCTL_A13;
            break;
        case GPIODomain::PADCTL_A14:
            domain_base = GPIODomainBase::PADCTL_A14;
            break;
        case GPIODomain::PADCTL_A16:
            domain_base = GPIODomainBase::PADCTL_A16;
            break;
        case GPIODomain::PADCTL_A24:
            domain_base = GPIODomainBase::PADCTL_A24;
            break;
        default:
            return; // Invalid domain
    }

    // Calculate the offset from the domain base
    uint32_t out_reg_offset = out_reg_addr - domain_base;

    // Get a pointer to the OUT register
    volatile uint32_t* out_reg_ptr = _gpio_domain_bases[static_cast<int>(domain)] +
                                   (out_reg_offset / sizeof(uint32_t));

    // Set the output value to low (0)
    *out_reg_ptr = 0;

    // Update our tracking of output state for this pin
    // We'll use the cnf_offset as a unique identifier for the pin
    int controller_index = static_cast<int>(domain) * 100 + static_cast<int>(controller);
    _gpio_output_status[controller_index] &= ~(1 << (cnf_offset & 0x1F)); // Use lower 5 bits of offset as bit index
}

bool GPIO_JETSON_ORIN_NN::get_gpio_logic_state(GPIODomain domain, GPIOController controller, uint32_t cnf_offset, uint32_t pinmux_offset)
{
    if (_gpio_domain_bases[static_cast<int>(domain)] == nullptr) {
        return false;
    }

    // Calculate the register address for the IN register
    // For Jetson Orin Nano, the IN register is at offset CNF + 0x20 (based on jetgpio.h)
    uint32_t in_reg_addr = get_gpio_register_address(domain, controller, cnf_offset + GPIORegisterOffsets::IN);

    // Get the domain base address
    uint32_t domain_base;
    switch (domain) {
        case GPIODomain::NON_AON:
            domain_base = GPIODomainBase::NON_AON;
            break;
        case GPIODomain::AON:
            domain_base = GPIODomainBase::AON;
            break;
        case GPIODomain::PADCTL_A0:
            domain_base = GPIODomainBase::PADCTL_A0;
            break;
        case GPIODomain::PADCTL_A4:
            domain_base = GPIODomainBase::PADCTL_A4;
            break;
        case GPIODomain::PADCTL_A13:
            domain_base = GPIODomainBase::PADCTL_A13;
            break;
        case GPIODomain::PADCTL_A14:
            domain_base = GPIODomainBase::PADCTL_A14;
            break;
        case GPIODomain::PADCTL_A16:
            domain_base = GPIODomainBase::PADCTL_A16;
            break;
        case GPIODomain::PADCTL_A24:
            domain_base = GPIODomainBase::PADCTL_A24;
            break;
        default:
            return false; // Invalid domain
    }

    // Calculate the offset from the domain base
    uint32_t in_reg_offset = in_reg_addr - domain_base;

    // Get a pointer to the IN register
    volatile uint32_t* in_reg_ptr = _gpio_domain_bases[static_cast<int>(domain)] +
                                  (in_reg_offset / sizeof(uint32_t));

    // Read the input value
    // For Jetson Orin Nano, the input value is typically in bit 0
    uint32_t in_reg_value = *in_reg_ptr;

    // Return the state of the input (bit 0)
    return (in_reg_value & 0x1) != 0;
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

    // Map memory for each GPIO domain
    uint32_t domain_addresses[] = {
        GPIODomainBase::NON_AON,
        GPIODomainBase::AON,
        GPIODomainBase::PADCTL_A0,
        GPIODomainBase::PADCTL_A4,
        GPIODomainBase::PADCTL_A13,
        GPIODomainBase::PADCTL_A14,
        GPIODomainBase::PADCTL_A16,
        GPIODomainBase::PADCTL_A24
    };

    for (int i = 0; i < 8; i++) {
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
    uint32_t cnf_offset, pinmux_offset;

    if (!pin_to_domain_controller_registers(pin, domain, controller, cnf_offset, pinmux_offset)) {
        return;
    }

    if (output == HAL_GPIO_INPUT) {
        set_gpio_mode_in(domain, controller, cnf_offset, pinmux_offset);
    } else {
        set_gpio_mode_in(domain, controller, cnf_offset, pinmux_offset);  // First set as input to avoid glitches
        set_gpio_mode_out(domain, controller, cnf_offset, pinmux_offset);  // Then set as output
    }
}

void GPIO_JETSON_ORIN_NN::pinMode(uint8_t pin, uint8_t output, uint8_t alt)
{
    GPIODomain domain;
    GPIOController controller;
    uint32_t cnf_offset, pinmux_offset;

    if (!pin_to_domain_controller_registers(pin, domain, controller, cnf_offset, pinmux_offset)) {
        return;
    }

    if (output == HAL_GPIO_INPUT) {
        set_gpio_mode_in(domain, controller, cnf_offset, pinmux_offset);
    } else if (output == HAL_GPIO_ALT) {
        set_gpio_mode_in(domain, controller, cnf_offset, pinmux_offset);  // First set as input to avoid glitches
        set_gpio_mode_alt(domain, controller, cnf_offset, pinmux_offset, alt);
    } else {
        set_gpio_mode_in(domain, controller, cnf_offset, pinmux_offset);  // First set as input to avoid glitches
        set_gpio_mode_out(domain, controller, cnf_offset, pinmux_offset);  // Then set as output
    }
}

uint8_t GPIO_JETSON_ORIN_NN::read(uint8_t pin)
{
    GPIODomain domain;
    GPIOController controller;
    uint32_t cnf_offset, pinmux_offset;

    if (!pin_to_domain_controller_registers(pin, domain, controller, cnf_offset, pinmux_offset)) {
        return 0;
    }

    return static_cast<uint8_t>(get_gpio_logic_state(domain, controller, cnf_offset, pinmux_offset));
}

void GPIO_JETSON_ORIN_NN::write(uint8_t pin, uint8_t value)
{
    GPIODomain domain;
    GPIOController controller;
    uint32_t cnf_offset, pinmux_offset;

    if (!pin_to_domain_controller_registers(pin, domain, controller, cnf_offset, pinmux_offset)) {
        return;
    }

    if (value != 0) {
        set_gpio_high(domain, controller, cnf_offset, pinmux_offset);
    } else {
        set_gpio_low(domain, controller, cnf_offset, pinmux_offset);
    }
}

void GPIO_JETSON_ORIN_NN::toggle(uint8_t pin)
{
    GPIODomain domain;
    GPIOController controller;
    uint32_t cnf_offset, pinmux_offset;

    if (!pin_to_domain_controller_registers(pin, domain, controller, cnf_offset, pinmux_offset)) {
        return;
    }

    int controller_index = static_cast<int>(domain) * 100 + static_cast<int>(controller);
    uint32_t pin_mask = 1 << (cnf_offset & 0x1F); // Use lower 5 bits of offset as bit index
    _gpio_output_status[controller_index] ^= pin_mask;

    if (_gpio_output_status[controller_index] & pin_mask) {
        set_gpio_high(domain, controller, cnf_offset, pinmux_offset);
    } else {
        set_gpio_low(domain, controller, cnf_offset, pinmux_offset);
    }
}
