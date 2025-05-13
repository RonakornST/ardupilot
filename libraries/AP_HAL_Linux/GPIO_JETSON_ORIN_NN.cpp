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
    _pin_mapping[3].pinmux_offset = PinmuxOffset::PIN_3;
    _pin_mapping[3].valid = true;

    // Pin 5 - AO_GEN8_I2C_SCL_0 (AON domain)
    _pin_mapping[5].domain = GPIODomain::PADCTL_A14;
    _pin_mapping[5].controller = GPIOController::AO;
    _pin_mapping[5].cnf_offset = GPIOPinOffset::PIN_5_CNF;
    _pin_mapping[5].pinmux_offset = PinmuxOffset::PIN_5;
    _pin_mapping[5].valid = true;

    // Pin 7 - G7_SOC_GPIO59_0 (PADCTL_A24 domain)
    _pin_mapping[7].domain = GPIODomain::PADCTL_A24;
    _pin_mapping[7].controller = GPIOController::G7;
    _pin_mapping[7].cnf_offset = GPIOPinOffset::PIN_7_CNF;
    _pin_mapping[7].pinmux_offset = PinmuxOffset::PIN_7;
    _pin_mapping[7].valid = true;

    // Pin 8 - G3_UART1_TX_0 (PADCTL_A0 domain)
    _pin_mapping[8].domain = GPIODomain::PADCTL_A0;
    _pin_mapping[8].controller = GPIOController::G3;
    _pin_mapping[8].cnf_offset = GPIOPinOffset::PIN_8_CNF;
    _pin_mapping[8].pinmux_offset = PinmuxOffset::PIN_8;
    _pin_mapping[8].valid = true;

    // Pin 10 - G3_UART1_RX_0 (PADCTL_A0 domain)
    _pin_mapping[10].domain = GPIODomain::PADCTL_A0;
    _pin_mapping[10].controller = GPIOController::G3;
    _pin_mapping[10].cnf_offset = GPIOPinOffset::PIN_10_CNF;
    _pin_mapping[10].pinmux_offset = PinmuxOffset::PIN_10;
    _pin_mapping[10].valid = true;

    // Pin 11 - G3_UART1_RTS_0 (PADCTL_A0 domain)
    _pin_mapping[11].domain = GPIODomain::PADCTL_A0;
    _pin_mapping[11].controller = GPIOController::G3;
    _pin_mapping[11].cnf_offset = GPIOPinOffset::PIN_11_CNF;
    _pin_mapping[11].pinmux_offset = PinmuxOffset::PIN_11;
    _pin_mapping[11].valid = true;

    // Pin 12 - G4_SOC_GPIO41_0 (PADCTL_A4 domain)
    _pin_mapping[12].domain = GPIODomain::PADCTL_A4;
    _pin_mapping[12].controller = GPIOController::G4;
    _pin_mapping[12].cnf_offset = GPIOPinOffset::PIN_12_CNF;
    _pin_mapping[12].pinmux_offset = PinmuxOffset::PIN_12;
    _pin_mapping[12].valid = true;

    // Pin 13 - G2_SPI3_SCK_0 (PADCTL_A13 domain)
    _pin_mapping[13].domain = GPIODomain::PADCTL_A13;
    _pin_mapping[13].controller = GPIOController::G2;
    _pin_mapping[13].cnf_offset = GPIOPinOffset::PIN_13_CNF;
    _pin_mapping[13].pinmux_offset = PinmuxOffset::PIN_13;
    _pin_mapping[13].valid = true;

    // Pin 15 - EDP_SOC_GPIO39_0 (PADCTL_A16 domain)
    _pin_mapping[15].domain = GPIODomain::PADCTL_A16;
    _pin_mapping[15].controller = GPIOController::EDP;
    _pin_mapping[15].cnf_offset = GPIOPinOffset::PIN_15_CNF;
    _pin_mapping[15].pinmux_offset = PinmuxOffset::PIN_15;
    _pin_mapping[15].valid = true;

    // Pin 16 - G2_SPI3_CS1_0 (PADCTL_A13 domain)
    _pin_mapping[16].domain = GPIODomain::PADCTL_A13;
    _pin_mapping[16].controller = GPIOController::G2;
    _pin_mapping[16].cnf_offset = GPIOPinOffset::PIN_16_CNF;
    _pin_mapping[16].pinmux_offset = PinmuxOffset::PIN_16;
    _pin_mapping[16].valid = true;

    // Pin 18 - G2_SPI3_CS0_0 (PADCTL_A13 domain)
    _pin_mapping[18].domain = GPIODomain::PADCTL_A13;
    _pin_mapping[18].controller = GPIOController::G2;
    _pin_mapping[18].cnf_offset = GPIOPinOffset::PIN_18_CNF;
    _pin_mapping[18].pinmux_offset = PinmuxOffset::PIN_18;
    _pin_mapping[18].valid = true;

    // Pin 19 - G2_SPI1_MOSI_0 (PADCTL_A13 domain)
    _pin_mapping[19].domain = GPIODomain::PADCTL_A13;
    _pin_mapping[19].controller = GPIOController::G2;
    _pin_mapping[19].cnf_offset = GPIOPinOffset::PIN_19_CNF;
    _pin_mapping[19].pinmux_offset = PinmuxOffset::PIN_19;
    _pin_mapping[19].valid = true;
}

bool GPIO_JETSON_ORIN_NN::pin_to_domain_controller_registers(uint8_t pin, GPIODomain& domain, GPIOController& controller,
                                                   uint32_t& cnf_offset, uint32_t& pinmux_offset) const
{
    // Check if the pin is valid
    if (pin >= JETSON_ORIN_NANO_MAX_PINS || !_pin_mapping[pin].valid) {
        return false;
    }

    // Get the domain, controller, and register offsets from the pin mapping table
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
            domain_base = base_CNF_NAON; // 0x02210000
            break;
        case GPIODomain::AON:
            domain_base = base_CNF_AON;  // 0x0c2f1000
            break;
        case GPIODomain::PADCTL_A0:
            domain_base = Pinmux_G3;     // 0x02430000
            break;
        case GPIODomain::PADCTL_A4:
            domain_base = Pinmux_G4;     // 0x02434000
            break;
        case GPIODomain::PADCTL_A13:
            domain_base = Pinmux_G2;     // 0x0243d000
            break;
        case GPIODomain::PADCTL_A14:
            domain_base = Pinmux_AON;    // 0x0c302000
            break;
        case GPIODomain::PADCTL_A16:
            domain_base = Pinmux_EDP;    // 0x02440000
            break;
        case GPIODomain::PADCTL_A24:
            domain_base = Pinmux_G7;     // 0x02448000
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

    // Get the domain base address
    uint32_t domain_base;
    switch (domain) {
        case GPIODomain::NON_AON:
            domain_base = base_CNF_NAON;
            break;
        case GPIODomain::AON:
            domain_base = base_CNF_AON;
            break;
        case GPIODomain::PADCTL_A0:
            domain_base = Pinmux_G3;
            break;
        case GPIODomain::PADCTL_A4:
            domain_base = Pinmux_G4;
            break;
        case GPIODomain::PADCTL_A13:
            domain_base = Pinmux_G2;
            break;
        case GPIODomain::PADCTL_A14:
            domain_base = Pinmux_AON;
            break;
        case GPIODomain::PADCTL_A16:
            domain_base = Pinmux_EDP;
            break;
        case GPIODomain::PADCTL_A24:
            domain_base = Pinmux_G7;
            break;
        default:
            return; // Invalid domain
    }

    // Calculate the register address for the CNF register
    uint32_t cnf_reg_addr = domain_base + cnf_offset;

    // Get a pointer to the GPIO_CNFO structure
    volatile GPIO_CNFO* gpio_cnf = (volatile GPIO_CNFO*)(_gpio_domain_bases[static_cast<int>(domain)] +
                                  (cnf_offset / sizeof(uint32_t)));

    // Set the CNF register to the alternative function value
    gpio_cnf->CNF[0] = alternative;

    // Now handle the pinmux register if needed
    if (pinmux_offset != 0) {
        // Calculate the register address for the PINMUX register
        uint32_t pinmux_reg_addr = domain_base + pinmux_offset;

        // Get a pointer to the PINMUX register
        volatile uint32_t* pinmux_reg_ptr = _gpio_domain_bases[static_cast<int>(domain)] +
                                          (pinmux_offset / sizeof(uint32_t));

        // Set the pinmux value for the alternative function
        // The exact value depends on the specific pin and function
        *pinmux_reg_ptr = alternative;
    }

    // Update our tracking of output status for this pin
    // We'll use the cnf_offset as a unique identifier for the pin
    int controller_index = static_cast<int>(domain) * 100 + static_cast<int>(controller);
    _gpio_output_status[controller_index] |= (1 << (cnf_offset & 0x1F)); // Use lower 5 bits of offset as bit index
}

void GPIO_JETSON_ORIN_NN::set_gpio_mode_in(GPIODomain domain, GPIOController controller, uint32_t cnf_offset, uint32_t pinmux_offset)
{
    if (_gpio_domain_bases[static_cast<int>(domain)] == nullptr) {
        return;
    }

    // Get the domain base address
    uint32_t domain_base;
    switch (domain) {
        case GPIODomain::NON_AON:
            domain_base = base_CNF_NAON;
            break;
        case GPIODomain::AON:
            domain_base = base_CNF_AON;
            break;
        case GPIODomain::PADCTL_A0:
            domain_base = Pinmux_G3;
            break;
        case GPIODomain::PADCTL_A4:
            domain_base = Pinmux_G4;
            break;
        case GPIODomain::PADCTL_A13:
            domain_base = Pinmux_G2;
            break;
        case GPIODomain::PADCTL_A14:
            domain_base = Pinmux_AON;
            break;
        case GPIODomain::PADCTL_A16:
            domain_base = Pinmux_EDP;
            break;
        case GPIODomain::PADCTL_A24:
            domain_base = Pinmux_G7;
            break;
        default:
            return; // Invalid domain
    }

    // Get a pointer to the GPIO_CNFO structure
    volatile GPIO_CNFO* gpio_cnf = (volatile GPIO_CNFO*)(_gpio_domain_bases[static_cast<int>(domain)] +
                                  (cnf_offset / sizeof(uint32_t)));

    // Set the CNF register to 0 for GPIO mode (input)
    gpio_cnf->CNF[0] = CFGO_IN;

    // Now handle the pinmux register if needed
    if (pinmux_offset != 0) {
        // Get a pointer to the PINMUX register
        volatile uint32_t* pinmux_reg_ptr = _gpio_domain_bases[static_cast<int>(domain)] +
                                          (pinmux_offset / sizeof(uint32_t));

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

    // Get the domain base address
    uint32_t domain_base;
    switch (domain) {
        case GPIODomain::NON_AON:
            domain_base = base_CNF_NAON;
            break;
        case GPIODomain::AON:
            domain_base = base_CNF_AON;
            break;
        case GPIODomain::PADCTL_A0:
            domain_base = Pinmux_G3;
            break;
        case GPIODomain::PADCTL_A4:
            domain_base = Pinmux_G4;
            break;
        case GPIODomain::PADCTL_A13:
            domain_base = Pinmux_G2;
            break;
        case GPIODomain::PADCTL_A14:
            domain_base = Pinmux_AON;
            break;
        case GPIODomain::PADCTL_A16:
            domain_base = Pinmux_EDP;
            break;
        case GPIODomain::PADCTL_A24:
            domain_base = Pinmux_G7;
            break;
        default:
            return; // Invalid domain
    }

    // Get a pointer to the GPIO_CNFO structure
    volatile GPIO_CNFO* gpio_cnf = (volatile GPIO_CNFO*)(_gpio_domain_bases[static_cast<int>(domain)] +
                                  (cnf_offset / sizeof(uint32_t)));

    // Set the CNF register to 1 for GPIO output mode
    gpio_cnf->CNF[0] = CFGO_OUT;

    // Now handle the pinmux register if needed
    if (pinmux_offset != 0) {
        // Get a pointer to the PINMUX register
        volatile uint32_t* pinmux_reg_ptr = _gpio_domain_bases[static_cast<int>(domain)] +
                                          (pinmux_offset / sizeof(uint32_t));

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

    // Get the domain base address
    uint32_t domain_base;
    switch (domain) {
        case GPIODomain::NON_AON:
            domain_base = base_CNF_NAON;
            break;
        case GPIODomain::AON:
            domain_base = base_CNF_AON;
            break;
        case GPIODomain::PADCTL_A0:
            domain_base = Pinmux_G3;
            break;
        case GPIODomain::PADCTL_A4:
            domain_base = Pinmux_G4;
            break;
        case GPIODomain::PADCTL_A13:
            domain_base = Pinmux_G2;
            break;
        case GPIODomain::PADCTL_A14:
            domain_base = Pinmux_AON;
            break;
        case GPIODomain::PADCTL_A16:
            domain_base = Pinmux_EDP;
            break;
        case GPIODomain::PADCTL_A24:
            domain_base = Pinmux_G7;
            break;
        default:
            return; // Invalid domain
    }

    // Get a pointer to the GPIO_CNFO structure
    volatile GPIO_CNFO* gpio_cnf = (volatile GPIO_CNFO*)(_gpio_domain_bases[static_cast<int>(domain)] +
                                  (cnf_offset / sizeof(uint32_t)));

    // Set the OUT_VLE register to high (1)
    gpio_cnf->OUT_VLE[0] = 1;

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

    // Get the domain base address
    uint32_t domain_base;
    switch (domain) {
        case GPIODomain::NON_AON:
            domain_base = base_CNF_NAON;
            break;
        case GPIODomain::AON:
            domain_base = base_CNF_AON;
            break;
        case GPIODomain::PADCTL_A0:
            domain_base = Pinmux_G3;
            break;
        case GPIODomain::PADCTL_A4:
            domain_base = Pinmux_G4;
            break;
        case GPIODomain::PADCTL_A13:
            domain_base = Pinmux_G2;
            break;
        case GPIODomain::PADCTL_A14:
            domain_base = Pinmux_AON;
            break;
        case GPIODomain::PADCTL_A16:
            domain_base = Pinmux_EDP;
            break;
        case GPIODomain::PADCTL_A24:
            domain_base = Pinmux_G7;
            break;
        default:
            return; // Invalid domain
    }

    // Get a pointer to the GPIO_CNFO structure
    volatile GPIO_CNFO* gpio_cnf = (volatile GPIO_CNFO*)(_gpio_domain_bases[static_cast<int>(domain)] +
                                  (cnf_offset / sizeof(uint32_t)));

    // Set the OUT_VLE register to low (0)
    gpio_cnf->OUT_VLE[0] = 0;

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

    // Get the domain base address
    uint32_t domain_base;
    switch (domain) {
        case GPIODomain::NON_AON:
            domain_base = base_CNF_NAON;
            break;
        case GPIODomain::AON:
            domain_base = base_CNF_AON;
            break;
        case GPIODomain::PADCTL_A0:
            domain_base = Pinmux_G3;
            break;
        case GPIODomain::PADCTL_A4:
            domain_base = Pinmux_G4;
            break;
        case GPIODomain::PADCTL_A13:
            domain_base = Pinmux_G2;
            break;
        case GPIODomain::PADCTL_A14:
            domain_base = Pinmux_AON;
            break;
        case GPIODomain::PADCTL_A16:
            domain_base = Pinmux_EDP;
            break;
        case GPIODomain::PADCTL_A24:
            domain_base = Pinmux_G7;
            break;
        default:
            return false; // Invalid domain
    }

    // Get a pointer to the GPIO_CNFO structure
    volatile GPIO_CNFO* gpio_cnf = (volatile GPIO_CNFO*)(_gpio_domain_bases[static_cast<int>(domain)] +
                                  (cnf_offset / sizeof(uint32_t)));

    // Read the IN register value
    uint32_t in_reg_value = gpio_cnf->IN[0];

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
        base_CNF_NAON,  // 0x02210000
        base_CNF_AON,   // 0x0c2f1000
        Pinmux_G3,      // 0x02430000
        Pinmux_G4,      // 0x02434000
        Pinmux_G2,      // 0x0243d000
        Pinmux_AON,     // 0x0c302000
        Pinmux_EDP,     // 0x02440000
        Pinmux_G7       // 0x02448000
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
