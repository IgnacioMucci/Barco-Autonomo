#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <stdint.h>
#include <stdint.h>
#include <math.h>
#include <AP_Math/AP_Math.h>

// Low-level driver for the MKS SERVO42C V1.1 UART protocol.
// This class handles framing/protocol only; Rover's control policy is kept separate.
class AP_MKS_Servo42C {
public:
    static constexpr uint8_t DEFAULT_ADDRESS = 0xE0;

    AP_MKS_Servo42C() = default;

    // Find the UART configured with SERIALx_PROTOCOL = 51 (MKS Servo42C).
    // The serial manager opens/configures the UART using SERIALx_BAUD.
    bool init(AP_SerialManager &serial_manager, uint8_t instance);
    bool is_initialized() const { return _uart != nullptr; }

    // CR_UART enable pin control.
    bool enable();
    bool disable();

    // Constant-speed command: 0..127 gear. reverse=true sets bit 7.
    bool set_speed(uint8_t speed, bool reverse = false);
    bool stop();

    // Position move by a pulse count. pulses is a magnitude; reverse selects direction.
    bool move_pulses(uint32_t pulses, uint8_t speed, bool reverse = false);

     //Pulses per turn = (200 for 1.8° motor, 400 for 0.9°) * MStep. 
    //Must match the actual driver configuration (default 1.8° and MStep 16 = 3200).
    void set_pulses_per_rev(uint32_t pulses_per_rev) { _pulses_per_rev = pulses_per_rev; }

    //RELATIVE MOVEMENT in degrees. Negative sign = reverse.
    bool move_degrees(float delta_degrees, uint8_t speed);

    // Read-only diagnostics.
    bool read_encoder(uint16_t &value);
    bool read_pulses(int32_t &value);
    bool read_shaft_angle(int32_t &value);
    bool read_angle_error(int16_t &value);
    bool read_enable_status(uint8_t &status); // 1=enabled, 2=disabled, 0=error
    bool read_shaft_status(uint8_t &status);  // 1=blocked, 2=unblocked, 0=error

    // Read the driver response from the most recent transaction only through these APIs;
    // callers never need to construct protocol packets manually.

private:
    static constexpr uint16_t RESPONSE_TIMEOUT_MS = 50;
    static constexpr uint8_t MAX_FRAME_SIZE = 8;

    AP_HAL::UARTDriver *_uart = nullptr;
    uint8_t _address = DEFAULT_ADDRESS;
    uint8_t _instance;

    uint32_t _pulses_per_rev = 3200;

    bool command_ack(uint8_t command, const uint8_t *payload, uint8_t payload_length);
    bool read_data(uint8_t command, uint8_t *payload, uint8_t payload_length);
    bool transact(uint8_t command,
                  const uint8_t *payload,
                  uint8_t payload_length,
                  uint8_t *response,
                  uint8_t response_length);

    static uint8_t checksum(const uint8_t *bytes, uint8_t length);
    static uint16_t decode_u16_be(const uint8_t *bytes);
    static uint32_t decode_u32_be(const uint8_t *bytes);
    void thread_main();
};
