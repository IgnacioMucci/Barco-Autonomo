#include "AP_MKS_Servo42C.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <stdint.h>


extern const AP_HAL::HAL &hal;

bool AP_MKS_Servo42C::init(AP_SerialManager &serial_manager, uint8_t instance)
{
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_MKS_Servo42C, instance);
    if (_uart == nullptr) {
        return false;
    }
    _instance = instance;

    char name[16];
    hal.util->snprintf(name, sizeof(name), "mks42c%u", (unsigned)instance);
    return hal.scheduler->thread_create(
        FUNCTOR_BIND_MEMBER(&AP_MKS_Servo42C::thread_main, void),
        name, 2048, AP_HAL::Scheduler::PRIORITY_IO, 0);
}

void AP_MKS_Servo42C::thread_main()
{
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay(100);
    }
    while (true) {
        hal.scheduler->delay(1000);

        // --- PRUEBA TEMPORAL: leer el encoder una vez por segundo ---
        uint16_t enc;
        if (read_encoder(enc)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MKS%u enc=%u", (unsigned)_instance, (unsigned)enc);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "MKS%u sin respuesta", (unsigned)_instance);
        }
    }
}

uint8_t AP_MKS_Servo42C::checksum(const uint8_t *bytes, uint8_t length)
{
    // MKS checksum: sum all bytes before the checksum and keep the low 8 bits.
    uint16_t sum = 0;
    for (uint8_t i = 0; i < length; i++) {
        sum += bytes[i];
    }
    return static_cast<uint8_t>(sum & 0xFFU);
}

uint16_t AP_MKS_Servo42C::decode_u16_be(const uint8_t *bytes)
{
    return (static_cast<uint16_t>(bytes[0]) << 8) |
           static_cast<uint16_t>(bytes[1]);
}

uint32_t AP_MKS_Servo42C::decode_u32_be(const uint8_t *bytes)
{
    return (static_cast<uint32_t>(bytes[0]) << 24) |
           (static_cast<uint32_t>(bytes[1]) << 16) |
           (static_cast<uint32_t>(bytes[2]) << 8) |
           static_cast<uint32_t>(bytes[3]);
}

bool AP_MKS_Servo42C::transact(uint8_t command,
                               const uint8_t *payload,
                               uint8_t payload_length,
                               uint8_t *response,
                               uint8_t response_length)
{
    if (_uart == nullptr || response == nullptr || response_length < 3) {
        return false;
    }

    const uint8_t request_length = payload_length + 3; // address + command + data + checksum
    if (request_length > MAX_FRAME_SIZE) {
        return false;
    }

    uint8_t request[MAX_FRAME_SIZE] = {};
    request[0] = _address;
    request[1] = command;
    for (uint8_t i = 0; i < payload_length; i++) {
        if (payload == nullptr) {
            return false;
        }
        request[2 + i] = payload[i];
    }
    request[request_length - 1] = checksum(request, request_length - 1);

    // A leftover response from a previous timed-out transaction must not be
    // mistaken for this command's reply.
    _uart->discard_input();

    if (_uart->write(request, request_length) != request_length) {
        return false;
    }

    if (!_uart->wait_timeout(response_length, RESPONSE_TIMEOUT_MS)) {
        return false;
    }

    for (uint8_t i = 0; i < response_length; i++) {
        const int16_t byte_read = _uart->read();
        if (byte_read < 0) {
            return false;
        }
        response[i] = static_cast<uint8_t>(byte_read);
    }

    if (response[0] != _address) {
        return false;
    }
    if (checksum(response, response_length - 1) != response[response_length - 1]) {
        return false;
    }

    return true;
}

bool AP_MKS_Servo42C::command_ack(uint8_t command,
                                  const uint8_t *payload,
                                  uint8_t payload_length)
{
    uint8_t response[3] = {};
    if (!transact(command, payload, payload_length, response, sizeof(response))) {
        return false;
    }
    // Successful command reply: address, 0x01, checksum.
    return response[1] == 0x01;
}

bool AP_MKS_Servo42C::read_data(uint8_t command, uint8_t *payload, uint8_t payload_length)
{
    if (payload == nullptr || payload_length > 6) {
        return false;
    }

    uint8_t response[8] = {};
    const uint8_t response_length = payload_length + 2; // address + value + checksum
    if (!transact(command, nullptr, 0, response, response_length)) {
        return false;
    }

    for (uint8_t i = 0; i < payload_length; i++) {
        payload[i] = response[i + 1];
    }
    return true;
}

bool AP_MKS_Servo42C::enable()
{
    const uint8_t value = 0x01;
    return command_ack(0xF3, &value, 1);
}

bool AP_MKS_Servo42C::disable()
{
    const uint8_t value = 0x00;
    return command_ack(0xF3, &value, 1);
}

bool AP_MKS_Servo42C::set_speed(uint8_t speed, bool reverse)
{
    if (speed > 127) {
        return false;
    }
    const uint8_t direction_speed = static_cast<uint8_t>((reverse ? 0x80U : 0x00U) | speed);
    return command_ack(0xF6, &direction_speed, 1);
}

bool AP_MKS_Servo42C::stop()
{
    // The F7 command has no data byte.
    return command_ack(0xF7, nullptr, 0);
}

bool AP_MKS_Servo42C::move_pulses(uint32_t pulses, uint8_t speed, bool reverse)
{
    if (speed > 127) {
        return false;
    }

    const uint8_t payload[5] = {
        static_cast<uint8_t>((reverse ? 0x80U : 0x00U) | speed),
        static_cast<uint8_t>((pulses >> 24) & 0xFFU),
        static_cast<uint8_t>((pulses >> 16) & 0xFFU),
        static_cast<uint8_t>((pulses >> 8) & 0xFFU),
        static_cast<uint8_t>(pulses & 0xFFU),
    };
    return command_ack(0xFD, payload, sizeof(payload));
}

bool AP_MKS_Servo42C::read_encoder(uint16_t &value)
{
    uint8_t data[6] = {};
    if (!read_data(0x30, data, sizeof(data))) {
        return false;
    }
    value = decode_u16_be(&data[4]);
    return true;
}

bool AP_MKS_Servo42C::read_pulses(int32_t &value)
{
    uint8_t data[4] = {};
    if (!read_data(0x33, data, sizeof(data))) {
        return false;
    }

    const uint32_t raw = decode_u32_be(data);
    const int64_t signed_value = (raw <= static_cast<uint32_t>(INT32_MAX))
        ? static_cast<int64_t>(raw)
        : static_cast<int64_t>(raw) - 0x100000000LL;
    value = static_cast<int32_t>(signed_value);
    return true;
}

bool AP_MKS_Servo42C::read_shaft_angle(int32_t &value)
{
    uint8_t data[4] = {};
    if (!read_data(0x36, data, sizeof(data))) {
        return false;
    }

    const uint32_t raw = decode_u32_be(data);
    const int64_t signed_value = (raw <= static_cast<uint32_t>(INT32_MAX))
        ? static_cast<int64_t>(raw)
        : static_cast<int64_t>(raw) - 0x100000000LL;
    value = static_cast<int32_t>(signed_value);
    return true;
}
bool AP_MKS_Servo42C::read_angle_error(int16_t &value)
{
    uint8_t data[2] = {};
    if (!read_data(0x39, data, sizeof(data))) {
        return false;
    }

    const uint16_t raw = decode_u16_be(data);
    const int32_t signed_value = (raw <= static_cast<uint16_t>(INT16_MAX))
        ? static_cast<int32_t>(raw)
        : static_cast<int32_t>(raw) - 0x10000;
    value = static_cast<int16_t>(signed_value);
    return true;
}

bool AP_MKS_Servo42C::read_enable_status(uint8_t &status)
{
    return read_data(0x3A, &status, 1);
}

bool AP_MKS_Servo42C::read_shaft_status(uint8_t &status)
{
    return read_data(0x3E, &status, 1);
}

bool AP_MKS_Servo42C::move_degrees(float delta_degrees, uint8_t speed)
{
    if (isnan(delta_degrees) || isinf(delta_degrees)) {
        return false;
    }
    const bool reverse = delta_degrees < 0.0f;
    const float pulses_f = roundf(fabsf(delta_degrees) / 360.0f * static_cast<float>(_pulses_per_rev));
    if (pulses_f > 4294967000.0f) {
        return false;
    }

    const uint32_t pulses = static_cast<uint32_t>(pulses_f);
    if (pulses == 0) {
        return true; // movimiento menor a un pulso: nada que hacer
    }
    return move_pulses(pulses, speed, reverse);
}