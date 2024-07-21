#include <Arduino.h>

#include "prodrivers.hpp"

#define SET_MODE_PINS(pin, mode) { \
    pinMode(_state.left.pins.pin, (mode)); \
    pinMode(_state.right.pins.pin, (mode)); \
}

#define SET_OUTPUT_PINS(pin, output) { \
    digitalWrite(_state.left.pins.pin, (output)); \
    digitalWrite(_state.right.pins.pin, (output)); \
}

#define SET_SERIAL_FIELDS(field, value) { \
    _state.left.serial.field = (value); \
    _state.right.serial.field = (value); \
}

#define RING_INCREMENT(value, min, max) { \
    if (++(value) > (max)) { (value) = (min); } \
}

#define RING_DECREMENT(value, min, max) { \
    if (--(value) < (min)) { (value) = (max); } \
}

Prodrivers::Prodrivers() {
    // Setup state with defaults
    _state.initialized = false;
    _state.enabled = false;
    SET_SERIAL_FIELDS(phaseA, 1);
    SET_SERIAL_FIELDS(phaseB, 1);
    SET_SERIAL_FIELDS(currentLimA, 1023);
    SET_SERIAL_FIELDS(currentLimB, 1023);
    SET_SERIAL_FIELDS(torque, 0);
    SET_SERIAL_FIELDS(openDetection, 0);
    SET_SERIAL_FIELDS(mixedDecayA, 0);
    SET_SERIAL_FIELDS(mixedDecayB, 0);
    SET_SERIAL_FIELDS(phasePosition, 1);
}

Prodrivers::~Prodrivers() {
    // Disable the motor
    SET_OUTPUT_PINS(enablePin, LOW);
}

bool Prodrivers::setPins(prodrivers_pins_t leftPins, prodrivers_pins_t rightPins) {
    _state.left.pins = leftPins;
    _state.right.pins = rightPins;
    return true;
}

bool Prodrivers::begin() {
    if (_state.initialized) {
        return true;
    }

    pinsSetup();

    // Wait TmodeSU (mode setting setup time) minimum 1 microsecond
    delayMicroseconds(1);

    // Release standby
    SET_OUTPUT_PINS(standbyPin, HIGH);

    // Wait TmodeHO (mode setting Data hold time) minimum 100 microseconds
    delayMicroseconds(100);
    _state.initialized = true;

    return getErrorStatus();
}

bool Prodrivers::setTorque(uint8_t newTorque) {
    SET_SERIAL_FIELDS(torque, newTorque);
    return true;
}

bool Prodrivers::setCurrentLimit(uint16_t newCurrentLimit) {
    if ((newCurrentLimit < 0) || (newCurrentLimit > 1023)) {
        return false;
    }
    SET_SERIAL_FIELDS(currentLimA, newCurrentLimit);
    SET_SERIAL_FIELDS(currentLimB, newCurrentLimit);
    return true;
}

bool Prodrivers::pinsSetup() {
    // Enable is active high
    SET_MODE_PINS(enablePin, OUTPUT);
    SET_OUTPUT_PINS(enablePin, LOW);

    // Standby is active low
    SET_MODE_PINS(standbyPin, OUTPUT);
    SET_OUTPUT_PINS(standbyPin, LOW);

    // Error is active low
    SET_MODE_PINS(errorPin, INPUT);

    // All mode pins to output low
    SET_MODE_PINS(mode0Pin, OUTPUT);
    SET_MODE_PINS(mode1Pin, OUTPUT);
    SET_MODE_PINS(mode2Pin, OUTPUT);
    SET_MODE_PINS(mode3Pin, OUTPUT);
    SET_OUTPUT_PINS(mode0Pin, LOW);
    SET_OUTPUT_PINS(mode1Pin, LOW);
    SET_OUTPUT_PINS(mode2Pin, LOW);
    SET_OUTPUT_PINS(mode3Pin, LOW);

    return getErrorStatus();
}

bool Prodrivers::stepSingle(bool direction) {
    if (!_state.enabled) {
        SET_OUTPUT_PINS(enablePin, HIGH);
        delayMicroseconds(5);
        _state.enabled = true;
    }
    // For a given direction, motors move in opposite directions
    if (direction) {
        RING_INCREMENT(_state.left.serial.phasePosition, 1, 4);
        RING_DECREMENT(_state.right.serial.phasePosition, 1, 4);
    } else {
        RING_DECREMENT(_state.left.serial.phasePosition, 1, 4);
        RING_INCREMENT(_state.right.serial.phasePosition, 1, 4);
    }
    (void)updatePhase(_state.left.serial);
    (void)updatePhase(_state.right.serial);
    return sendSerialCommand();
}

bool Prodrivers::getErrorStatus() {
    return (digitalRead(_state.left.pins.errorPin) && digitalRead(_state.right.pins.errorPin));
}

// Sending serial command takes ~100us
bool Prodrivers::sendSerialCommand() {
    uint32_t commandLeft = buildCommand(_state.left.serial);
    uint32_t commandRight = buildCommand(_state.right.serial);
    for (int i = 0; i < 32; i++) {
        // Clock high
        SET_OUTPUT_PINS(mode2Pin, HIGH);
        delayMicroseconds(1);
        // Set data pins independently
        if (bitRead(commandLeft, i)) {
            digitalWrite(_state.left.pins.mode0Pin, HIGH);
        } else {
            digitalWrite(_state.left.pins.mode0Pin, LOW);
        }
        if (bitRead(commandRight, i)) {
            digitalWrite(_state.right.pins.mode0Pin, HIGH);
        } else {
            digitalWrite(_state.right.pins.mode0Pin, LOW);
        }
        delayMicroseconds(1);
        // Clock low
        SET_OUTPUT_PINS(mode2Pin, LOW);
        delayMicroseconds(1);
    }
    // Latch high
    SET_OUTPUT_PINS(mode1Pin, HIGH);
    delayMicroseconds(1);
    // Latch low
    SET_OUTPUT_PINS(mode1Pin, LOW);
    delayMicroseconds(1);
    return true;
}

uint32_t Prodrivers::buildCommand(const prodrivers_serial_t& serial) {
    uint32_t command = 0;
    // Phase
    command |= ((uint32_t)serial.phaseA << 2);
    command |= ((uint32_t)serial.phaseB << 18);
    // Current limit
    command |= ((uint32_t)serial.currentLimA << 3);
    command |= ((uint32_t)serial.currentLimB << 19);
    // Mixed decay
    command |= ((uint32_t)serial.mixedDecayA << 0);
    command |= ((uint32_t)serial.mixedDecayB << 16);
    // Shared between coils
    command |= ((uint32_t)serial.torque << 29);
    command |= ((uint32_t)serial.openDetection << 31);
    return command;
}

bool Prodrivers::updatePhase(prodrivers_serial_t& serial) {
    switch (serial.phasePosition) {
    case 1:
        serial.phaseA = 1;
        serial.phaseB = 1;
        break;
    case 2:
        serial.phaseA = 0;
        serial.phaseB = 1;
        break;
    case 3:
        serial.phaseA = 0;
        serial.phaseB = 0;
        break;
    case 4:
        serial.phaseA = 1;
        serial.phaseB = 0;
        break;
    default:
        return false;
    }
    return true;
}
