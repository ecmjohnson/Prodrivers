/*
 * Driver for ProDriver TC78H670FTG that only supports simultaneous operation of two motors
 */

#pragma once

#include <Arduino.h>

typedef struct {
    // Mode 0 pin differs between left and right motors
    uint8_t mode0Pin;
    // All other pins can be shared
    uint8_t mode1Pin;
    uint8_t mode2Pin;
    uint8_t mode3Pin;
    uint8_t enablePin;
    uint8_t standbyPin;
    // Probably shouldn't be shared...
    uint8_t errorPin;
} prodrivers_pins_t;

typedef struct {
    bool phaseA;
    bool phaseB;
    uint16_t currentLimA; // can be only 10 bits (aka 0-1023) see datasheet pg 20
    uint16_t currentLimB; // can be only 10 bits (aka 0-1023) see datasheet pg 20
    uint8_t torque;
    bool openDetection;
    uint8_t mixedDecayA;
    uint8_t mixedDecayB;
    uint8_t phasePosition; // Used to keep track of phaseA/B to allow single steps in either direction
} prodrivers_serial_t;

typedef struct {
    prodrivers_pins_t pins;
    prodrivers_serial_t serial;
} prodrivers_motor_t;

typedef struct {
    bool initialized;
    bool enabled;
    prodrivers_motor_t left;
    prodrivers_motor_t right;
} prodrivers_state_t;

class Prodrivers {
public:
    Prodrivers();
    ~Prodrivers();

    /* Initialization */
    bool setPins(prodrivers_pins_t leftPins, prodrivers_pins_t rightPins);
    bool setTorque(uint8_t newTorque);
    bool setCurrentLimit(uint16_t newCurrentLimit);
    bool begin();

    /* Control */
    bool stepSingle(bool direction);

private:
    prodrivers_state_t _state {};

    bool pinsSetup();
    bool getErrorStatus();
    bool sendSerialCommand();
    uint32_t buildCommand(const prodrivers_serial_t& serial);
    bool updatePhase(prodrivers_serial_t& serial);
};