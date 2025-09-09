#pragma once
#include <stdint.h>

/**
 * @brief Packet describing brew/steam state for ESP-NOW transport.
 *
 * This structure is designed to be packed so it can be sent directly
 * over ESP-NOW without additional serialization. All numeric values are
 * little-endian.
 */
struct __attribute__((packed)) EspNowPacket {
    uint8_t shotFlag;        //!< 1 if a shot is in progress
    uint8_t steamFlag;       //!< 1 if the machine is in steam mode
    uint8_t heaterSwitch;    //!< Heater switch state (1=on)
    uint32_t shotTimeMs;     //!< Shot duration in milliseconds
    float shotVolumeMl;      //!< Volume pulled in milliliters
    float setTempC;          //!< Currently configured temperature setpoint
    float currentTempC;      //!< Current sensed temperature in °C
    float pressureBar;       //!< Brew pressure in bar
    float steamSetpointC;    //!< Steam temperature setpoint in °C
    float brewSetpointC;     //!< Brew temperature setpoint in °C
};
