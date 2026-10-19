#pragma once

#include <cstdint>

/// @brief FlexCAN CAN message struct suitable for host compilation (I stole that shi).
typedef struct CAN_message_t {
    /// @brief CAN identifier.
    uint32_t id = 0;
    /// @brief FlexCAN time when message arrived.
    uint16_t timestamp = 0;
    /// @brief Filter that ID came from.
    uint8_t idhit = 0;
    /// @brief Miscellaneous flags.
    struct {
        /// @brief Identifier is extended (29-bit).
        bool extended = 0;
        /// @brief Remote transmission request packet type.
        bool remote = 0;
        /// @brief Message overrun.
        bool overrun = 0;
        /// @brief No documentation provided by FlexCAN.
        bool reserved = 0;
    } flags;
    /// @brief Length of data.
    uint8_t len = 8;
    /// @brief Data.
    uint8_t buf[8] = {0};
    /// @brief Used to identify mailbox reception.
    int8_t mb = 0;
    /// @brief Used to identify where the message came frum when events() is used.
    uint8_t bus = 0;
    /// @brief Sequential frames.
    bool seq = 0;
} CAN_message_t;