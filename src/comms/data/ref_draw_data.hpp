#pragma once

#include "comms/data/comms_data.hpp"

constexpr uint16_t REF_DRAW_RECORD_SIZE = 115;
/// @brief Byte length of one packed figure; equal to ClientGraphic::packet_size
constexpr uint16_t REF_DRAW_FIGURE_SIZE = 15;
/// @brief Byte length of the character text block of a character drawing record
constexpr uint16_t REF_DRAW_TEXT_SIZE = 30;
constexpr uint16_t REF_DRAW_PAYLOAD_SIZE = 105;

/// @brief Contents of the header field of a hive-to-firmware drawing record.
/// @note The values are identical to the Referee System 0x0301 sub-content IDs, so a
///       record header can be forwarded as the content ID directly to
///       RefSystem::write_robot_interaction without translation.
enum class RefDrawHeader : uint16_t {
    /// @brief Delete one or all Player's Client drawing layers; payload is the delete operation byte followed by the layer byte.
    DELETE = 0x0100,
    /// @brief Draw one packed figure; payload is one packed figure, see RefDrawing.hpp
    SHAPE_1 = 0x0101,
    /// @brief Draw two packed figures; payload is two packed figures.
    SHAPE_2 = 0x0102,
    /// @brief Draw five packed figures; payload is five packed figures.
    SHAPE_5 = 0x0103,
    /// @brief Draw seven packed figures; payload is seven packed figures.
    SHAPE_7 = 0x0104,
    /// @brief Draw a character string; payload is one packed figure followed by the character data.
    CHARACTER = 0x0110,
};

/// @brief Give the number of content bytes for a drawing record with the given header.
/// @param header One of the RefDrawHeader values.
/// @return The payload length in bytes: DELETE -> 2, SHAPE_N -> 15 * N, CHARACTER -> 45.
///         An unknown header returns 0
constexpr uint16_t ref_draw_content_length(uint16_t header) {
    switch (header) {
    case static_cast<uint16_t>(RefDrawHeader::DELETE):
        return 2;
    case static_cast<uint16_t>(RefDrawHeader::SHAPE_1):
        return REF_DRAW_FIGURE_SIZE;
    case static_cast<uint16_t>(RefDrawHeader::SHAPE_2):
        return REF_DRAW_FIGURE_SIZE * 2;
    case static_cast<uint16_t>(RefDrawHeader::SHAPE_5):
        return REF_DRAW_FIGURE_SIZE * 5;
    case static_cast<uint16_t>(RefDrawHeader::SHAPE_7):
        return REF_DRAW_FIGURE_SIZE * 7;
    case static_cast<uint16_t>(RefDrawHeader::CHARACTER):
        return REF_DRAW_FIGURE_SIZE + REF_DRAW_TEXT_SIZE;
    default:
        return 0;
    }
}

/// @brief Comms data struct for receiving one Player's Client drawing command from hive
/// @note Wire size is exactly 115 bytes: the 8-byte CommsData base, the 2-byte header,
///       and 105 payload bytes. ARM adds one trailing struct padding byte, which is not
///       transmitted, so only REF_DRAW_RECORD_SIZE bytes should be copied into a record.
/// @note The payload bytes are already packed in the referee wire format, so they are
///       not unpacked or repacked when forwarding to the referee system.
struct RefDrawData : Comms::CommsData {
    /// @brief Default constructor that initializes the CommsData with the correct type label, physical medium, priority, and data size for the RefDrawData struct.
    RefDrawData() : CommsData(Comms::TypeLabel::RefDrawData, Comms::PhysicalMedium::Ethernet, Comms::Priority::Medium, REF_DRAW_RECORD_SIZE) { }
    /// @brief Referee robot-interaction sub-content ID for this record.
    uint16_t header = 0;
    /// @brief Content bytes: packed figures, a delete command, or a character block.
    uint8_t payload[REF_DRAW_PAYLOAD_SIZE] = {0};
};

static_assert(sizeof(Comms::CommsData) == 8, "RefDrawData layout assumes an 8-byte CommsData base");
static_assert(sizeof(RefDrawData) == REF_DRAW_RECORD_SIZE + 1, "RefDrawData is expected to have one trailing ARM padding byte");
static_assert(ref_draw_content_length(static_cast<uint16_t>(RefDrawHeader::SHAPE_7)) == REF_DRAW_PAYLOAD_SIZE, "Largest drawing content must fit the payload");
