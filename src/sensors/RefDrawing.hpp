#pragma once

#include "RefSystem.hpp"

enum class ClientGraphicOperation : uint8_t {
    NONE = 0,
    ADD = 1,
    EDIT = 2,
    DELETE = 3,
};

enum class ClientGraphicType : uint8_t {
    LINE = 0,
    RECTANGLE = 1,
    CIRCLE = 2,
    ELLIPSE = 3,
    ARC = 4,
    FLOAT = 5,
    INTEGER = 6,
    CHARACTER = 7,
};

enum class ClientGraphicColor : uint8_t {
    TEAM = 0,
    YELLOW = 1,
    GREEN = 2,
    ORANGE = 3,
    MAGENTA = 4,
    PINK = 5,
    CYAN = 6,
    BLACK = 7,
    WHITE = 8,
};

enum class ClientLayerDeleteOperation : uint8_t {
    NONE = 0,
    DELETE_LAYER = 1,
    DELETE_ALL = 2,
};

/// @brief Packed figure description used by player-client drawing packets.
struct ClientGraphic {
    static constexpr uint8_t packet_size = 15;

    uint8_t name[3] = {0};
    ClientGraphicOperation operation = ClientGraphicOperation::ADD;
    ClientGraphicType type = ClientGraphicType::LINE;
    uint8_t layer = 0;
    ClientGraphicColor color = ClientGraphicColor::TEAM;
    uint16_t details_a = 0;
    uint16_t details_b = 0;
    uint16_t width = 1;
    uint16_t start_x = 0;
    uint16_t start_y = 0;
    uint16_t details_c = 0;
    uint16_t details_d = 0;
    uint16_t details_e = 0;
};

class RefDrawing {
  public:
    explicit RefDrawing(RefSystem &ref_system);

    bool delete_layer(uint8_t layer, uint16_t receiver_id = 0);
    bool delete_all_layers(uint16_t receiver_id = 0);
    bool draw_graphic(const ClientGraphic &graphic, uint16_t receiver_id = 0);
    bool draw_graphics(const ClientGraphic *graphics, uint8_t graphic_count, uint16_t receiver_id = 0);
    bool draw_line(const char *name, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y, uint16_t width = 2, uint8_t layer = 0, ClientGraphicColor color = ClientGraphicColor::TEAM, ClientGraphicOperation operation = ClientGraphicOperation::ADD, uint16_t receiver_id = 0);
    bool draw_rectangle(const char *name, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y, uint16_t width = 2, uint8_t layer = 0, ClientGraphicColor color = ClientGraphicColor::TEAM, ClientGraphicOperation operation = ClientGraphicOperation::ADD, uint16_t receiver_id = 0);
    bool draw_circle(const char *name, uint16_t center_x, uint16_t center_y, uint16_t radius, uint16_t width = 2, uint8_t layer = 0, ClientGraphicColor color = ClientGraphicColor::TEAM, ClientGraphicOperation operation = ClientGraphicOperation::ADD, uint16_t receiver_id = 0);
    bool draw_ellipse(const char *name, uint16_t center_x, uint16_t center_y, uint16_t x_semi_axis, uint16_t y_semi_axis, uint16_t width = 2, uint8_t layer = 0, ClientGraphicColor color = ClientGraphicColor::TEAM, ClientGraphicOperation operation = ClientGraphicOperation::ADD, uint16_t receiver_id = 0);
    bool draw_arc(const char *name, uint16_t center_x, uint16_t center_y, uint16_t start_angle, uint16_t end_angle, uint16_t x_semi_axis, uint16_t y_semi_axis, uint16_t width = 2, uint8_t layer = 0, ClientGraphicColor color = ClientGraphicColor::TEAM, ClientGraphicOperation operation = ClientGraphicOperation::ADD, uint16_t receiver_id = 0);
    bool draw_integer(const char *name, int32_t value, uint16_t start_x, uint16_t start_y, uint16_t font_size, uint16_t width = 2, uint8_t layer = 0, ClientGraphicColor color = ClientGraphicColor::TEAM, ClientGraphicOperation operation = ClientGraphicOperation::ADD, uint16_t receiver_id = 0);
    bool draw_float(const char *name, int32_t value_milli, uint16_t start_x, uint16_t start_y, uint16_t font_size, uint16_t width = 2, uint8_t layer = 0, ClientGraphicColor color = ClientGraphicColor::TEAM, ClientGraphicOperation operation = ClientGraphicOperation::ADD, uint16_t receiver_id = 0);
    bool draw_character(const char *name, const char *text, uint16_t start_x, uint16_t start_y, uint16_t font_size, uint16_t width = 2, uint8_t layer = 0, ClientGraphicColor color = ClientGraphicColor::TEAM, ClientGraphicOperation operation = ClientGraphicOperation::ADD, uint16_t receiver_id = 0);

  private:
    RefSystem &ref_system;
};

extern RefDrawing ref_drawing;
