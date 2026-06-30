#pragma once

#include "RefSystem.hpp"
constexpr size_t MAX_REF_GRAPHICS = 7;

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
    /// @brief Three-byte figure name used by the Player's Client to add, edit, or delete the figure.
    uint8_t name[3] = {0};
    /// @brief Figure operation field: no operation, add, edit, or delete.
    ClientGraphicOperation operation = ClientGraphicOperation::NONE;
    /// @brief Figure type field: line, rectangle, circle, ellipse, arc, float, integer, or character.
    ClientGraphicType type = ClientGraphicType::LINE;
    /// @brief Player-client drawing layer, from 0 to 9.
    uint8_t layer = 0;
    /// @brief Player-client drawing color.
    ClientGraphicColor color = ClientGraphicColor::TEAM;
    /// @brief Type-dependent detail field; used for arc start angle and numeric or character font size.
    uint16_t details_a = 0;
    /// @brief Type-dependent detail field; used for arc end angle or character length.
    uint16_t details_b = 0;
    /// @brief Line width in pixels.
    uint16_t width = 1;
    /// @brief Starting point or center x-coordinate.
    uint16_t start_x = 0;
    /// @brief Starting point or center y-coordinate.
    uint16_t start_y = 0;
    /// @brief Type-dependent detail field; used for circle radius or the low bits of numeric values.
    uint16_t details_c = 0;
    /// @brief Type-dependent detail field; used for end x, diagonal x, semi-axis x, or numeric value bits.
    uint16_t details_d = 0;
    /// @brief Type-dependent detail field; used for end y, diagonal y, semi-axis y, or numeric value bits.
    uint16_t details_e = 0;
};

/// @brief Helper for sending RoboMaster Player's Client drawing packets through the Referee System.
class RefDrawing {
  public:
    /// @brief Construct a RefDrawing helper that sends packets through a RefSystem object.
    /// @param ref_system RefSystem instance used to send robot-interaction frames.
    explicit RefDrawing(RefSystem &ref_system);

    /// @brief Delete one Player's Client drawing layer.
    /// @param layer Layer number to delete, from 0 to 9.
    /// @param receiver_id Player's Client receiver ID, or 0 to use the sender robot's corresponding client.
    /// @return true when the delete-layer packet is accepted for transmission.
    bool delete_layer(uint8_t layer, uint16_t receiver_id = 0);
    /// @brief Delete all Player's Client drawing layers.
    /// @param receiver_id Player's Client receiver ID, or 0 to use the sender robot's corresponding client.
    /// @return true when the delete-all-layers packet is accepted for transmission.
    bool delete_all_layers(uint16_t receiver_id = 0);
    /// @brief Draw or update one packed Player's Client figure.
    /// @param graphic Packed figure description to send.
    /// @param receiver_id Player's Client receiver ID, or 0 to use the sender robot's corresponding client.
    /// @return true when the drawing packet is accepted for transmission.
    bool draw_graphic(const ClientGraphic &graphic, uint16_t receiver_id = 0);
    /// @brief Draw or update a supported batch of packed Player's Client figures.
    /// @param graphics Array of packed figure descriptions to send.
    /// @param graphic_count Number of figures in the array; must be 1, 2, 5, or 7.
    /// @param receiver_id Player's Client receiver ID, or 0 to use the sender robot's corresponding client.
    /// @return true when the drawing packet is accepted for transmission.
    bool draw_graphics(const ClientGraphic *graphics, uint8_t graphic_count, uint16_t receiver_id = 0);
    /// @brief Draw or update a supported batch of packed Player's Client figures. Add padding to get to the
    /// closest valid amount of graphics.
    /// @param graphics Array of packed figure descriptions to send.
    /// @param graphic_count Number of figures in the array; must be 1, 2, 5, or 7.
    /// @param receiver_id Player's Client receiver ID, or 0 to use the sender robot's corresponding client.
    /// @return true when the drawing packet is accepted for transmission.
    bool draw_graphics_with_pad(const ClientGraphic graphics[7], uint8_t graphic_count, uint16_t receiver_id = 0);
    /// @brief Draw or update a straight line on the Player's Client.
    /// @param name Three-character figure name used for later edit or delete operations.
    /// @param start_x Starting x-coordinate.
    /// @param start_y Starting y-coordinate.
    /// @param end_x Ending x-coordinate.
    /// @param end_y Ending y-coordinate.
    /// @param width Line width in pixels.
    /// @param layer Drawing layer, from 0 to 9.
    /// @param color Drawing color.
    /// @param operation Drawing operation to perform.
    /// @param receiver_id Player's Client receiver ID, or 0 to use the sender robot's corresponding client.
    /// @return true when the drawing packet is accepted for transmission.
    bool draw_line(const char *name, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y, uint16_t width = 2, uint8_t layer = 0, ClientGraphicColor color = ClientGraphicColor::TEAM, ClientGraphicOperation operation = ClientGraphicOperation::ADD, uint16_t receiver_id = 0);
    /// @brief Draw or update a rectangle on the Player's Client.
    /// @param name Three-character figure name used for later edit or delete operations.
    /// @param start_x Starting x-coordinate.
    /// @param start_y Starting y-coordinate.
    /// @param end_x Diagonal vertex x-coordinate.
    /// @param end_y Diagonal vertex y-coordinate.
    /// @param width Line width in pixels.
    /// @param layer Drawing layer, from 0 to 9.
    /// @param color Drawing color.
    /// @param operation Drawing operation to perform.
    /// @param receiver_id Player's Client receiver ID, or 0 to use the sender robot's corresponding client.
    /// @return true when the drawing packet is accepted for transmission.
    bool draw_rectangle(const char *name, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y, uint16_t width = 2, uint8_t layer = 0, ClientGraphicColor color = ClientGraphicColor::TEAM, ClientGraphicOperation operation = ClientGraphicOperation::ADD, uint16_t receiver_id = 0);
    /// @brief Draw or update a circle on the Player's Client.
    /// @param name Three-character figure name used for later edit or delete operations.
    /// @param center_x Center x-coordinate.
    /// @param center_y Center y-coordinate.
    /// @param radius Circle radius.
    /// @param width Line width in pixels.
    /// @param layer Drawing layer, from 0 to 9.
    /// @param color Drawing color.
    /// @param operation Drawing operation to perform.
    /// @param receiver_id Player's Client receiver ID, or 0 to use the sender robot's corresponding client.
    /// @return true when the drawing packet is accepted for transmission.
    bool draw_circle(const char *name, uint16_t center_x, uint16_t center_y, uint16_t radius, uint16_t width = 2, uint8_t layer = 0, ClientGraphicColor color = ClientGraphicColor::TEAM, ClientGraphicOperation operation = ClientGraphicOperation::ADD, uint16_t receiver_id = 0);
    /// @brief Draw or update an ellipse on the Player's Client.
    /// @param name Three-character figure name used for later edit or delete operations.
    /// @param center_x Center x-coordinate.
    /// @param center_y Center y-coordinate.
    /// @param x_semi_axis X semi-axis length.
    /// @param y_semi_axis Y semi-axis length.
    /// @param width Line width in pixels.
    /// @param layer Drawing layer, from 0 to 9.
    /// @param color Drawing color.
    /// @param operation Drawing operation to perform.
    /// @param receiver_id Player's Client receiver ID, or 0 to use the sender robot's corresponding client.
    /// @return true when the drawing packet is accepted for transmission.
    bool draw_ellipse(const char *name, uint16_t center_x, uint16_t center_y, uint16_t x_semi_axis, uint16_t y_semi_axis, uint16_t width = 2, uint8_t layer = 0, ClientGraphicColor color = ClientGraphicColor::TEAM, ClientGraphicOperation operation = ClientGraphicOperation::ADD, uint16_t receiver_id = 0);
    /// @brief Draw or update an arc on the Player's Client.
    /// @param name Three-character figure name used for later edit or delete operations.
    /// @param center_x Center x-coordinate.
    /// @param center_y Center y-coordinate.
    /// @param start_angle Start angle, where 0 degrees points up and angles increase clockwise.
    /// @param end_angle End angle, where 0 degrees points up and angles increase clockwise.
    /// @param x_semi_axis X semi-axis length.
    /// @param y_semi_axis Y semi-axis length.
    /// @param width Line width in pixels.
    /// @param layer Drawing layer, from 0 to 9.
    /// @param color Drawing color.
    /// @param operation Drawing operation to perform.
    /// @param receiver_id Player's Client receiver ID, or 0 to use the sender robot's corresponding client.
    /// @return true when the drawing packet is accepted for transmission.
    bool draw_arc(const char *name, uint16_t center_x, uint16_t center_y, uint16_t start_angle, uint16_t end_angle, uint16_t x_semi_axis, uint16_t y_semi_axis, uint16_t width = 2, uint8_t layer = 0, ClientGraphicColor color = ClientGraphicColor::TEAM, ClientGraphicOperation operation = ClientGraphicOperation::ADD, uint16_t receiver_id = 0);
    /// @brief Draw or update an integer value on the Player's Client.
    /// @param name Three-character figure name used for later edit or delete operations.
    /// @param value Integer value to display.
    /// @param start_x Starting x-coordinate.
    /// @param start_y Starting y-coordinate.
    /// @param font_size Text font size.
    /// @param width Line width in pixels.
    /// @param layer Drawing layer, from 0 to 9.
    /// @param color Drawing color.
    /// @param operation Drawing operation to perform.
    /// @param receiver_id Player's Client receiver ID, or 0 to use the sender robot's corresponding client.
    /// @return true when the drawing packet is accepted for transmission.
    bool draw_integer(const char *name, int32_t value, uint16_t start_x, uint16_t start_y, uint16_t font_size, uint16_t width = 2, uint8_t layer = 0, ClientGraphicColor color = ClientGraphicColor::TEAM, ClientGraphicOperation operation = ClientGraphicOperation::ADD, uint16_t receiver_id = 0);
    /// @brief Draw or update a fixed-point floating value on the Player's Client.
    /// @param name Three-character figure name used for later edit or delete operations.
    /// @param value_milli Value to display multiplied by 1000.
    /// @param start_x Starting x-coordinate.
    /// @param start_y Starting y-coordinate.
    /// @param font_size Text font size.
    /// @param width Line width in pixels.
    /// @param layer Drawing layer, from 0 to 9.
    /// @param color Drawing color.
    /// @param operation Drawing operation to perform.
    /// @param receiver_id Player's Client receiver ID, or 0 to use the sender robot's corresponding client.
    /// @return true when the drawing packet is accepted for transmission.
    bool draw_float(const char *name, int32_t value_milli, uint16_t start_x, uint16_t start_y, uint16_t font_size, uint16_t width = 2, uint8_t layer = 0, ClientGraphicColor color = ClientGraphicColor::TEAM, ClientGraphicOperation operation = ClientGraphicOperation::ADD, uint16_t receiver_id = 0);
    /// @brief Draw or update a character string on the Player's Client.
    /// @param name Three-character figure name used for later edit or delete operations.
    /// @param text Character bytes to display, truncated to the 30-byte protocol payload.
    /// @param start_x Starting x-coordinate.
    /// @param start_y Starting y-coordinate.
    /// @param font_size Text font size.
    /// @param width Line width in pixels.
    /// @param layer Drawing layer, from 0 to 9.
    /// @param color Drawing color.
    /// @param operation Drawing operation to perform.
    /// @param receiver_id Player's Client receiver ID, or 0 to use the sender robot's corresponding client.
    /// @return true when the drawing packet is accepted for transmission.
    bool draw_character(const char *name, const char *text, uint16_t start_x, uint16_t start_y, uint16_t font_size, uint16_t width = 2, uint8_t layer = 0, ClientGraphicColor color = ClientGraphicColor::TEAM, ClientGraphicOperation operation = ClientGraphicOperation::ADD, uint16_t receiver_id = 0);

  private:
    /// @brief RefSystem object used to send drawing interaction packets to the referee system.
    RefSystem &ref_system;
};

extern RefDrawing ref_drawing;
