#include "RefSystemPacketDefs.hpp"
#include "RefSystem.hpp"

// Sub-content IDs for various drawing commands
enum DrawType {
    /// @brief a player client deletes graphic layers
    DELETE_LAYER = 0x0100,
    /// @brief a player client draws one graphic
    DRAW_ONE_GRAPHIC = 0x0101,
    /// @brief a player client draws two graphics
    DRAW_TWO_GRAPHICS = 0x0102,
    /// @brief a player client draws five graphics
    DRAW_FIVE_GRAPHICS = 0x0103,
    /// @brief a player client draws seven graphics
    DRAW_SEVEN_GRAPHICS = 0x0104,
    /// @brief a player client draws a character graphic
    DRAW_CHARACTER_GRAPHIC = 0x0110,
};

/// @brief Graphic operation types
enum class GraphicOperation {
    /// @brief No operation (0)
    NONE = 0,
    /// @brief Add a graphic (1)
    ADD,
    /// @brief Modify a graphic (2)
    MODIFY,
    /// @brief Delete a graphic (3)
    DELETE
};

enum class DeleteOperation {
    /// @brief No operation (0)
    NONE = 0,
    /// @brief Delete a graphic layer (1)
    DELETE_LAYER,
    /// @brief Delete all graphic layers (2)
    DELETE_ALL
};

/// @brief Graphic Types
enum class GraphicType {
    /// @brief Straight line (0)
    LINE = 0,
    /// @brief Rectangle (1)
    RECTANGLE,
    /// @brief Circle (2)
    CIRCLE,
    /// @brief Ellipse (3)
    ELLIPSE,
    /// @brief Arc (4)
    ARC,
    /// @brief Float (5)
    FLOATING_NUMBER,
    /// @brief Integer (6)
    INTEGER,
    /// @brief Character (7)
    CHARACTER
};

/// @brief Colors that a graphic can be
enum class Color {
    /// @brief Color of the side of the robot (red/blue) (0
    SIDE_COLOR = 0,
    /// @brief Yellow (1)
    YELLOW,
    /// @brief Green (2)
    GREEN,
    /// @brief Orange (3)
    ORANGE,
    /// @brief Purple (4)
    PURPLE,
    /// @brief Pink (5)
    PINK,
    /// @brief Cyan (6)
    CYAN,
    /// @brief Light blue (7)
    BLACK,
    /// @brief Dark blue (8)
    WHITE
};

/// @brief  Draw a graphic layer
/// @param graphic GraphicData to be drawn
void drawOneGraphic(GraphicData graphic);

/// @brief Draw two graphic layers
/// @param graphic1 GraphicData for the first graphic
/// @param graphic2 GraphicData for the second graphic
void drawTwoGraphics(GraphicData graphic1, GraphicData graphic2);

/// @brief Delete a graphic layer or all layers
/// @param deleteOperation type of delete operation to perform
/// @note Options are in enum class DeleteOperation
void deleteLayer(DeleteOperation deleteOperation, uint8_t layer = 0);

/// @brief Get GraphicData for a line
/// @param x1 x coordinate of the start point
/// @param y1 y coordinate of the start point
/// @param x2 x coordinate of the end point
/// @param y2 y coordinate of the end point
/// @param color color of the line
GraphicData createLineData(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, Color color = Color::SIDE_COLOR);

// /// @brief Get GraphicData for a rectangle
// /// @param x1 x coordinate of the starting point
// /// @param y1 y coordinate of the starting point
// /// @param x2 x coordinate of the diagonal point
// /// @param y2 y coordinate of the diagonal point
// /// @param color color of the rectangle
// GraphicData getRectangle(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, Color color = Color::SIDE_COLOR);

// /// @brief Get GraphicData for a circle
// /// @param x x coordinate of the center
// /// @param y y coordinate of the center
// /// @param radius radius of the circle
// /// @param color color of the circle
// GraphicData getCircle(uint32_t x, uint32_t y, uint32_t radius, Color color = Color::SIDE_COLOR);

// /// @brief Get GraphicData for an ellipse
// /// @param x x coordinate of the center
// /// @param y y coordinate of the center
// /// @param width width of the ellipse
// /// @param height height of the ellipse
// /// @param color color of the ellipse
// GraphicData getEllipse(uint32_t x, uint32_t y, uint32_t width, uint32_t height, Color color = Color::SIDE_COLOR);

// /// @brief Get GraphicData for an arc
// /// @param x x coordinate of the center
// /// @param y y coordinate of the center
// /// @param start_angle start angle of the arc
// /// @param end_angle end angle of the arc
// /// @param width width of the arc
// /// @param height height of the arc
// /// @param color color of the arc
// GraphicData getArc(uint32_t x, uint32_t y, uint32_t start_angle, uint32_t end_angle, uint32_t width, uint32_t height, Color color = Color::SIDE_COLOR);

// /// @brief Get GraphicData for a floating point number
// /// @param x x coordinate
// /// @param y y coordinate
// /// @param number number to be drawn
// /// @param font_size font size of the number
// /// @param color color of the number
// GraphicData getFloatingPointNumber(uint32_t x, uint32_t y, float number, uint32_t font_size = 12, Color color = Color::SIDE_COLOR);

// /// @brief Get GraphicData for an integer
// /// @param x x coordinate
// /// @param y y coordinate
// /// @param number number to be drawn
// /// @param font_size font size of the number
// /// @param color color of the number
// GraphicData getInteger(uint32_t x, uint32_t y, int32_t number, uint32_t font_size = 12, Color color = Color::SIDE_COLOR) {
//     // TODO
//     // draw 32-bit integer, int32_t
// }

// TODO: Draw character?