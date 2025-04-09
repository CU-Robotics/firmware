#include "RefSystemPacketDefs.hpp"

/// @brief Graphic operation types
enum class GraphicOperation {
    /// @brief No operation
    NONE = 0,
    /// @brief Add a graphic
    ADD,
    /// @brief Modify a graphic
    MODIFY,
    /// @brief Delete a graphic
    DELETE
};

/// @brief Graphic Types
enum class GraphicType {
    LINE = 0,
    RECTANGLE,
    CIRCLE,
    ELLIPSE,
    ARC,
    FLOATING_NUMBER,
    INTEGER,
    CHARACTER
};

/// @brief Colors that a graphic can be
enum class Color {
    /// @brief Color of the side of the robot (red/blue)
    SIDE_COLOR = 0,
    YELLOW,
    GREEN,
    ORANGE,
    PURPLE,
    PINK,
    CYAN,
    BLACK,
    WHITE
};


void drawLine(int x1, int y1, int x2, int y2, Color color = Color::SIDE_COLOR) {
    // TODO
}

void drawRectangle(int x1, int y1, int x2, int y2, Color color = Color::SIDE_COLOR) {
    // TODO
}

void drawCircle(int x, int y, int radius, Color color = Color::SIDE_COLOR) {
    // TODO
}

void drawEllipse(int x, int y, int width, int height, Color color = Color::SIDE_COLOR) {
    // TODO
}

void drawArc(int x, int y, int start_angle, int end_angle, int width, int height, Color color = Color::SIDE_COLOR) {
    // TODO
}

void drawFloatingNumber(int x, int y, float number, int font_size = 12, Color color = Color::SIDE_COLOR) {
    // TODO
    // NOTE ! "divide value by 1000 to get actual displayed value"
}

void drawInteger(int x, int y, int32_t number, int font_size = 12, Color color = Color::SIDE_COLOR) {
    // TODO
    // draw 32-bit integer, int32_t
}

// TODO: Draw character?