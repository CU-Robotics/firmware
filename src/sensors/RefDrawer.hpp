#ifndef REF_DRAWER_HPP
#define REF_DRAWER_HPP

#include <cstdint>
#include <cstring>

#include "RefSystem.hpp"
#include "RefSystemPacketDefs.hpp"

/// @brief Class for drawing graphics to the desktop client
class RefDrawer {
   public:
    /// @brief Available colors for drawing graphics
    enum Color {
        /// @brief Red or blue, depending on the robot's side (0)
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
        /// @brief Black (7)
        BLACK,
        /// @brief White (8)
        WHITE
    };

    /// @brief Deletes a specific layer
    /// @param layer The layer to delete (0-9)
    void deleteLayer(uint8_t layer);

    /// @brief Deletes all layers
    void deleteAllLayers();

    /// @brief Draw a line
    /// @param x1 x coordinate of the start point
    /// @param y1 y coordinate of the start point
    /// @param x2 x coordinate of the end point
    /// @param y2 y coordinate of the end point
    /// @param color color of the line (default SIDE_COLOR)
    /// @param layer layer to draw the line on (default 0)
    void drawLine(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint8_t color = 0, uint8_t layer = 0);

    /// @brief Draw a rectangle
    /// @param x1 x coordinate of the top-left corner
    /// @param y1 y coordinate of the top-left corner
    /// @param x2 x coordinate of the bottom-right corner
    /// @param y2 y coordinate of the bottom-right corner
    /// @param color color of the rectangle (default SIDE_COLOR)
    /// @param layer layer to draw the rectangle on (default 0)
    void drawRectangle(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint8_t color = 0, uint8_t layer = 0);

    /// @brief Draw a circle
    /// @param x x coordinate of the center
    /// @param y y coordinate of the center
    /// @param radius radius of the circle
    /// @param color color of the circle (default SIDE_COLOR)
    /// @param layer layer to draw the circle on (default 0)
    void drawCircle(uint32_t x, uint32_t y, uint32_t radius, uint8_t color = 0, uint8_t layer = 0);

   private:
    /// @brief Available drawing commands
    enum DrawType {
        /// @brief Delete layers command
        DELETE_LAYERS = 0x0100,
        /// @brief Draw one graphic command
        DRAW_ONE_GRAPHIC = 0x0101,
    };

    /// @brief Sends a drawing command through ref
    /// @tparam T Type of the drawing data (either GraphicData or LayerData)
    /// @param type Type of drawing command
    /// @param data GraphicData or LayerData to send
    template <typename T>
    void sendPacket(DrawType type, const T& data) {
        RobotInteraction ri = {};
        ri.content_id = static_cast<uint16_t>(type);
        ri.sender_id = ref->ref_data.robot_performance.robot_ID;
        ri.receiver_id = ri.sender_id >> 8;  // Send to own team's player client (upper byte of robot ID is team ID)
        ri.size = sizeof(data);
        memcpy(ri.data, &data, ri.size);

        FrameData fd = {};
        size_t idx = 0;
        memcpy(fd.data + idx, &ri.content_id, sizeof(ri.content_id));
        idx += sizeof(ri.content_id);
        memcpy(fd.data + idx, &ri.sender_id, sizeof(ri.sender_id));
        idx += sizeof(ri.sender_id);
        memcpy(fd.data + idx, &ri.receiver_id, sizeof(ri.receiver_id));
        idx += sizeof(ri.receiver_id);
        memcpy(fd.data + idx, ri.data, ri.size);
        idx += ri.size;

        ref->write(&fd, idx);
    }
};

#endif  // REF_DRAWER_HPP