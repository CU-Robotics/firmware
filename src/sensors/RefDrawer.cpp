#include "RefDrawer.hpp"

uint32_t getNextGraphicId() {
    static uint32_t id = 0;
    return id++ & 0x00FFFFFF;
}

enum LayerOperation { DELETE_LAYER = 1, DELETE_ALL };

enum GraphicType { LINE = 0, RECTANGLE, CIRCLE, ELLIPSE, ARC, FLOAT, INTEGER, CHARACTER };

enum GraphicOperation { ADD = 1, MODIFY, DELETE };

void RefDrawer::deleteLayer(uint8_t layer) {
    if (layer > 9) {
        Serial.println("Did you seriously just try to delete a RefDrawer layer over 9?");
        return;
    }
    LayerData ld = {};
    ld.delete_type = LayerOperation::DELETE_LAYER;
    ld.layer = layer;
    sendPacket(DrawType::DELETE_LAYERS, ld);
}

void RefDrawer::deleteAllLayers() {
    LayerData ld = {};
    ld.delete_type = LayerOperation::DELETE_ALL;
    ld.layer = 0;
    sendPacket(DrawType::DELETE_LAYERS, ld);
}

void RefDrawer::drawLine(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint8_t color, uint8_t layer) {
    GraphicData gd = {};
    uint32_t id = getNextGraphicId();
    gd.figure_name[0] = (id >> 16) & 0xFF;
    gd.figure_name[1] = (id >> 8) & 0xFF;
    gd.figure_name[2] = id & 0xFF;
    gd.operate_type = GraphicOperation::ADD;
    gd.figure_type = GraphicType::LINE;
    gd.layer = layer;
    gd.color = color;
    gd.details_a = 0;
    gd.details_b = 0;
    gd.width = 10;    // line width
    gd.start_x = x1;  // x coordinate of start point
    gd.start_y = y1;  // y coordinate of start point
    gd.details_c = 0;
    gd.details_d = x2;  // x coordinate of end point
    gd.details_e = y2;  // y coordinate of end point
    sendPacket(DrawType::DRAW_ONE_GRAPHIC, gd);
}
