#include "RefDrawer.hpp"
#include <Arduino.h>

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
    gd.width = 10;    // line width
    gd.start_x = x1;  // x coordinate of start point
    gd.start_y = y1;  // y coordinate of start point
    gd.details_a = 0;
    gd.details_b = 0;
    gd.details_c = 0;
    gd.details_d = x2;  // x coordinate of end point
    gd.details_e = y2;  // y coordinate of end point
    sendPacket(DrawType::DRAW_ONE_GRAPHIC, gd);
}

void RefDrawer::drawRectangle(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint8_t color, uint8_t layer) {
    GraphicData gd = {};
    uint32_t id = getNextGraphicId();
    gd.figure_name[0] = (id >> 16) & 0xFF;
    gd.figure_name[1] = (id >> 8) & 0xFF;
    gd.figure_name[2] = id & 0xFF;
    gd.operate_type = GraphicOperation::ADD;
    gd.figure_type = GraphicType::RECTANGLE;
    gd.layer = layer;
    gd.color = color;
    gd.width = 10;    // line width
    gd.start_x = x1;  // start x
    gd.start_y = y1;  // start y
    gd.details_a = 0;
    gd.details_b = 0;
    gd.details_c = 0;
    gd.details_d = x2;  // diagonal x
    gd.details_e = y2;  // diagonal y
    sendPacket(DrawType::DRAW_ONE_GRAPHIC, gd);
}

void RefDrawer::drawCircle(uint32_t x, uint32_t y, uint32_t radius, uint8_t color, uint8_t layer) {
    GraphicData gd = {};
    uint32_t id = getNextGraphicId();
    gd.figure_name[0] = (id >> 16) & 0xFF;
    gd.figure_name[1] = (id >> 8) & 0xFF;
    gd.figure_name[2] = id & 0xFF;
    gd.operate_type = GraphicOperation::ADD;
    gd.figure_type = GraphicType::CIRCLE;
    gd.layer = layer;
    gd.color = color;
    gd.details_a = 0;
    gd.details_b = 0;
    gd.width = 10;          // line width
    gd.start_x = x;         // center x
    gd.start_y = y;         // center y
    gd.details_c = radius;  // radius
    gd.details_d = 0;
    gd.details_e = 0;
    sendPacket(DrawType::DRAW_ONE_GRAPHIC, gd);
}

void RefDrawer::drawEllipse(uint32_t x, uint32_t y, uint32_t x1, uint32_t y1, uint8_t color, uint8_t layer) {
    GraphicData gd = {};
    uint32_t id = getNextGraphicId();
    gd.figure_name[0] = (id >> 16) & 0xFF;
    gd.figure_name[1] = (id >> 8) & 0xFF;
    gd.figure_name[2] = id & 0xFF;
    gd.operate_type = GraphicOperation::ADD;
    gd.figure_type = GraphicType::ELLIPSE;
    gd.layer = layer;
    gd.color = color;
    gd.details_a = 0;
    gd.details_b = 0;
    gd.details_c = 0;
    gd.details_d = x1;
    gd.details_e = y1;
    gd.start_x = x;
    gd.start_y = y;
    sendPacket(DrawType::DRAW_ONE_GRAPHIC, gd);
}

void RefDrawer::drawArc(uint32_t x, uint32_t y, uint32_t x1, uint32_t y1, uint32_t start_angle, uint32_t end_angle, uint8_t color, uint8_t layer) {
    GraphicData gd = {};
    uint32_t id = getNextGraphicId();
    gd.figure_name[0] = (id >> 16) & 0xFF;
    gd.figure_name[1] = (id >> 8) & 0xFF;
    gd.figure_name[2] = id & 0xFF;
    gd.operate_type = GraphicOperation::ADD;
    gd.figure_type = GraphicType::ARC;
    gd.layer = layer;
    gd.color = color;
    gd.details_a = start_angle;
    gd.details_b = end_angle;
    gd.details_c = 0;
    gd.details_d = x1;
    gd.details_e = y1;
    gd.start_x = x;
    gd.start_y = y;
    sendPacket(DrawType::DRAW_ONE_GRAPHIC, gd);
}

void RefDrawer::drawInt(uint32_t x, uint32_t y, uint32_t fontSize, uint32_t integer, uint8_t color, uint8_t layer) {
    GraphicData gd = {};
    uint32_t id = getNextGraphicId();
    gd.figure_name[0] = (id >> 16) & 0xFF;
    gd.figure_name[1] = (id >> 8) & 0xFF;
    gd.figure_name[2] = id & 0xFF;
    gd.operate_type = GraphicOperation::ADD;
    gd.figure_type = GraphicType::INTEGER;
    gd.layer = layer;
    gd.color = color;
    gd.details_a = fontSize;
    gd.details_b = 0;
    gd.details_c = integer;
    gd.details_d = 0;
    gd.details_e = 0;
    gd.start_x = x;
    gd.start_y = y;
    sendPacket(DrawType::DRAW_ONE_GRAPHIC, gd);
}

void RefDrawer::drawChar(uint32_t x, uint32_t y, uint32_t fontSize, uint32_t charLength, uint8_t color, uint8_t layer) {
    GraphicData gd = {};
    uint32_t id = getNextGraphicId();
    gd.figure_name[0] = (id >> 16) & 0xFF;
    gd.figure_name[1] = (id >> 8) & 0xFF;
    gd.figure_name[2] = id & 0xFF;
    gd.operate_type = GraphicOperation::ADD;
    gd.figure_type = GraphicType::CHARACTER;
    gd.layer = layer;
    gd.color = color;
    gd.details_a = fontSize;
    gd.details_b = charLength;
    gd.details_c = 0;
    gd.details_d = 0;
    gd.details_e = 0;
    gd.start_x = x;
    gd.start_y = y;
    sendPacket(DrawType::DRAW_ONE_GRAPHIC, gd);
}
