#include "RefDrawing.hpp"

static uint32_t getNextGraphicId() {
    // generate a unique 24 bit graphic id each time this is called.
    static uint32_t id = 0;
    return id++ & 0x00FFFFFF; // add 1 and wrap at 24bits max
}

void deleteLayer(DeleteOperation deleteOperation, uint8_t layer) {
    // make DeleteLayerData
    DeleteLayerData deleteLayerData {};

    deleteLayerData.delete_type = static_cast<uint8_t>(deleteOperation); // type of delete operation
    deleteLayerData.layer = layer; // number of layer to delete

    // populate RobotInteraction packet with the data
    RobotInteraction robotInteraction = {};
    robotInteraction.content_id = DrawType::DELETE_LAYER; // ID for DELETE_LAYER
    robotInteraction.sender_id = ref->ref_data.robot_performance.robot_ID; // ID of the robot that should send this packet
    robotInteraction.receiver_id = ref->ref_data.robot_performance.robot_ID >> 8; // ID of the robot that should receive this packet
    robotInteraction.size = sizeof(DeleteLayerData); // size of the data
    memcpy(robotInteraction.data, &deleteLayerData, sizeof(DeleteLayerData)); // copy the data to the packet

    // put the robotInteraction packet into a FrameData
    FrameData frameData = {};
    uint8_t index = 0;
    memcpy(frameData.data + index, &robotInteraction.content_id, sizeof(robotInteraction.content_id)); // copy the content_id
    index += sizeof(robotInteraction.content_id);
    memcpy(frameData.data + index, &robotInteraction.sender_id, sizeof(robotInteraction.sender_id)); // copy the sender_id
    index += sizeof(robotInteraction.sender_id);
    memcpy(frameData.data + index, &robotInteraction.receiver_id, sizeof(robotInteraction.receiver_id)); // copy the receiver_id
    index += sizeof(robotInteraction.receiver_id);
    memcpy(frameData.data + index, &robotInteraction.data, robotInteraction.size); // copy the data
    index += robotInteraction.size;

    ref->write(&frameData, index); // send the frameData to the ref
}

void drawOneGraphic(GraphicData graphicData) {
    // make DrawOneGraphicData
    DrawOneGraphicData drawOneGraphicData {};
    drawOneGraphicData.graphic = graphicData;

    // populate a RobotInteraction packet with the data
    RobotInteraction robotInteraction = {};
    robotInteraction.content_id = DrawType::DRAW_ONE_GRAPHIC; // ID for DRAW_ONE_GRAPHIC
    robotInteraction.sender_id = 0; // ID of the robot that should send this packet
    robotInteraction.receiver_id = 0; // ID of the robot that should receive this packet
    robotInteraction.size = sizeof(DrawOneGraphicData); // size of the data
    memcpy(robotInteraction.data, &drawOneGraphicData, sizeof(DrawOneGraphicData)); // copy the data to the packet

    // Send the RobotInteraction packet to the ref
    // TODO once we rewrite ref->write(...);
}

GraphicData createLineData(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, Color color) {
    GraphicData graphicData {};
    uint32_t id = getNextGraphicId();
    graphicData.figure_name[0] = (id >> 16) & 0xFF;
    graphicData.figure_name[1] = (id >> 8) & 0xFF;
    graphicData.figure_name[2] = id & 0xFF;
    graphicData.operate_type = static_cast<uint32_t>(GraphicOperation::ADD);
    graphicData.figure_type = static_cast<uint32_t>(GraphicType::LINE);
    graphicData.layer = 1;
    graphicData.color = static_cast<uint32_t>(color);
    graphicData.details_a = 0;
    graphicData.details_b = 0;
    graphicData.width = 10; // line width
    graphicData.start_x = x1; // x coordinate of start point
    graphicData.start_y = y1; // y coordinate of start point
    graphicData.details_c = 0;
    graphicData.details_d = x2; // x coordinate of end point
    graphicData.details_e = y2; // y coordinate of end point
    return graphicData;
}

GraphicData createRectangleData(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, Color color) {
    GraphicData graphicData {};
    uint32_t id = getNextGraphicId();
    graphicData.figure_name[0] = (id >> 16) & 0xFF;
    graphicData.figure_name[1] = (id >> 8) & 0xFF;
    graphicData.figure_name[2] = id & 0xFF;
    graphicData.operate_type = static_cast<uint32_t>(GraphicOperation::ADD);
    graphicData.figure_type = static_cast<uint32_t>(GraphicType::RECTANGLE);
    graphicData.layer = 1;
    graphicData.color = static_cast<uint32_t>(color);
    graphicData.details_a = 0;
    graphicData.details_b = 0;
    graphicData.width = 10; // line width
    graphicData.start_x = x1;
    graphicData.start_y = y1;
    graphicData.details_c = 0;
    graphicData.details_d = x2; // x coordinate of the diagonal point
    graphicData.details_e = y2; // y coordinate of the diagonal point
    return graphicData;
}

GraphicData createCircleData(uint32_t x, uint32_t y, uint32_t radius, Color color) {
    GraphicData graphicData {};
    uint32_t id = getNextGraphicId();
    graphicData.figure_name[0] = (id >> 16) & 0xFF;
    graphicData.figure_name[1] = (id >> 8) & 0xFF;
    graphicData.figure_name[2] = id & 0xFF;
    graphicData.operate_type = static_cast<uint32_t>(GraphicOperation::ADD);
    graphicData.figure_type = static_cast<uint32_t>(GraphicType::CIRCLE);
    graphicData.layer = 1;
    graphicData.color = static_cast<uint32_t>(color);
    graphicData.details_a = 0;
    graphicData.details_b = 0;
    graphicData.width = 10; // line width
    graphicData.start_x = x;
    graphicData.start_y = y;
    graphicData.details_c = radius; // radius of the circle
    graphicData.details_d = 0;
    graphicData.details_e = 0;
    return graphicData;
}

GraphicData createEllipseData(uint32_t x, uint32_t y, uint32_t width, uint32_t height, Color color) {
    GraphicData graphicData {};
    uint32_t id = getNextGraphicId();
    graphicData.figure_name[0] = (id >> 16) & 0xFF;
    graphicData.figure_name[1] = (id >> 8) & 0xFF;
    graphicData.figure_name[2] = id & 0xFF;
    graphicData.operate_type = static_cast<uint32_t>(GraphicOperation::ADD);
    graphicData.figure_type = static_cast<uint32_t>(GraphicType::ELLIPSE);
    graphicData.layer = 1;
    graphicData.color = static_cast<uint32_t>(color);
    graphicData.details_a = 0;
    graphicData.details_b = 0;
    graphicData.width = 10; // line width
    graphicData.start_x = x;
    graphicData.start_y = y;
    graphicData.details_c = 0;
    graphicData.details_d = width; // length of the x axis
    graphicData.details_e = height; // length of the y axis
    return graphicData;
}

GraphicData createFloatData(uint32_t x, uint32_t y, float number, uint32_t font_size, Color color) {
    GraphicData graphicData {};
    uint32_t id = getNextGraphicId();
    graphicData.figure_name[0] = (id >> 16) & 0xFF;
    graphicData.figure_name[1] = (id >> 8) & 0xFF;
    graphicData.figure_name[2] = id & 0xFF;
    graphicData.operate_type = static_cast<uint32_t>(GraphicOperation::ADD);
    graphicData.figure_type = static_cast<uint32_t>(GraphicType::FLOAT);
    graphicData.layer = 1;
    graphicData.color = static_cast<uint32_t>(color);
    graphicData.details_a = font_size; // font size
    graphicData.details_b = 0;
    graphicData.width = 10; // line width
    graphicData.start_x = x;
    graphicData.start_y = y;
    // Floating number: all integers are 32 bit. the actual displayed value is 1/1000 of the entered values. 
    // Entering 1234 into details c, d, e will display 1.234.
    // multiply by 1000 to get the value we will pack into the details c, d, e
    uint32_t unumber = static_cast<uint32_t>(number * 1000);
    // most significant 10 bits into details c
    graphicData.details_c = (unumber >> 22) & 0x3FF; // 10 bits
    // next 11 bits go into details d
    graphicData.details_d = (unumber >> 11) & 0x7FF; // 11 bits
    // least significant 11 bits go into details e
    graphicData.details_e = unumber & 0x7FF; // 11 bits
    return graphicData;
}

GraphicData createIntegerData(uint32_t x, uint32_t y, int32_t number, uint32_t font_size, Color color) {
    GraphicData graphicData {};
    uint32_t id = getNextGraphicId();
    graphicData.figure_name[0] = (id >> 16) & 0xFF;
    graphicData.figure_name[1] = (id >> 8) & 0xFF;
    graphicData.figure_name[2] = id & 0xFF;
    graphicData.operate_type = static_cast<uint32_t>(GraphicOperation::ADD);
    graphicData.figure_type = static_cast<uint32_t>(GraphicType::INTEGER);
    graphicData.layer = 1;
    graphicData.color = static_cast<uint32_t>(color);
    graphicData.details_a = font_size; // font size
    graphicData.details_b = 0;
    graphicData.width = 10; // line width
    graphicData.start_x = x;
    graphicData.start_y = y;
    // 32 bit integer goes into details c, d and e
    // most significant 10 bits into details c
    uint32_t unumber = static_cast<uint32_t>(number);
    graphicData.details_c = (unumber >> 22) & 0x3FF; // 10 bits
    // next 11 bits go into details d
    graphicData.details_d = (unumber >> 11) & 0x7FF; // 11 bits
    // least significant 11 bits go into details e
    graphicData.details_e = unumber & 0x7FF; // 11 bits
    return graphicData;
}




