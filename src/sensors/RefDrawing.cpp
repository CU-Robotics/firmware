#include "RefDrawing.hpp"

static uint32_t getNextGraphicId() {
    // generate a unique 24 bit graphic id each time this is called.
    static uint32_t id = 0;
    id = (id + 1) & 0x00FFFFFF; // add 1 and wrap at 24bits max
    return id;
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
    // populate graphicData struct

    /// put a unique id in the figure_name field
    uint32_t id = getNextGraphicId();
    graphicData.figure_name[0] = (id >> 16) & 0xFF;
    graphicData.figure_name[1] = (id >> 8) & 0xFF;
    graphicData.figure_name[2] = id & 0xFF;

    graphicData.operate_type = static_cast<uint32_t>(GraphicOperation::ADD);
    graphicData.figure_type = static_cast<uint32_t>(GraphicType::LINE);
    graphicData.layer = 1;
    graphicData.color = static_cast<uint32_t>(color);
    graphicData.details_a = 0; // not used for line
    graphicData.details_b = 0; // not used for line
    graphicData.width = 10; // line width
    graphicData.start_x = x1; // x coordinate of start point
    graphicData.start_y = y1; // y coordinate of start point
    graphicData.details_c = 0; // not used for line
    graphicData.details_d = x2; // x coordinate of end point
    graphicData.details_e = y2; // y coordinate of end point

    return graphicData;
}

GraphicData createFloatData(uint32_t x, uint32_t y, float number, uint32_t font_size, Color color) {
    GraphicData graphicData {};
    // populate graphicData struct

    /// put a unique id in the figure_name field
    uint32_t id = getNextGraphicId();
    graphicData.figure_name[0] = (id >> 16) & 0xFF;
    graphicData.figure_name[1] = (id >> 8) & 0xFF;
    graphicData.figure_name[2] = id & 0xFF;

    graphicData.operate_type = static_cast<uint32_t>(GraphicOperation::ADD);
    graphicData.figure_type = static_cast<uint32_t>(GraphicType::FLOAT);
    graphicData.layer = 1;
    graphicData.color = static_cast<uint32_t>(color);
    graphicData.details_a = font_size; // font size
    graphicData.details_b = 0; // not used for float

    graphicData.width = 10; // line width
    graphicData.start_x = x; // x coordinate of start point
    graphicData.start_y = y; // y coordinate of start point

    // Floating number: all integers are 32 bit. the actual displayed value is 1/1000 of the entered values. Entering 1234 into details c, d, e will display 1.234.
    // multiply by 1000 to get the actual value
    int32_t value = static_cast<int32_t>(number * 1000);
    graphicData.details_c = value & 0x7FF; // least significant 11 bits
    graphicData.details_d = (value >> 11) & 0x7FF; // next 11 bits
    graphicData.details_e = (value >> 21) & 0x3FF; // the last 10 bits

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
    graphicData.details_b = 0; // not used for integer

    graphicData.width = 10; // line width
    graphicData.start_x = x; // x coordinate of start point
    graphicData.start_y = y; // y coordinate of start point

    // 32 bit integer goes into details c, d and e
    graphicData.details_c = number & 0x7FF; // least significant 11 bits
    graphicData.details_d = (number >> 11) & 0x7FF; // next 11 bits
    graphicData.details_e = (number >> 22) & 0x3FF; // the last 10 bits
    return graphicData;
}




