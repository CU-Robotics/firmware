#include "RefDrawing.hpp"

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
    FrameData frameData {};
    memcpy(frameData.data, &robotInteraction, sizeof(RobotInteraction)); // copy the packet data to frameData

    ref->write(&frameData, sizeof(frameData)); // send the frameData to the ref
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
    static uint32_t id = 0;
    graphicData.figure_name[0] = (id >> 16) & 0xFF;
    graphicData.figure_name[1] = (id >> 8) & 0xFF;
    graphicData.figure_name[2] = id & 0xFF;
    id++;
    if (id >= (1 << 24)) { // 0x00FFFFFF is the max value for our id
        // Handle overflow — reset or ceiling
        id = 0;
    }

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

