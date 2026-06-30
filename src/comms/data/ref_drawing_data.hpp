#pragma once

#include "comms/data/comms_data.hpp"
#include "sensors/RefDrawing.hpp"

enum class GraphicOperation : uint32_t {
    NONE = 0,
    ADD = 1,
    EDIT = 2,
    DELETE = 3,
};

enum class GraphicType : uint32_t {
    LINE = 0,
    RECTANGLE = 1,
    CIRCLE = 2,
    ELLIPSE = 3,
    ARC = 4,
    FLOAT = 5,
    INTEGER = 6,
    CHARACTER = 7,
};

enum class GraphicColor : uint32_t {
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

enum class DeleteOperation : uint32_t {
    NONE = 0,
    DELETE_LAYER = 1,
    DELETE_ALL = 2,
};

struct CommsRefGraphic {
    uint8_t name[3] = {0};
    uint8_t _padding = 0;
    GraphicOperation operation = GraphicOperation::NONE;
    GraphicType type = GraphicType::LINE;
    uint32_t layer = 0;
    GraphicColor color = GraphicColor::TEAM;
    uint32_t details_a = 0;
    uint32_t details_b = 0;
    uint32_t width = 1;
    uint32_t start_x = 0;
    uint32_t start_y = 0;
    uint32_t details_c = 0;
    uint32_t details_d = 0;
    uint32_t details_e = 0;

    ClientGraphic to_client_graphic() {
        ClientGraphic cg;
        for (int i = 0; i < 3; ++i) {
            cg.name[i] = name[i];
        }
        cg.operation = static_cast<ClientGraphicOperation>(static_cast<uint8_t>(operation));
        cg.type = static_cast<ClientGraphicType>(static_cast<uint8_t>(type));
        cg.layer = static_cast<uint8_t>(layer);
        cg.color = static_cast<ClientGraphicColor>(static_cast<uint8_t>(color));
        cg.details_a = static_cast<uint16_t>(details_a);
        cg.details_b = static_cast<uint16_t>(details_b);
        cg.width = static_cast<uint16_t>(width);
        cg.start_x = static_cast<uint16_t>(start_x);
        cg.start_y = static_cast<uint16_t>(start_y);
        cg.details_c = static_cast<uint16_t>(details_c);
        cg.details_d = static_cast<uint16_t>(details_d);
        cg.details_e = static_cast<uint16_t>(details_e);
        return cg;
    }
};

struct RefDrawingData : Comms::CommsData {
    RefDrawingData() : CommsData(Comms::TypeLabel::RefDrawingData, Comms::PhysicalMedium::Ethernet, Comms::Priority::High, sizeof(RefDrawingData)) {}

    uint32_t num_graphics = 0;
    CommsRefGraphic graphics[7] = {};

    void fill_client_graphics(ClientGraphic c_graphics[MAX_REF_GRAPHICS]) {
        for (size_t i = 0; i < MAX_REF_GRAPHICS; i++) {
            c_graphics[i] = graphics[i].to_client_graphic();
        }
    }
};