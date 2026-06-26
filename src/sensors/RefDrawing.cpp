#include "RefDrawing.hpp"

static constexpr uint8_t CLIENT_CHARACTER_DATA_SIZE = 30;

static void put_u32(uint8_t *data, uint32_t value) {
    data[0] = value & 0x000000FF;
    data[1] = (value >> 8) & 0x000000FF;
    data[2] = (value >> 16) & 0x000000FF;
    data[3] = value >> 24;
}

static void copy_graphic_name(uint8_t name[3], const char *source) {
    name[0] = 0;
    name[1] = 0;
    name[2] = 0;

    if (source == nullptr) {
        return;
    }

    for (uint8_t i = 0; i < 3 && source[i] != '\0'; i++) {
        name[i] = static_cast<uint8_t>(source[i]);
    }
}

static uint8_t copy_client_text(uint8_t *destination, const char *text, uint8_t max_length) {
    if (text == nullptr) {
        return 0;
    }

    uint8_t length = 0;
    while (length < max_length && text[length] != '\0') {
        destination[length] = static_cast<uint8_t>(text[length]);
        length++;
    }
    return length;
}

static ClientGraphic make_client_graphic(const char *name, ClientGraphicType type, uint16_t start_x, uint16_t start_y, uint16_t width, uint8_t layer, ClientGraphicColor color, ClientGraphicOperation operation) {
    ClientGraphic graphic;
    copy_graphic_name(graphic.name, name);
    graphic.operation = operation;
    graphic.type = type;
    graphic.layer = layer;
    graphic.color = color;
    graphic.width = width;
    graphic.start_x = start_x;
    graphic.start_y = start_y;
    return graphic;
}

static void set_client_graphic_value(ClientGraphic &graphic, int32_t value) {
    uint32_t raw_value = static_cast<uint32_t>(value);
    graphic.details_c = raw_value & 0x000003FF;
    graphic.details_d = (raw_value >> 10) & 0x000007FF;
    graphic.details_e = (raw_value >> 21) & 0x000007FF;
}

static void pack_client_graphic(const ClientGraphic &graphic, uint8_t *data) {
    data[0] = graphic.name[0];
    data[1] = graphic.name[1];
    data[2] = graphic.name[2];

    uint32_t config_1 = 0;
    config_1 |= static_cast<uint32_t>(graphic.operation) & 0x00000007;
    config_1 |= (static_cast<uint32_t>(graphic.type) & 0x00000007) << 3;
    config_1 |= (static_cast<uint32_t>(graphic.layer) & 0x0000000F) << 6;
    config_1 |= (static_cast<uint32_t>(graphic.color) & 0x0000000F) << 10;
    config_1 |= (static_cast<uint32_t>(graphic.details_a) & 0x000001FF) << 14;
    config_1 |= (static_cast<uint32_t>(graphic.details_b) & 0x000001FF) << 23;
    put_u32(data + 3, config_1);

    uint32_t config_2 = 0;
    config_2 |= static_cast<uint32_t>(graphic.width) & 0x000003FF;
    config_2 |= (static_cast<uint32_t>(graphic.start_x) & 0x000007FF) << 10;
    config_2 |= (static_cast<uint32_t>(graphic.start_y) & 0x000007FF) << 21;
    put_u32(data + 7, config_2);

    uint32_t config_3 = 0;
    config_3 |= static_cast<uint32_t>(graphic.details_c) & 0x000003FF;
    config_3 |= (static_cast<uint32_t>(graphic.details_d) & 0x000007FF) << 10;
    config_3 |= (static_cast<uint32_t>(graphic.details_e) & 0x000007FF) << 21;
    put_u32(data + 11, config_3);
}
RefDrawing::RefDrawing(RefSystem &ref_system) : ref_system(ref_system) {}

bool RefDrawing::delete_layer(uint8_t layer, uint16_t receiver_id) {
    uint8_t payload[2] = {
        static_cast<uint8_t>(ClientLayerDeleteOperation::DELETE_LAYER),
        layer,
    };
    return ref_system.write_robot_interaction(RobotInteraction::DELETE_CLIENT_LAYER, payload, sizeof(payload), receiver_id);
}

bool RefDrawing::delete_all_layers(uint16_t receiver_id) {
    uint8_t payload[2] = {
        static_cast<uint8_t>(ClientLayerDeleteOperation::DELETE_ALL),
        0,
    };
    return ref_system.write_robot_interaction(RobotInteraction::DELETE_CLIENT_LAYER, payload, sizeof(payload), receiver_id);
}

bool RefDrawing::draw_graphic(const ClientGraphic &graphic, uint16_t receiver_id) { return draw_graphics(&graphic, 1, receiver_id); }

bool RefDrawing::draw_graphics(const ClientGraphic *graphics, uint8_t graphic_count, uint16_t receiver_id) {
    if (graphics == nullptr) {
        Serial.println("No client graphics to draw");
        return false;
    }

    uint16_t content_id = 0;
    switch (graphic_count) {
    case 1:
        content_id = RobotInteraction::DRAW_CLIENT_GRAPHIC_1;
        break;
    case 2:
        content_id = RobotInteraction::DRAW_CLIENT_GRAPHIC_2;
        break;
    case 5:
        content_id = RobotInteraction::DRAW_CLIENT_GRAPHIC_5;
        break;
    case 7:
        content_id = RobotInteraction::DRAW_CLIENT_GRAPHIC_7;
        break;
    default:
        Serial.println("Client graphics count must be 1, 2, 5, or 7");
        return false;
    }

    uint8_t payload[ClientGraphic::packet_size * 7] = {0};
    for (uint8_t i = 0; i < graphic_count; i++) {
        pack_client_graphic(graphics[i], payload + (i * ClientGraphic::packet_size));
    }

    uint16_t payload_length = static_cast<uint16_t>(graphic_count) * ClientGraphic::packet_size;
    return ref_system.write_robot_interaction(content_id, payload, payload_length, receiver_id);
}

bool RefDrawing::draw_line(const char *name, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y, uint16_t width, uint8_t layer, ClientGraphicColor color, ClientGraphicOperation operation, uint16_t receiver_id) {
    ClientGraphic graphic = make_client_graphic(name, ClientGraphicType::LINE, start_x, start_y, width, layer, color, operation);
    graphic.details_d = end_x;
    graphic.details_e = end_y;
    return draw_graphic(graphic, receiver_id);
}

bool RefDrawing::draw_rectangle(const char *name, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y, uint16_t width, uint8_t layer, ClientGraphicColor color, ClientGraphicOperation operation, uint16_t receiver_id) {
    ClientGraphic graphic = make_client_graphic(name, ClientGraphicType::RECTANGLE, start_x, start_y, width, layer, color, operation);
    graphic.details_d = end_x;
    graphic.details_e = end_y;
    return draw_graphic(graphic, receiver_id);
}

bool RefDrawing::draw_circle(const char *name, uint16_t center_x, uint16_t center_y, uint16_t radius, uint16_t width, uint8_t layer, ClientGraphicColor color, ClientGraphicOperation operation, uint16_t receiver_id) {
    ClientGraphic graphic = make_client_graphic(name, ClientGraphicType::CIRCLE, center_x, center_y, width, layer, color, operation);
    graphic.details_c = radius;
    return draw_graphic(graphic, receiver_id);
}

bool RefDrawing::draw_ellipse(const char *name, uint16_t center_x, uint16_t center_y, uint16_t x_semi_axis, uint16_t y_semi_axis, uint16_t width, uint8_t layer, ClientGraphicColor color, ClientGraphicOperation operation, uint16_t receiver_id) {
    ClientGraphic graphic = make_client_graphic(name, ClientGraphicType::ELLIPSE, center_x, center_y, width, layer, color, operation);
    graphic.details_d = x_semi_axis;
    graphic.details_e = y_semi_axis;
    return draw_graphic(graphic, receiver_id);
}

bool RefDrawing::draw_arc(const char *name, uint16_t center_x, uint16_t center_y, uint16_t start_angle, uint16_t end_angle, uint16_t x_semi_axis, uint16_t y_semi_axis, uint16_t width, uint8_t layer, ClientGraphicColor color, ClientGraphicOperation operation, uint16_t receiver_id) {
    ClientGraphic graphic = make_client_graphic(name, ClientGraphicType::ARC, center_x, center_y, width, layer, color, operation);
    graphic.details_a = start_angle;
    graphic.details_b = end_angle;
    graphic.details_d = x_semi_axis;
    graphic.details_e = y_semi_axis;
    return draw_graphic(graphic, receiver_id);
}

bool RefDrawing::draw_integer(const char *name, int32_t value, uint16_t start_x, uint16_t start_y, uint16_t font_size, uint16_t width, uint8_t layer, ClientGraphicColor color, ClientGraphicOperation operation, uint16_t receiver_id) {
    ClientGraphic graphic = make_client_graphic(name, ClientGraphicType::INTEGER, start_x, start_y, width, layer, color, operation);
    graphic.details_a = font_size;
    set_client_graphic_value(graphic, value);
    return draw_graphic(graphic, receiver_id);
}

bool RefDrawing::draw_float(const char *name, int32_t value_milli, uint16_t start_x, uint16_t start_y, uint16_t font_size, uint16_t width, uint8_t layer, ClientGraphicColor color, ClientGraphicOperation operation, uint16_t receiver_id) {
    ClientGraphic graphic = make_client_graphic(name, ClientGraphicType::FLOAT, start_x, start_y, width, layer, color, operation);
    graphic.details_a = font_size;
    set_client_graphic_value(graphic, value_milli);
    return draw_graphic(graphic, receiver_id);
}

bool RefDrawing::draw_character(const char *name, const char *text, uint16_t start_x, uint16_t start_y, uint16_t font_size, uint16_t width, uint8_t layer, ClientGraphicColor color, ClientGraphicOperation operation, uint16_t receiver_id) {
    uint8_t payload[ClientGraphic::packet_size + CLIENT_CHARACTER_DATA_SIZE] = {0};

    ClientGraphic graphic = make_client_graphic(name, ClientGraphicType::CHARACTER, start_x, start_y, width, layer, color, operation);
    graphic.details_a = font_size;
    graphic.details_b = copy_client_text(payload + ClientGraphic::packet_size, text, CLIENT_CHARACTER_DATA_SIZE);
    pack_client_graphic(graphic, payload);

    return ref_system.write_robot_interaction(RobotInteraction::DRAW_CLIENT_CHARACTER, payload, sizeof(payload), receiver_id);
}
