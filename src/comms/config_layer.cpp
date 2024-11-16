#include "config_layer.hpp"

const Config* const ConfigLayer::configure(HIDLayer* comms) {
    // grab and process all config packets until finished
    while (!is_configured()) {
        comms->ping();
        process(comms->get_incoming_packet(), comms->get_outgoing_packet());
    }

    // put the data from the packets into the config object
    config.fill_data(config_packets, subsec_sizes);

    return &config;
}

void ConfigLayer::process(CommsPacket *in, CommsPacket *out) {
    char *in_raw = in->raw;
    char *out_raw = out->raw;
    int8_t sec_id = in_raw[1];
    uint8_t subsec_id = in_raw[2];
    uint8_t info_bit = in_raw[3];

    // if this is the packet we want
    if (sec_id == seek_sec && subsec_id == seek_subsec && info_bit == 1) {
        // received the initial config packet
        if (sec_id == -1) {
             /*
            the khadas sends a config packet with its raw data set to a byte
            array where each index corresponds to a YAML section, and the value
            at that index indicates the number of subsections for the given section
            */
            num_sec = (in_raw[5] << 8) | in_raw[4];
            memcpy(
                subsec_sizes, 
                in_raw + 8, // byte array starts at byte 8
                num_sec);
            
        #ifdef CONFIG_LAYER_DEBUG
            Serial.printf("YAML metadata received:\n");
            for (int i = 0; i < num_sec; i++) {
                Serial.printf("\tSection %d: %u subsection(s)\n", i, subsec_sizes[i]);
            }
        #endif

            // look for the next YAML section
            seek_sec++;
        } else {
            config_packets[index] = *in;
            index++;
        
        #ifdef CONFIG_LAYER_DEBUG
            Serial.printf("Received YAML configuration packet: (%u, %u)\n", sec_id, subsec_id);
        #endif

            // add one because subsections are zero-indexed
            if (subsec_id + 1 < subsec_sizes[sec_id]) {
                // if we haven't received all subsections, request the next one
                seek_subsec++;
            } else {
                // if all subsections have been received, request the next section
                seek_subsec = 0;
                seek_sec++;

                // or, if all sections have been received, configuration is complete
                // add one because sections are zero-indexed
                if (seek_sec + 1 > num_sec) {
                    configured = true;
                #ifdef CONFIG_LAYER_DEBUG
                    Serial.printf("YAML configuration complete\n");
                #endif
                }
            }

        }
    }

    // if configuration isn't complete, request the next packet
    if (!configured) {
        out_raw[0] = 0xff; // filler byte (needed for some reason)
        out_raw[1] = seek_sec;
        out_raw[2] = seek_subsec;
        out_raw[3] = 1; // set info bit

        // Serial.printf("Requesting YAML configuration packet: (%u, %u)\n", seek_sec, seek_subsec);
    }
}

void Config::fill_data(CommsPacket packets[MAX_CONFIG_PACKETS], uint8_t sizes[MAX_CONFIG_PACKETS]) {
    for (int i = 0; i < MAX_CONFIG_PACKETS; i++) {
        uint8_t id = packets[i].get_id();
        uint8_t subsec_id = *reinterpret_cast<uint8_t*>(packets[i].raw + 2);
        uint16_t sub_size = *reinterpret_cast<uint16_t*>(packets[i].raw + 6);

        if (subsec_id == 0) index = 0;

        Serial.printf("id: %d, subsec_id: %d, sub_size: %d\n", id, subsec_id, sub_size);
        Serial.println();

        if (id == yaml_section_id_mappings.at("motor_info")) {
            size_t linear_index = index / sizeof(float);
            size_t i1 = linear_index / (NUM_MOTORS);
            size_t i2 = linear_index % NUM_MOTORS;
            memcpy(&motor_info[i1][i2], packets[i].raw + 8, sub_size);
            index += sub_size;
            Serial.printf("hello\n");
        }

        if(id == yaml_section_id_mappings.at("sensor_info")){
            size_t linear_index = index / sizeof(float);
            size_t i1 = linear_index / (NUM_SENSORS);
            size_t i2 = linear_index % NUM_SENSORS;
            memcpy(&sensor_info[i1][i2], packets[i].raw + 8, sub_size);
            index += sub_size;
        }

        if(id == yaml_section_id_mappings.at("gains")){
            size_t linear_index = index / sizeof(float);
            size_t i1 = linear_index / (NUM_GAINS);
            size_t i2 = linear_index % NUM_GAINS;
            memcpy(&gains[i1][i2], packets[i].raw + 8, sub_size);
            index += sub_size;
        }
        if (id == yaml_section_id_mappings.at("gear_ratios")) {
            size_t linear_index = index / sizeof(float);
            size_t i1 = linear_index / (NUM_GAINS);
            size_t i2 = linear_index % NUM_GAINS;
            memcpy(&gear_ratios[i1][i2], packets[i].raw + 8, sub_size);
            index += sub_size;
        }

        if (id == yaml_section_id_mappings.at("reference_limits")) {
            size_t linear_index = index / sizeof(float);
            size_t i1 = linear_index / (STATE_LEN * 3 * 2);
            size_t i2 = (linear_index % (STATE_LEN * 3 * 2)) / (3 * 2);
            size_t i3 = (linear_index % (STATE_LEN * 3 * 2)) % (3 * 2);
            memcpy(&set_reference_limits[i1][i2][i3], packets[i].raw + 8, sub_size);
            index += sub_size;
            Serial.printf("indices: %d, %d, %d\n", i1, i2, i3);
        }
        if (id == yaml_section_id_mappings.at("controller_info")) {
            size_t linear_index = index / sizeof(float);
            size_t i1 = linear_index / (NUM_MOTORS);
            size_t i2 = linear_index % NUM_MOTORS;
            memcpy(&controller_info[i1][i2], packets[i].raw + 8, sub_size);
            index += sub_size;
        }
        if (id == yaml_section_id_mappings.at("governor_types")) {
            memcpy(governor_types, packets[i].raw + 8, sub_size);
        }
        if (id == yaml_section_id_mappings.at("estimator_info")) {
            size_t linear_index = index / sizeof(float);
            size_t i1 = linear_index / (STATE_LEN);
            size_t i2 = linear_index % STATE_LEN;
            memcpy(&estimator_info[i1][i2], packets[i].raw + 8, sub_size);
            index += sub_size;
        }
    }

    //print estimoator info
    // for (int i = 0; i < NUM_ESTIMATORS; i++) {
    //     Serial.printf("estimator %d: ", i);
    //     for (int j = 0; j < STATE_LEN + 1; j++) {
    //         Serial.printf("%f, ", estimator_info[i][j]);
    //     }
    //     Serial.println();
    // }

    //print controller info
    // for(int i = 0; i < NUM_ROBOT_CONTROLLERS; i++){
    //     Serial.printf("controller %d: ", i);
    //     for(int j = 0; j < NUM_MOTORS + 1; j++){
    //         Serial.printf("%f, ", controller_info[i][j]);
    //     }
    //     Serial.println();
    // }

    //print governor types
    // for(int i = 0; i < STATE_LEN; i++){
    //     Serial.printf("governor type %d: %f\n", i, governor_types[i]);
    // }

    //print sensor info
    // for(int i = 0; i < NUM_SENSORS; i++){
    //     Serial.printf("sensor %d: ", i);
    //     for(int j = 0; j < NUM_SENSOR_VALUES + 1; j++){
    //         Serial.printf("%f, ", sensor_info[i][j]);
    //     }
    //     Serial.println();
    // }

    //print gains
    // for(int i = 0; i < NUM_ROBOT_CONTROLLERS; i++){
    //     Serial.printf("gains %d: ", i);
    //     for(int j = 0; j < NUM_GAINS; j++){
    //         Serial.printf("%f, ", gains[i][j]);
    //     }
    //     Serial.println();
    // }

    //print gear ratios
    // for(int i = 0; i < NUM_ROBOT_CONTROLLERS; i++){
    //     Serial.printf("gear ratios %d: ", i);
    //     for(int j = 0; j < NUM_MOTORS; j++){
    //         Serial.printf("%f, ", gear_ratios[i][j]);
    //     }
    //     Serial.println();
    // }

    //print motor info 
    // for(int i = 0; i < NUM_MOTORS; i++){
    //     Serial.printf("motor %d: ",  i);
    //     for(int j = 0; j < 3; j++){
    //         Serial.printf("%f, ", motor_info[i][j]);
    //     }
    //     Serial.println();
    // }

    // //print reference limits
    // for(int i = 0; i < STATE_LEN; i++){
    //     for(int j = 0; j < 3; j++){
    //         for(int k = 0; k < 2; k++){
    //             Serial.printf("reference limits %d: %f\n", i, set_reference_limits[i][j][k]);
    //         }
    //     }
    // }
}
