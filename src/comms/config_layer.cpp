#include "config_layer.hpp"

const Config* const ConfigLayer::configure(HIDLayer* comms) {
    // check SD
    if(sdcard.exists("/.config")){
        // load sd config into config_packets
        configured = sd_load();
    }
    
    // if no config on SD, then await transmission
    // grab and process all config packets until finished
    while (!is_configured()) {
        comms->ping();
        process(comms->get_incoming_packet(), comms->get_outgoing_packet());
    }

    // update stored config
    store_config();

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

        if (id == yaml_section_id_mappings.at("kinematics_p")) {
            size_t linear_index = index / sizeof(float);
            size_t i1 = linear_index / STATE_LEN;
            size_t i2 = linear_index % STATE_LEN;
            memcpy(&kinematics_p[i1][i2], packets[i].raw + 8, sub_size);
            index += sub_size;
        }
        if (id == yaml_section_id_mappings.at("kinematics_v")) {
            size_t linear_index = index / sizeof(float);
            size_t i1 = linear_index / STATE_LEN;
            size_t i2 = linear_index % STATE_LEN;
            memcpy(&kinematics_v[i1][i2], packets[i].raw + 8, sub_size);
            index += sub_size;
        }
        if (id == yaml_section_id_mappings.at("gains")) {
            size_t linear_index = index / sizeof(float);
            size_t i1 = linear_index / (NUM_CONTROLLER_LEVELS * NUM_GAINS);
            size_t i2 = (linear_index % (NUM_CONTROLLER_LEVELS * NUM_GAINS)) / NUM_GAINS;
            size_t i3 = (linear_index % (NUM_CONTROLLER_LEVELS * NUM_GAINS)) % NUM_GAINS;
            memcpy(&gains[i1][i2][i3], packets[i].raw + 8, sub_size);
            // Serial.println(index/sizeof(float));
            index += sub_size;
        }
        if (id == yaml_section_id_mappings.at("assigned_states")) {
            size_t linear_index = index / sizeof(float);
            size_t i1 = linear_index / STATE_LEN;
            size_t i2 = linear_index % STATE_LEN;
            memcpy(&assigned_states[i1][i2], packets[i].raw + 8, sub_size);
            index += sub_size;
        }
        if (id == yaml_section_id_mappings.at("num_states_per_estimator")) {
            memcpy(num_states_per_estimator, packets[i].raw + 8, sub_size);
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
        if (id == yaml_section_id_mappings.at("yaw_axis_vector")) {
            memcpy(yaw_axis_vector, packets[i].raw + 8, sub_size);
        }
        if (id == yaml_section_id_mappings.at("pitch_axis_vector")) {
            memcpy(pitch_axis_vector, packets[i].raw + 8, sub_size);
        }
        if (id == yaml_section_id_mappings.at("default_gimbal_starting_angles")) {
            memcpy(default_gimbal_starting_angles, packets[i].raw + 8, sub_size);
        }
        if (id == yaml_section_id_mappings.at("default_chassis_starting_angles")) {
            memcpy(default_chassis_starting_angles, packets[i].raw + 8, sub_size);
        }
        if (id == yaml_section_id_mappings.at("controller_types")) {
            memcpy(controller_types, packets[i].raw + 8, sub_size);
        }
        if (id == yaml_section_id_mappings.at("pitch_angle_at_yaw_imu_calibration")) {
            memcpy(&pitch_angle_at_yaw_imu_calibration, packets[i].raw + 8, sub_size);
        }
        if (id == yaml_section_id_mappings.at("governor_types")) {
            memcpy(governor_types, packets[i].raw + 8, sub_size);
        }
        if (id == yaml_section_id_mappings.at("drive_conversion_factors")) {
            memcpy(drive_conversion_factors, packets[i].raw + 8, sub_size);
        }
        if (id == yaml_section_id_mappings.at("estimators")) {
            memcpy(estimators, packets[i].raw + 8, sub_size);
            Serial.println(sub_size);
            Serial.println(estimators[0]);
        }
        if (id == yaml_section_id_mappings.at("odom_values")) {
            memcpy(odom_values, packets[i].raw + 8, sub_size);
        }
        if (id == yaml_section_id_mappings.at("switcher_values")) {
            memcpy(switcher_values, packets[i].raw + 8, sub_size);
        }
        if (id == yaml_section_id_mappings.at("num_sensors")) {
            memcpy(&num_sensors, packets[i].raw + 8, sub_size);
        }
        if (id == yaml_section_id_mappings.at("encoder_offsets")) {
            memcpy(encoder_offsets, packets[i].raw + 8, sub_size);
        }
        if (id == yaml_section_id_mappings.at("encoder_pins")) {
            memcpy(encoder_pins, packets[i].raw + 8, sub_size);
        }
    }
}

bool ConfigLayer::sd_load(){
    // num of total bytes in config_packets
    int config_byte_size = MAX_CONFIG_PACKETS * sizeof(CommsPacket);
    uint8_t buf[config_byte_size];
    sdcard.open("/.config");

    // read config file into buf, copy into config_packets
    sdcard.read(buf, config_byte_size);
    memcpy(config_packets, buf, config_byte_size);

    // read config file into buf, copy into subsec_sizes
    sdcard.read(buf, MAX_CONFIG_PACKETS);
    memcpy(subsec_sizes, buf, MAX_CONFIG_PACKETS);

    sdcard.close();
    return 1;
}

void ConfigLayer::store_config(){
    int config_byte_size = MAX_CONFIG_PACKETS * sizeof(CommsPacket);
    uint8_t buf[config_byte_size];
    if(sdcard.exists("/.config")) sdcard.rm("/.config");
    sdcard.touch("/.config");
    sdcard.open("/.config");

    // copy contents of config packets into byte buffer, then write
    memcpy(buf, config_packets, config_byte_size);
    sdcard.write(buf, config_byte_size);

    // copy contents of subsec_sizes into byte buffer, then write
    memcpy(buf, subsec_sizes, MAX_CONFIG_PACKETS);
    sdcard.write(buf, MAX_CONFIG_PACKETS);
    
    sdcard.close();
}