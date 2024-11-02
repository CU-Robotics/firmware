#include "config_layer.hpp"

const Config* const ConfigLayer::configure(HIDLayer* comms) {
    // check SD
    while(ref.ref_data.robot_performance.robot_ID != 0) ref.read();

    if(sdcard.exists("/config.pack")){
        #ifdef CONFIG_LAYER_DEBUG
        Serial.printf("Config located on SD in /config.pack, attempting to load from file\n");
        #endif
        
        // load sd config into config_packets
        configured = sd_load();
        if(configured) Serial.printf("SD load successful!\n");
    }
    #ifdef CONFIG_LAYER_DEBUG
    else{
        Serial.printf("No config packet located, awaiting input from comms....\n");
    }
    #endif
    // if no config on SD, then await transmission
    // grab and process all config packets until finished
    double prev_time = millis();
    double delta_time = 0;
    while (!is_configured()) {
        #ifdef CONFIG_LAYER_DEBUG
        if(delta_time >= 2000){
            Serial.printf("Pinging for config packet....\n");
            prev_time = millis();
        }
        delta_time = millis() - prev_time;
        #endif

        comms->ping();
        process(comms->get_incoming_packet(), comms->get_outgoing_packet());
    }

    // put the data from the packets into the config object
    config.fill_data(config_packets, subsec_sizes);

    // update stored config, repeat until successful
    do {
        Serial.println("Attempting to store config...");
    } while(!store_config());

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

    if(sdcard.open("/config.pack") != 0) return false;


    // DEBUG: we are expecting this to run after hive sends a packet. compare results (should be ***identical***!!!)
    uint8_t received_id;
    sdcard.read(&received_id, 1);
    if(ref.ref_data.robot_performance.robot_ID != received_id){
        Serial.printf("NOTICE: attempting to load firmware for different robot type! \n");
        Serial.printf("Current robot ID: %d\nStored config robot ID: %d\n", ref.ref_data.robot_performance.robot_ID, received_id);
        return false;
    }
    
    // read checksum and each packet
    // grab first packet and checksum (kept separate in order to validate subsequent checksum values)
    uint64_t checksum;
    if(sdcard.read((uint8_t *)(&checksum), sizeof(uint64_t)) != sizeof(uint64_t)) return false;
    if(sdcard.read((uint8_t *)(&config_packets[0]), sizeof(CommsPacket)) != sizeof(CommsPacket)) return false;
    // read remaining packets
    for(int i = 1; i < MAX_CONFIG_PACKETS; i++){
        uint64_t temp_checksum;
        if(sdcard.read((uint8_t *)(&temp_checksum), sizeof(uint64_t)) != sizeof(uint64_t)) {
            Serial.printf("Unexpected mismatch in number of bytes read, requesting config from hive...\n");
            return false;
        }
        if(temp_checksum != checksum){
            Serial.printf("Inconsistent data found in config file, requesting config from hive...\n");
            return false;
        }
        if(sdcard.read((uint8_t *)(&config_packets[i]), sizeof(CommsPacket)) != sizeof(CommsPacket)){
            Serial.printf("Unexpected mismatch in number of bytes read, requesting config from hive...\n");
            return false;
        }
    }
    if(sdcard.read(subsec_sizes, MAX_CONFIG_PACKETS) != MAX_CONFIG_PACKETS){
        Serial.printf("Unexpected mismatch in number of bytes read, requesting config from hive...\n");
        return false;
    }

    sdcard.close();

    uint8_t *config_bytes = (uint8_t *)config_packets;
    #ifdef CONFIG_LAYER_DEBUG
    Serial.printf("sd_load: computing checksum for stored config\n");
    #endif
    if(sd_checksum64(config_bytes, config_byte_size) != checksum){
        Serial.printf("Checksum for config file does not match stored config, requesting config from hive...\n");
        return false;
    }

    return true;
}

bool ConfigLayer::store_config(){
    // initialize: erase config.pack if exists, reopen
    // check return values of everything!!!
    if(sdcard.exists("/config.pack")) {
        if(sdcard.rm("/config.pack") != 0){
            if(!SD_ERR_HANDLER(CONFIG_RM_FAIL)) return false;
        }
    }
    if(sdcard.touch("/config.pack") != 0){
        if(!SD_ERR_HANDLER(CONFIG_TOUCH_FAIL)) return false;
    }
    if(sdcard.open("/config.pack") != 0){
        if(!SD_ERR_HANDLER(CONFIG_OPEN_FAIL)) return false;
    }

    // total size of /config.pack: MAX_CONFIG_PACKETS * (sizeof(CommsPacket) + 1)
    // num of packets * size of each packet == num of bytes for all packets
    int config_byte_size = MAX_CONFIG_PACKETS * sizeof(CommsPacket);
    
    // calculate checksum on config packet array
    #ifdef CONFIG_LAYER_DEBUG
    Serial.printf("store_config: computing checksum for received config\n");
    #endif
    uint8_t* config_bytes = (uint8_t *)config_packets;
    uint64_t checksum = sd_checksum64(config_bytes, config_byte_size);

    // write robot id byte to ref data so it can be compared directly
    sdcard.write(&ref.ref_data.robot_performance.robot_ID, 1);

    for(int i = 0; i < MAX_CONFIG_PACKETS; i++){
        // write checksum once
        sdcard.write((uint8_t *)(&checksum), sizeof(uint64_t)); // reinterpret addr of checksum as byte array
        // write one packet
        sdcard.write((uint8_t *)(&config_packets[i]), sizeof(CommsPacket));
    }
    sdcard.write(subsec_sizes, MAX_CONFIG_PACKETS);

    sdcard.close();

    Serial.printf("Finished writing to SD\n");

    return true;
}

uint64_t ConfigLayer::sd_checksum64(uint8_t* arr, uint64_t n){
    uint64_t out = 0;
    for(uint32_t i = 0; i < n; i++){
        out += arr[i];
    }
    #ifdef CONFIG_LAYER_DEBUG
    Serial.printf("sd_checksum64: returned %x\n", out);
    #endif
    return out; 
}

bool ConfigLayer::SD_ERR_HANDLER(int err_code){
    if(err_code == CONFIG_RM_FAIL){
        Serial.println("CONFIG_ERROR::config failed to remove /config.pack from SD");
        return false;
    }
    if(err_code == CONFIG_TOUCH_FAIL){
        Serial.println("CONFIG_ERROR::config failed to create /config.pack on SD");
        return false;
    }
    if(err_code == CONFIG_OPEN_FAIL){
        Serial.println("CONFIG_ERROR::config failed to open /config.pack from SD");
        return false;
    }

    // default case, in the event that invalid err_code passed
    Serial.printf("CONFIG_SD_ERR_HANDLER::invalid err_code (%d) passed\n", err_code);
    return false;
}