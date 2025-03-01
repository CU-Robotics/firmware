#include "config_layer.hpp"

/// @brief This resets the whole processor and kicks it back to program entry (teensy4/startup.c)
/// @param void specify no arguments (needed in C)
/// @note Dont abuse this function, it is not to be used lightly
extern "C" void reset_teensy(void) {
    // Register information found in the NXP IM.XRT 1060 reference manual
    SRC_GPR5 = 0x0BAD00F1;
    // Register information found in the Arm-v7-m reference manual
    SCB_AIRCR = 0x05FA0004;
    // loop to catch execution while the reset occurs
    while (1);
}

const Config* const ConfigLayer::configure(HIDLayer* comms, bool config_off_SD) {
    if (config_off_SD) {
        // attempt SD card load
        config_SD_init(comms);
        if (configured) return &config;
    }

    // if no config on SD, then await transmission
    // grab and process all config packets until finished
    uint32_t prev_time = millis();
    uint32_t delta_time = 0;
    while (!is_configured()) {
    #ifdef CONFIG_LAYER_DEBUG
        if (delta_time >= 2000) {
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

    // verify that config received matches ref system: if not, error out
#ifndef DISABLE_REF_CONFIG_SAFETY_CHECK
    Serial.printf("Received robot ID from config: %d\nRobot ID from ref system: %d\n", (int)config.robot, ref->ref_data.robot_performance.robot_ID);
    // id check with modulo 100 to account for red and blue teams. Blue is the id + 100. (ID == 101, 102, ...)
    if ((ref->ref_data.robot_performance.robot_ID % 100) != (int)config.robot) {
        Serial.printf("ERROR: IDs do not match!! Check robot_id.cfg and robot settings from ref system!\n");
        if (!CONFIG_ERR_HANDLER(CONFIG_ID_MISMATCH)) {
            // in current implementation, CONFIG_ERR_HANDLER w/ err code CONFIG_ID_MISMATCH will
            // enter an infinite while(1) loop-- if that is changed, this loop should be changed accordingly as well
            while (1) {
                Serial.println("CONFIG_ERR_HANDLER: exited with error code CONFIG_ID_MISMATCH");
                Serial.println("if function was modified for case CONFIG_ID_MISMATCH, remove this loop in config_layer.cpp");
                delay(2000);
            }
        }
    }
#endif

    // update stored config, msg if successful
    Serial.println("Attempting to store config...");
    if (store_config()) Serial.println("\tConfig successfully stored in /config.pack\n");
    else Serial.println("\tConfig not successfully stored (is an SD card inserted?)\n");    // not a fatal error, can still return

    // blink 4 times quickly to show that we received a config packet finished processing it
    for (int i = 0; i < 8; i++) {
        digitalToggle(13);
        delay(100);
    }

    // get the outgoing packet and set it to 0
    CommsPacket* outgoing = comms->get_outgoing_packet();
    memset(outgoing->raw, 0, sizeof(outgoing->raw));

    // Hive marks the config process as done once teensy sends a packet with info_bit == 0
    // Hive then sends a packet back with info_bit == 0 as well, so ping until we get a non-config back
    while (comms->get_incoming_packet()->raw[3] == 1) {
        comms->ping();
    }

    // update the num_of_(sensor) variables in the config struct
    return &config;
}

void ConfigLayer::reconfigure(HIDLayer* comms) {
    // force it to reconfigure
    configured = false;

    // reset the section, subsection, and index vars to defaults since we need to use process() again
    seek_sec = -1;
    seek_subsec = 0;
    index = 0;

    // perform a normal config, but dont try to load from the config, just process and store
    configure(comms, false);

    // issue the reboot call
    reset_teensy();

    // reset_teensy() never returns
    __builtin_unreachable();
}

void ConfigLayer::config_SD_init(HIDLayer* comms) {
    // if on robot, we need to wait for ref to send robot_id
#ifndef DISABLE_REF_CONFIG_SAFETY_CHECK
    Serial.println("Waiting for ref system to initialize...");
    while (ref->ref_data.robot_performance.robot_ID == 0)
        ref->read();
    Serial.println("Ref system online");
#endif


// check SD
    if (sdcard.exists(CONFIG_PATH)) {
        Serial.printf("Config located on SD in /config.pack, attempting to load from file\n");

        // load sd config into config_packets
        configured = sd_load();
        if (configured) {
            Serial.printf("SD config load successful!\n");
        } else {
            Serial.printf("SD config load failed, awaiting input from comms....\n");
        }
    } else {
        Serial.printf("No %s in SD card, awaiting input from comms....\n", CONFIG_PATH);
    }
}

void ConfigLayer::process(CommsPacket* in, CommsPacket* out) {
    char* in_raw = in->raw;
    char* out_raw = out->raw;
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

    }
}

void Config::fill_data(CommsPacket packets[MAX_CONFIG_PACKETS], uint8_t sizes[MAX_CONFIG_PACKETS]) {
    for (int i = 0; i < MAX_CONFIG_PACKETS; i++) {
        uint8_t id = packets[i].get_id();
        uint8_t subsec_id = *reinterpret_cast<uint8_t*>(packets[i].raw + 2);
        uint16_t sub_size = *reinterpret_cast<uint16_t*>(packets[i].raw + 6);

        if (subsec_id == 0)
            index = 0;

        if (sub_size != 0) {
            Serial.printf("id: %d, subsec_id: %d, sub_size: %d\n", id, subsec_id, sub_size);
        }

        if (id == yaml_section_id_mappings.at("robot")) {
            memcpy(&robot, packets[i].raw + 8, sub_size);
            index += sub_size;
        }

        if (id == yaml_section_id_mappings.at("motor_info")) {

            size_t linear_index = index / sizeof(float);
            size_t i1 = linear_index / (CAN_MAX_MOTORS);
            size_t i2 = linear_index % CAN_MAX_MOTORS;
            memcpy(&motor_info[i1][i2], packets[i].raw + 8, sub_size);
            index += sub_size;
        }

        if (id == yaml_section_id_mappings.at("sensor_info")) {
            size_t linear_index = index / sizeof(float);
            size_t i1 = linear_index / (NUM_SENSORS);
            size_t i2 = linear_index % NUM_SENSORS;
            memcpy(&sensor_info[i1][i2], packets[i].raw + 8, sub_size);
            index += sub_size;
        }

        if (id == yaml_section_id_mappings.at("gains")) {
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
        }
        if (id == yaml_section_id_mappings.at("controller_info")) {
            size_t linear_index = index / sizeof(float);
            size_t i1 = linear_index / (CAN_MAX_MOTORS);
            size_t i2 = linear_index % CAN_MAX_MOTORS;
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

    Serial.println();


    //fill num_of_(sensor) variables with the number of sensors defined for this robot
    for (int i = 0; i < 16; i++) {
        // if the sensor at index i is not -1
        if (sensor_info[i][0] != -1.f) {
            switch ((int)sensor_info[i][0]) {
            case 0:
                this->num_of_buffEnc++;
                break;
            case 1:
                this->num_of_revEnc++;
                break;
            case 2:
                this->num_of_icm++;
                break;
            case 3:
                this->num_of_tof++;
                break;
            case 4:
                this->num_of_lidar++;
                break;
            case 5:
                this->num_of_realsense++;
            default:
                break;
            }
        }
    }
}

void Config::print() const {
    Serial.printf("Config:\n");
    Serial.printf("Robot ID: %.3f\n", robot);
    Serial.printf("Controller Info:\n");
    for (size_t i = 0; i < NUM_ROBOT_CONTROLLERS; i++) {
        Serial.printf("\tController %d: ", i);
        for (size_t j = 0; j < CAN_MAX_MOTORS; j++) {
            Serial.printf("%.3f ", controller_info[i][j]);
        }
        Serial.println();
    }
    // gains
    Serial.printf("Gains:\n");
    for (size_t i = 0; i < NUM_ROBOT_CONTROLLERS; i++) {
        Serial.printf("\tController %d: ", i);
        for (size_t j = 0; j < NUM_GAINS; j++) {
            Serial.printf("%.3f ", gains[i][j]);
        }
        Serial.println();
    }

    // gear ratios
    Serial.printf("Gear Ratios:\n");
    for (size_t i = 0; i < NUM_ROBOT_CONTROLLERS; i++) {
        Serial.printf("\tController %d: ", i);
        for (size_t j = 0; j < CAN_MAX_MOTORS; j++) {
            Serial.printf("%.3f ", gear_ratios[i][j]);
        }
        Serial.println();
    }

    // sensor info
    Serial.printf("Sensor Info:\n");
    for (size_t i = 0; i < NUM_SENSORS; i++) {
        Serial.printf("\tSensor %d: ", i);
        for (size_t j = 0; j < NUM_SENSOR_VALUES; j++) {
            Serial.printf("%.3f ", sensor_info[i][j]);
        }
        Serial.println();
    }

    // estimator info
    Serial.printf("Estimator Info:\n");
    for (size_t i = 0; i < NUM_ESTIMATORS; i++) {
        Serial.printf("\tEstimator %d: ", i);
        for (size_t j = 0; j < STATE_LEN; j++) {
            Serial.printf("%.3f ", estimator_info[i][j]);
        }
        Serial.println();
    }

    // governor types
    Serial.printf("Governor Types:\n");
    for (size_t i = 0; i < STATE_LEN; i++) {
        Serial.printf("\tState %d: %.3f\n", i, governor_types[i]);
    }

    // motor info
    Serial.printf("Motor Info:\n");
    for (size_t i = 0; i < CAN_MAX_MOTORS; i++) {
        Serial.printf("\tMotor %d: ", i);
        for (size_t j = 0; j < 3; j++) {
            Serial.printf("%.3f ", motor_info[i][j]);
        }
        Serial.println();
    }
}

bool ConfigLayer::sd_load() {
    // total size of /config.pack: MAX_CONFIG_PACKETS * (sizeof(CommsPacket) + 1)
    // num of packets * size of each packet == num of bytes for all packets
    const int config_byte_size = MAX_CONFIG_PACKETS * sizeof(CommsPacket);

    if (sdcard.open(CONFIG_PATH) != 0)
        return false;

    // grab ID from config (originally from hive). should match ID from ref system- if not, abort!!
    float received_id;
    sdcard.read((uint8_t*)(&received_id), sizeof(float));

    // checksum is passed by reference and written to for later reference
    uint64_t checksum;
    if (!config_SD_read_packets(checksum))
        return false;

    sdcard.close();

    uint8_t* config_bytes = (uint8_t*)config_packets;
#ifdef CONFIG_LAYER_DEBUG
    Serial.printf("sd_load: computing checksum for stored config\n");
#endif
    // need combined checksum of config_bytes, subsec_sizes
    if ((sd_checksum64(config_bytes, config_byte_size) + sd_checksum64(subsec_sizes, MAX_CONFIG_PACKETS)) != checksum) {
        Serial.printf("Checksum for config file does not match stored config, requesting config from hive...\n");
        return false;
    }

    // fill_data fills a Config object using info from config_packets and subsec_sizes
    // we want to make sure requesting data from ref is reliable, so we load the config
    // if the ID from packets does not match ref, we need to get rid of the config- we use a temp config that we can throw out
    // set config = temp_config if IDs match
    Config temp_config;

    temp_config.fill_data(config_packets, subsec_sizes);

#ifndef DISABLE_REF_CONFIG_SAFETY_CHECK
    // id check with modulo 100 to account for red and blue teams. Blue is the id + 100. (ID == 101, 102, ...)
    if ((ref->ref_data.robot_performance.robot_ID % 100) != (int)received_id) {
        Serial.printf("NOTICE: attempting to load firmware for different robot type! \n");
        Serial.printf("Current robot ID: %d\nStored config robot ID: %d\n", ref->ref_data.robot_performance.robot_ID, (int)received_id);
        Serial.println("Requesting config from hive...");
        return false;
    }
#endif

    config = temp_config;

    return true;
}

bool ConfigLayer::config_SD_read_packets(uint64_t& checksum) {
    // read checksum and each packet
    // grab first packet and checksum (kept separate in order to validate subsequent checksum values)
    if (sdcard.read((uint8_t*)(&checksum), sizeof(uint64_t)) != sizeof(uint64_t))
        return false;
    if (sdcard.read((uint8_t*)(&config_packets[0]), sizeof(CommsPacket)) != sizeof(CommsPacket))
        return false;
    // read remaining packets
    for (int i = 1; i < MAX_CONFIG_PACKETS; i++) {
        uint64_t temp_checksum;
        if (sdcard.read((uint8_t*)(&temp_checksum), sizeof(uint64_t)) != sizeof(uint64_t)) {
            Serial.printf("Unexpected mismatch in number of bytes read, requesting config from hive...\n");
            return false;
        }
        if (temp_checksum != checksum) {
            Serial.printf("Inconsistent data found in config file, requesting config from hive...\n");
            return false;
        }
        if (sdcard.read((uint8_t*)(&config_packets[i]), sizeof(CommsPacket)) != sizeof(CommsPacket)) {
            Serial.printf("Unexpected mismatch in number of bytes read, requesting config from hive...\n");
            return false;
        }
    }
    if (sdcard.read(subsec_sizes, MAX_CONFIG_PACKETS) != MAX_CONFIG_PACKETS) {
        Serial.printf("Unexpected mismatch in number of bytes read, requesting config from hive...\n");
        return false;
    }

    return true;
}

bool ConfigLayer::store_config() {
    // total size of /config.pack: MAX_CONFIG_PACKETS * (sizeof(CommsPacket) + 1)
    // num of packets * size of each packet == num of bytes for all packets
    const int config_byte_size = MAX_CONFIG_PACKETS * sizeof(CommsPacket);
    const char* config_path = "/config.pack";

    // initialize: erase config.pack if exists, reopen
    // check return values of everything!!!
    if (sdcard.exists(config_path)) {
        if (sdcard.rm(config_path) != 0) {
            if (!CONFIG_ERR_HANDLER(CONFIG_RM_FAIL))
                return false;
        }
    }
    if (sdcard.touch(config_path) != 0) {
        if (!CONFIG_ERR_HANDLER(CONFIG_TOUCH_FAIL))
            return false;
    }
    if (sdcard.open(config_path) != 0) {
        if (!CONFIG_ERR_HANDLER(CONFIG_OPEN_FAIL))
            return false;
    }

    // calculate checksum on config packet array
#ifdef CONFIG_LAYER_DEBUG
    Serial.printf("store_config: computing checksum for received config\n");
#endif
    uint8_t* config_bytes = (uint8_t*)config_packets;
    uint64_t checksum = sd_checksum64(config_bytes, config_byte_size);
    checksum += sd_checksum64(subsec_sizes, MAX_CONFIG_PACKETS);

    // write robot id byte to ref data so it can be compared directly
    sdcard.write((uint8_t*)(&config.robot), sizeof(float));

    Serial.printf("Robot ID from config: %d\n", (int)config.robot);

    for (int i = 0; i < MAX_CONFIG_PACKETS; i++) {
        // write checksum once
        sdcard.write((uint8_t*)(&checksum), sizeof(uint64_t)); // reinterpret addr of checksum as byte array
        // write one packet
        sdcard.write((uint8_t*)(&config_packets[i]), sizeof(CommsPacket));
    }
    sdcard.write(subsec_sizes, MAX_CONFIG_PACKETS);

    sdcard.close();

    Serial.printf("Finished writing to SD\n");

    return true;
}

uint64_t ConfigLayer::sd_checksum64(uint8_t* arr, uint64_t n) {
    uint64_t out = 0;
    for (uint32_t i = 0; i < n; i++) {
        out += arr[i];
    }
#ifdef CONFIG_LAYER_DEBUG
    Serial.printf("sd_checksum64: returned %x\n", out);
#endif
    return out;
}

bool ConfigLayer::CONFIG_ERR_HANDLER(int err_code) {
    // vars that may be used by err handler procedures
    uint32_t prev_time, delta_time;

    // make sure that every error case returns a value!
    switch (err_code) {
    case CONFIG_RM_FAIL:
        Serial.println("\tCONFIG_ERROR::config failed to remove /config.pack from SD");
        return false;

    case CONFIG_TOUCH_FAIL:
        Serial.println("\tCONFIG_ERROR::config failed to create /config.pack on SD");
        return false;

    case CONFIG_OPEN_FAIL:
        Serial.println("\tCONFIG_ERROR::config failed to open /config.pack from SD");
        return false;

    case CONFIG_ID_MISMATCH:
        prev_time = millis();
        delta_time = millis() - prev_time;
        while (1) {
            if (delta_time >= 2000) {
                Serial.println("\tCONFIG_ERROR::config from comms does not match from ref system!!");
                Serial.println("\tCheck robot_id.cfg and robot ID on ref system for inconsistency");
                prev_time = millis();
            }
            delta_time = millis() - prev_time;
        }
        return false;

    default:
        // default case, in the event that invalid err_code passed
        Serial.printf("\tCONFIG_ERR_HANDLER::invalid err_code (%d) passed\n", err_code);
        return false;
    }
}