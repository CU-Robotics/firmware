#include "config_layer.hpp"

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