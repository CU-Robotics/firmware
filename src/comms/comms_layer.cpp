#include "comms_layer.hpp"

namespace Comms {


//
// CommsLayer PUBLIC definitions
//

// - init functions

int CommsLayer::init() {
    sequence = 0;
    bool success = Ethernet.begin();
    if (success) {
        Serial.printf("Ethernet socket online\n");
    } else {
        Serial.printf("Ethernet FAILED to initialize, exiting...\n");
        return -1;
    }

    return 0;
}

int CommsLayer::loop() {
    Ethernet.loop();
    
    return 0;
}

void CommsLayer::send(CommsData* data, PhysicalMedium medium) {
    
}

HiveData CommsLayer::receive(PhysicalMedium medium) {
    // todo rest of this
    return HiveData();
}



//
// CommsLayer PRIVATE definitions
//




} // namespace Comms    