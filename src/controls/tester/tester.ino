#include "rm_CAN.hpp"
#include "dr16.hpp"
#include "controllers.hpp"


rm_CAN can;
DR16 dr16;
Controllers control(&can, &dr16);


void setup() {
    Serial.begin(9600);

    can.init();
    dr16.init();
}

void loop() {

    dr16.read();

    // if info from the remote is not being detected
    // or if safety switch is on, don't write anything
    if (!dr16.is_connected() || dr16.get_l_switch() == 1) {
        Serial.println("SAFETY: ON");

        control.zero_speeds();
        can.zero();
        can.zero_motors();
        can.write();
    } else {
        Serial.println("SAFETY: OFF");
        while (can.read());

        // here is where the main driver code will be

        control.update();

        can.write();
    }
}