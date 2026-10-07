#pragma once

// Keep the host test independent of unrelated robot controls and Teensy headers.
// The normal firmware build checks against the real Sensor interface.
class Sensor {
public:
    virtual void init() = 0;
    virtual void read() = 0;
    virtual void send_to_comms() const = 0;
};
