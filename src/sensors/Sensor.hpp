// Sensor.hpp

#ifndef SENSOR_HPP
#define SENSOR_HPP

#include "constants.hpp"

class Sensor {
public:
    Sensor(SensorType type) : type_(type), id_(next_id_++) {}
    virtual ~Sensor() = default;

    SensorType getType() const { return type_; }
    uint8_t getId() const { return id_; }

    virtual void serialize(uint8_t* buffer, size_t& offset) const = 0;
    virtual void deserialize(const uint8_t* data, size_t& offset) = 0;

protected:
    SensorType type_;
    uint8_t id_;

private:
 static uint8_t next_id_;
};

#endif // SENSOR_HPP
