// Sensor.hpp

#ifndef SENSOR_HPP
#define SENSOR_HPP

#include "sensor_constants.hpp"

/// @brief Base class for all sensors.
class Sensor {
public:
    /// @brief Constructor to initialize the sensor with a specific type.
    /// @param type The type of the sensor.
    Sensor(SensorType type) : type_(type), id_(next_id_++) { };

    /// @brief Constructor to initialize the sensor with a specific type and ID.
    /// @param type The type of the sensor.
    /// @param id The ID of the sensor.
    Sensor(SensorType type, uint8_t id) : type_(type), id_(id) { };

    /// @brief Virtual destructor for the sensor.
    virtual ~Sensor() = default;

    /// @brief Get the type of the sensor.
    /// @return The type of the sensor.
    SensorType getType() const { return type_; }

    /// @brief Get the ID of the sensor.
    /// @return The ID of the sensor.
    uint8_t getId() const { return id_; }

    /// @brief Set the ID of the sensor.
    /// @param id The new ID of the sensor.
    void setId(uint8_t id) { id_ = id; }

    /// @brief Read the sensor data.
    /// @return true if successful, false if no data available
    virtual bool read() = 0;

protected:

    ///The type of the sensor.
    SensorType type_;
    ///The ID of the sensor.
    uint8_t id_;

private:
     /// Static variable to generate unique IDs for sensors.
    static uint8_t next_id_;
};

#endif // SENSOR_HPP
