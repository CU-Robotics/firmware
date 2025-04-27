#pragma once

#include <type_traits>                          // for is_base_of, is_copy_constructible

#if defined(HIVE)
#include "modules/hive/environment.hpp"         // for Hive::env
#include "modules/comms/comms_layer.hpp"        // for CommsLayer
#elif defined(FIRMWARE)
#include "comms_layer.hpp"                      // for CommsLayer
#endif

namespace Comms {

// Forward declaration since CommsLayer is required in Sendable.
struct CommsData;
    
/// @brief Sendable is a wrapper around a CommsData struct that allows it to be sent over comms.
/// @tparam T The type of data to be sent.
template <typename T> 
struct Sendable {
    static_assert(std::is_base_of<CommsData, T>::value, "The datatype stored in Comms::Container must be a subtype of CommsData");
    static_assert(std::is_copy_constructible<T>::value, "The datatype stored in Comms::Container must be copy constructible");
public:
    /// @brief Default constructor.
    Sendable() = default;

    /// @brief Implicit constructor.
    /// @param data The data to be sent.
    Sendable(const T& data) : data(data) {}

    /// @brief Add this Sendable to the comms layer to be sent.
    void send_to_comms() {
    #if defined(HIVE)
        Hive::env->comms_layer->queue_data(new T(data)); 
    #elif defined(FIRMWARE)
        comms_layer.queue_data(new T(data));
    #endif
    }

    /// @brief Copy assignment operator. Allows easier creation of Sendables.
    /// @param rhs The data to copy into this Sendable. This is implicitly passed in.
    /// @return A reference to this Sendable.
    Sendable<T>& operator=(const T& rhs) {
        data = rhs;
        return *this;
    }

public:
    /// @brief The data to be sent over comms.
    T data;
};

}   // namespace Comms