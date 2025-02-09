#pragma once

#include "../comms_layer.hpp"
#include "comms_data.hpp"
#include "modules/hive/environment.hpp"

namespace Comms {

/// @brief Holds an object derived from CommsData, to allow it to easily be sent using the internal send_to_comms() method.
/// @tparam T Type of data being sent over comms
template <typename T>
class Container {
    static_assert < (std::is_base_of<CommsData, T>::value, "The datatype stored in Comms::Container must be a subtype of CommsData");
public:
    /// @brief Sends the internal data type "data" over the PhysicalMedium medium.
    /// @param medium Physical medium for data to be sent over
    void send_to_comms(PhysicalMedium medium) {
        T* send_ptr = new T;
        memcpy(send_ptr, &data, sizeof(T));

        // TODO: how do we access the comms_layer object in a way that makes sense...?
        // comms_layer->send(send_ptr, PhysicalMedium::Ethernet);
    }

    /// @brief Data object to be sent over comms
    T data;
};

};