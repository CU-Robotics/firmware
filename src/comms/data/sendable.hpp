#pragma once

#include <type_traits>                          // for is_base_of, is_copy_constructible

#if defined(HIVE)
#include "modules/hive/environment.hpp"         // for Hive::env
#include "modules/comms/comms_layer.hpp"        // for CommsLayer
#include "modules/comms/data/firmware_data.hpp" // for TestData
#include "modules/hive/robot_info.hpp"          // for Hive::RobotInfo
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

#if defined(HIVE)

TEST_CASE("Testing Sendable") {
    Hive::RobotInfo robot_info;
    robot_info.init();
    robot_info.load_info();

    CommsLayer comms_layer;
    comms_layer.init();

    TestData data;
    data.x = 1.5f;
    data.y = 2.5f;
    data.z = 3.5f;
    data.w = 0x12345678;

    Sendable<TestData> test_data;
    test_data = data;

    SUBCASE("Requesting to send data that will fit in a single packet") {
        SUBCASE("Will receive data after sending only one Sendable") {
            test_data.send_to_comms();
            // TODO simulate the CommsLayer send, and check it was received.
        }
        SUBCASE("Will receive data after sending two Sendables") {
            test_data.send_to_comms();
            
            TestData data2; // TODO write test helpers to easily generate lots of unique test data
            data2.x = 2.5f;
            data2.y = 3.5f;
            data2.z = 4.5f;
            data2.w = 0xABCDEFAB;
        
            Sendable<TestData> test_data2;
            test_data2 = data2;

            test_data2.send_to_comms();

            // TODO simulate the CommsLayer send, and check it was received.
        } 
    }

    SUBCASE("Requesting to send more data than can fit in a single packet will still eventually send") {
        SUBCASE("Testing sending data that will fit in two packets") {
            // TODO simulate the CommsLayer send, and check it was received.

        }
        SUBCASE("Testing sending data that will fit in three packets") {
            // TODO simulate the CommsLayer send, and check it was received.
        }
        SUBCASE("Testing sending data that will fit in N packets") {
            // TODO simulate the CommsLayer send, and check it was received.
        }
    }
}

TEST_CASE("A Sendable that goes out of scope doesn't lose its reference or data") {
    Hive::RobotInfo robot_info;
    robot_info.init();
    robot_info.load_info();

    CommsLayer comms_layer;
    comms_layer.init();

    {
        TestData data;
        data.x = 1.5f;
        data.y = 2.5f;
        data.z = 3.5f;
        data.w = 0x12345678;
    
        Sendable<TestData> test_data;
        test_data = data;

        test_data.send_to_comms();
    }

    // TODO simulate the send while test_data no longer exists, and check the data was received in a simulated send.
}

#endif  // HIVE

}   // namespace Comms