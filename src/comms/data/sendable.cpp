#include "sendable.hpp"

#if defined(HIVE)

TEST_CASE("Sendable constructors") {
    TestData data;
    data.x = 1.5f;
    data.y = 2.5f;
    data.z = 3.5f;
    data.w = 0x12345678;

    // Constructor with data
    Comms::Sendable<TestData> test_data(data);
    CHECK(test_data.data.x == 1.5f);
    CHECK(test_data.data.y == 2.5f);
    CHECK(test_data.data.z == 3.5f);
    CHECK(test_data.data.w == 0x12345678);

    // assignment operator and default constructor
    Comms::Sendable<TestData> test_data2;
    test_data2 = data;
    CHECK(test_data2.data.x == 1.5f);
    CHECK(test_data2.data.y == 2.5f);
    CHECK(test_data2.data.z == 3.5f);
    CHECK(test_data2.data.w == 0x12345678);

    // implicit conversion
    Comms::Sendable<TestData> test_data3 = data;
    CHECK(test_data3.data.x == 1.5f);
    CHECK(test_data3.data.y == 2.5f);
    CHECK(test_data3.data.z == 3.5f);
    CHECK(test_data3.data.w == 0x12345678);
}

// TEST_CASE("Testing Sendable") {
//     Hive::RobotInfo robot_info;
//     robot_info.init();
//     robot_info.load_info();

//     CommsLayer comms_layer;
//     comms_layer.init();

//     TestData data;
//     data.x = 1.5f;
//     data.y = 2.5f;
//     data.z = 3.5f;
//     data.w = 0x12345678;

//     Sendable<TestData> test_data;
//     test_data = data;

//     SUBCASE("Requesting to send data that will fit in a single packet") {
//         SUBCASE("Will receive data after sending only one Sendable") {
//             test_data.send_to_comms();
//             // TODO simulate the CommsLayer send, and check it was received.
//         }
//         SUBCASE("Will receive data after sending two Sendables") {
//             test_data.send_to_comms();
            
//             TestData data2; // TODO write test helpers to easily generate lots of unique test data
//             data2.x = 2.5f;
//             data2.y = 3.5f;
//             data2.z = 4.5f;
//             data2.w = 0xABCDEFAB;
        
//             Sendable<TestData> test_data2;
//             test_data2 = data2;

//             test_data2.send_to_comms();

//             // TODO simulate the CommsLayer send, and check it was received.
//         } 
//     }

//     SUBCASE("Requesting to send more data than can fit in a single packet will still eventually send") {
//         SUBCASE("Testing sending data that will fit in two packets") {
//             // TODO simulate the CommsLayer send, and check it was received.

//         }
//         SUBCASE("Testing sending data that will fit in three packets") {
//             // TODO simulate the CommsLayer send, and check it was received.
//         }
//         SUBCASE("Testing sending data that will fit in N packets") {
//             // TODO simulate the CommsLayer send, and check it was received.
//         }
//     }
// }

// TEST_CASE("A Sendable that goes out of scope doesn't lose its reference or data") {
//     Hive::RobotInfo robot_info;
//     robot_info.init();
//     robot_info.load_info();

//     CommsLayer comms_layer;
//     comms_layer.init();

//     {
//         TestData data;
//         data.x = 1.5f;
//         data.y = 2.5f;
//         data.z = 3.5f;
//         data.w = 0x12345678;
    
//         Sendable<TestData> test_data;
//         test_data = data;

//         test_data.send_to_comms();
//     }

//     // TODO simulate the send while test_data no longer exists, and check the data was received in a simulated send.
// }


#endif  // HIVE