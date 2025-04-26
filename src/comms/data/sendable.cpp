#include "sendable.hpp"

#if defined(HIVE)
#include "modules/comms/data/data_structs.hpp"
#include "modules/comms/hid_packet.hpp"

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

TEST_CASE("Sendable sends data to CommsLayer") {
    TestData data;
    data.x = 1.5f;
    data.y = 2.5f;
    data.z = 3.5f;
    data.w = 0x12345678;

    Comms::CommsLayer comms_layer;
    comms_layer.init_without_physical_layers();

    Comms::Sendable<TestData> test_data(data);
    test_data.send_to_comms();

    comms_layer.send_packets();
    comms_layer.set_hid_incoming(comms_layer.get_hid_outgoing());
    comms_layer.set_ethernet_incoming(comms_layer.get_ethernet_outgoing());
    comms_layer.receive_packets();

    REQUIRE(comms_layer.get_firmware_data().test_data.x == data.x);
    REQUIRE(comms_layer.get_firmware_data().test_data.y == data.y);
    REQUIRE(comms_layer.get_firmware_data().test_data.z == data.z);
    REQUIRE(comms_layer.get_firmware_data().test_data.w == data.w);

    comms_layer.stop();
}

TEST_CASE("Sending too much data to fit in one packet") {
    Comms::CommsLayer comms_layer;
    comms_layer.init_without_physical_layers();

    int count = 10;

    // Simulate sending too much data
    for (int i = 0; i < count; ++i) {
        BigTestData data;
        data.blah[0] = i;
        Comms::Sendable<BigTestData> test_data(data);
        test_data.send_to_comms();
    }

    // No more than 1 packet can fit in an HID packet, validates this test of checking one after another packet
    REQUIRE(sizeof(BigTestData) > Comms::HID_PACKET_PAYLOAD_SIZE / 2);

    comms_layer.send_packets();
    comms_layer.set_hid_incoming(comms_layer.get_hid_outgoing());
    comms_layer.set_ethernet_incoming(comms_layer.get_ethernet_outgoing());
    comms_layer.receive_packets();

    // The first possible big test data should be the one that was sent first
    REQUIRE(comms_layer.get_firmware_data().big_test_data.blah[0] == 0);

    comms_layer.send_packets();
    comms_layer.set_hid_incoming(comms_layer.get_hid_outgoing());
    comms_layer.set_ethernet_incoming(comms_layer.get_ethernet_outgoing());
    comms_layer.receive_packets();

    // then the second
    REQUIRE(comms_layer.get_firmware_data().big_test_data.blah[0] == 1);

    // then the rest
    for (int i = 2; i < count; ++i) {
        comms_layer.send_packets();
        comms_layer.set_hid_incoming(comms_layer.get_hid_outgoing());
        comms_layer.set_ethernet_incoming(comms_layer.get_ethernet_outgoing());
        comms_layer.receive_packets();
        REQUIRE(comms_layer.get_firmware_data().big_test_data.blah[0] == i);
    }

    comms_layer.stop();
}

TEST_CASE("Sending more than one Sendable") {
    Comms::CommsLayer comms_layer;
    comms_layer.init_without_physical_layers();

    TestData data1;
    data1.x = 1.5f;
    data1.y = 2.5f;
    data1.z = 3.5f;
    data1.w = 0x12345678;

    BigTestData data2;
    data2.blah[0] = 2.5f;

    Comms::Sendable<TestData> test_data1(data1);
    Comms::Sendable<BigTestData> test_data2(data2);
    test_data1.send_to_comms();
    test_data2.send_to_comms();

    comms_layer.send_packets();
    comms_layer.set_hid_incoming(comms_layer.get_hid_outgoing());
    comms_layer.set_ethernet_incoming(comms_layer.get_ethernet_outgoing());
    comms_layer.receive_packets();

    REQUIRE(comms_layer.get_firmware_data().test_data.x == data1.x);
    REQUIRE(comms_layer.get_firmware_data().test_data.y == data1.y);
    REQUIRE(comms_layer.get_firmware_data().test_data.z == data1.z);
    REQUIRE(comms_layer.get_firmware_data().test_data.w == data1.w);

    REQUIRE(comms_layer.get_firmware_data().big_test_data.blah[0] == data2.blah[0]);
}

TEST_CASE("Sendable works if it gets out of scope") {
    Comms::CommsLayer comms_layer;
    comms_layer.init_without_physical_layers();

    TestData data;
    data.x = 1.5f;
    data.y = 2.5f;
    data.z = 3.5f;
    data.w = 0x12345678;

    {
        TestData data2 = data;
        Comms::Sendable<TestData> test_data(data2);
        test_data.send_to_comms();
    }  // test_data goes out of scope here

    comms_layer.send_packets();
    comms_layer.set_hid_incoming(comms_layer.get_hid_outgoing());
    comms_layer.set_ethernet_incoming(comms_layer.get_ethernet_outgoing());
    comms_layer.receive_packets();

    REQUIRE(comms_layer.get_firmware_data().test_data.x == data.x);
    REQUIRE(comms_layer.get_firmware_data().test_data.y == data.y);
    REQUIRE(comms_layer.get_firmware_data().test_data.z == data.z);
    REQUIRE(comms_layer.get_firmware_data().test_data.w == data.w);
}

#endif  // HIVE