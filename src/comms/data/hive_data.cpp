#include "hive_data.hpp"

#if defined(HIVE)
#include "modules/comms/comms_layer.hpp"    // for CommsLayer
#include "modules/hive/environment.hpp"     // for Hive::env
#include <doctest/doctest.h>                // for doctest
#elif defined(FIRMWARE)
#include "comms/comms_layer.hpp"            // for CommsLayer
#endif

namespace Comms {

void HiveData::set_data(CommsData* data) {
    // place the data in the mega struct
    switch (data->type_label) {
    case TypeLabel::TestData: {
        TestData *test = static_cast<TestData*>(data);
        // TODO: why does doing test_data = *test; not work?
        memcpy(&test_data, test, sizeof(TestData));
        break;
    }
    case TypeLabel::BigTestData: {
        BigTestData *big_test = static_cast<BigTestData*>(data);
        memcpy(&big_test_data, big_test, sizeof(BigTestData));
        // big_test_data = *static_cast<BigTestData*>(data);
        break;
    }
    case TypeLabel::TargetState: {
        TargetState* target = static_cast<TargetState*>(data);
        memcpy(&target_state, target, sizeof(TargetState));
        // target_state = *static_cast<TargetState*>(data);
        break;
    }
    case TypeLabel::OverrideState: {
        OverrideState* o_state = static_cast<OverrideState*>(data);
        memcpy(&override_state, o_state, sizeof(OverrideState));
        // override_state = *static_cast<OverrideState*>(data);
        break;
    }
    case TypeLabel::ConfigSection: {
        ConfigSection* config = static_cast<ConfigSection*>(data);
        memcpy(&config_section, config, sizeof(ConfigSection));
        // config_section = *static_cast<ConfigSection*>(data);
        break;
    }
    default:
    #if defined(HIVE)
        throw std::runtime_error("Invalid type label given to place in mega struct");
    #elif defined(FIRMWARE)
        assert(false && "Invalid type label given to place in mega struct");
    #endif
    }
}

}   // namespace Comms

#if defined(HIVE)

TEST_CASE("setting hive data structs") {
    Comms::HiveData hive_data;

    TestData test_data;
    test_data.x = 55;
    hive_data.set_data(&test_data);
    CHECK(hive_data.test_data.x == 55);

    BigTestData big_test_data;
    big_test_data.blah[1] = 55;
    hive_data.set_data(&big_test_data);
    CHECK(hive_data.big_test_data.blah[1] == 55);

    TargetState target_state;
    target_state.time = 55;
    hive_data.set_data(&target_state);
    CHECK(hive_data.target_state.time == 55);

    OverrideState override_state;
    override_state.time = 55;
    hive_data.set_data(&override_state);
    CHECK(hive_data.override_state.time == 55);

    ConfigSection config_section;
    config_section.section_id = 55;
    hive_data.set_data(&config_section);
    CHECK(hive_data.config_section.section_id == 55);
}

#endif  // defined (HIVE)