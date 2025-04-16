#include "hive_data.hpp"

#if defined(HIVE)
#include "modules/comms/comms_layer.hpp"    // for CommsLayer
#include "modules/hive/environment.hpp"     // for Hive::env
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
