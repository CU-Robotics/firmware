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
        test_data = *static_cast<TestData*>(data);
        break;
    }
    case TypeLabel::TargetState: {
        target_state = *static_cast<TargetState*>(data);
        break;
    }
    case TypeLabel::OverrideState: {
        override_state = *static_cast<OverrideState*>(data);
        break;
    }
    case TypeLabel::ConfigSection: {
        config_section = *static_cast<ConfigSection*>(data);
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
