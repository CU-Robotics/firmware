#pragma once
#include <cstdio>
#include <utility>

namespace firmware_sim_host {
struct FatalSafety { char message[512]; };
}
namespace safety {
/* Host-only unwind replaces the target's motor-zeroing infinite halt. The bridge
 * catches this record, zeroes every sink and permanently latches the instance.
 * A failed invariant never returns into the firmware's dereference. */
template<class... Args> [[noreturn]] inline void safety_procedure(const char* format, Args&&... args) {
    firmware_sim_host::FatalSafety fault{};
    if constexpr (sizeof...(Args) == 0) {
        std::snprintf(fault.message, sizeof(fault.message), "%s", format);
    } else {
        std::snprintf(fault.message, sizeof(fault.message), format, std::forward<Args>(args)...);
    }
    throw fault;
}
template<class... Args> inline void assert_or_safety_procedure(bool condition, const char* format, Args&&... args) {
    if (!condition) safety_procedure(format, std::forward<Args>(args)...);
}
}
