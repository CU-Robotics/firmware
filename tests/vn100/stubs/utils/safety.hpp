#pragma once
#include <stdexcept>
namespace safety {
template<class... Args> [[noreturn]] void safety_procedure(const char* message, Args...) {
    throw std::runtime_error(message);
}
template<class... Args> void assert_or_safety_procedure(bool condition, const char* message, Args... args) {
    if (!condition) safety_procedure(message, args...);
}
}
