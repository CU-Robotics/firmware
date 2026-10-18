#pragma once

/**
 * To support simulation, we can distinguish sensors from sources. Sensors comply
 * with the `Sensor` interface and supply the concrete hardware implementation for
 * any particular sensor. Sensor owns device lifecycle, communication, and measurement
 * acquisition. Source is distinct in its responsiblity and represents an interface
 * through which consumers can access measurements, regardless of how those measurements
 * were acquired or generated. This enables simulations to use mocks instead of concrete
 * hardware sensor implementations. Since separate sources do not share any hardware
 * management responsibilities, this compile-time trait is used for sources instead of a
 * base class to identify sources.
 */
template <typename T> inline constexpr bool is_source_v = false;
template <typename T>
concept Source = is_source_v<T>;