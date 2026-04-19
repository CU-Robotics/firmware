#include "watchdog.hpp"
#include "safety.hpp"

void watchdog_callback() {
    const bool safety_called = safety::call_safety_function();

    // TODO: Read and print a flag to identify the cause of stall

    Serial.println(
        safety_called
            ? "[Watchdog] Warning fired, safety function called."
            : "[Watchdog] Warning fired, but no safety function registered!"
    );
}

Watchdog::Watchdog() {
    this->config.trigger = trigger; //default trigger value = 0.5 second
    this->config.timeout = timeout; //default timeout value = 1 second
    this->config.callback = watchdog_callback; 
}

Watchdog::Watchdog(double trigger, double timeout) {
    this->timeout = timeout; 
    this->trigger = trigger;
    this->config.trigger = trigger;
    this->config.timeout = timeout;
    this->config.callback = watchdog_callback;
}

void Watchdog::start() {
    wdt.begin(config);
}

void Watchdog::reset_teensey() {
    wdt.reset();
}

void Watchdog::set(double trigger, double timeout) {
    this->timeout = timeout;
    this->trigger = trigger;
    this->config.trigger = trigger;
    this->config.timeout = timeout;
    this->config.callback = watchdog_callback;
}

void Watchdog::print_config() {
    Serial.println("Trigger");
    Serial.println(this->config.trigger);
    Serial.println("Timeout");
    Serial.println(this->config.timeout);
}

void Watchdog::reconfigure(double trigger, double timeout) {
    this->timeout = timeout;
    this->trigger = trigger;
    this->config.trigger = trigger;
    this->config.timeout = timeout;
    wdt.feed();
}

void Watchdog::feed() {
    wdt.feed();
}
