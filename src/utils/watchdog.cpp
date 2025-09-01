#include "watchdog.hpp"
#include "logger.hpp"

void watchdog_callback() {
    logger.println(LogDestination::Serial, "Watchdog is almost out of time, please feed the watchdog");
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
    logger.println(LogDestination::Serial, "Trigger");
    logger.println(LogDestination::Serial, this->config.trigger);
    logger.println(LogDestination::Serial, "Timeout");
    logger.println(LogDestination::Serial, this->config.timeout);
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