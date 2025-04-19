#pragma once

#include <WDT_T4/Watchdog_t4.h>

/// @brief watchdog callback that is triggered before the CPU is reset, this is used to give a warning that the CPU is about to be reset
void watchdog_callback() {
    Serial.println("Watchdog is almost out of time, please feed the watchdog");
}

/// @brief This watchdog timer restarts the CPU if the trigger variable has not been sensed after the timout time, this is used to safely exits out when a software error occurs
class Watchdog {
private:
    /// @brief tigger variable is used as a warning that a trigger should happen (callback time - trigger time) = time the callback will be triggered, this is in seconds, range: (0, 128), the callback will be called,
    /// @note that 0 is actually 0.5 seconds, 128 seconds is actually 127.5 seconds(subtract )
    double trigger = 2;
    /// @brief  timout varible is used to reference after a certain amount a time if the watchdog has not been fed it will restart the CPU, units: seconds, range: (0, 128)
    /// @note that 0 is actually 0.5 seconds, 128 seconds is actually 127.5 seconds(subtract )
    double timeout = 3;
    
    /// @brief the wdt is the type to initialize the watchdog and the watchdog you used is selected, here watchdog 1 is selected which is used to reset the CPU
    WDT_T4<WDT1> wdt;
    /// @brief  struct for the watchdog configurations used to initialize the configurations of the timer, including the callback, the trigger,  and the timout
    WDT_timings_t config;

public:
    /// @brief default watchdog if no user input is given, in order to start the watchdog, call start()
    Watchdog() {
        this->config.trigger = trigger; //default trigger value = 0.5 second
        this->config.timeout = timeout; //default timeout value = 1 second
        this->config.callback = watchdog_callback; 
    }

    /// @brief watchdog constructor, which initializes the watchdog
    /// @param trigger value given by the user that will set the watchdog configuration's timer, double which valid ranges go from 0 to 128 seconds
    /// @param timeout value given by user that will set the watchdog configuration's for the timout, it is a double which valid ranges go from 0 to 128 seconds
    Watchdog(double trigger, double timeout) {
        this->timeout = timeout; 
        this->trigger = trigger;
        this->config.trigger = trigger;
        this->config.timeout = timeout;
        this->config.callback = watchdog_callback;
    }

    /// @brief this must be called in order to start the watchdog, after this is called, watchdog 1 
    /// is turned on to keep track of when trigger and timouts should occur based on previous initializations
    /// of timer and trigger
    void start() {
        wdt.begin(config);
    }

    /// @brief this resets the cpu, calling this will immediatly reset the cpu
    /// @note Dont call this lightly
    void reset_teensey() {
        wdt.reset();
    }

    /// @brief This function is to set up the configuration of the watchdog, this is what the watchdog will be restricted to
    /// @param trigger The trigger variable, is in seconds, that will call the callback function for the watchdog when it is trigger seconds away from a timeout
    /// @param timeout The timeout variable is the time, in seconds, when the cpu will be reset if a feed to the watchdog timer hasn't been recieved between the current time and the current time - timeout
    void set(double trigger, double timeout) {
        this->timeout = timeout;
        this->trigger = trigger;
        this->config.trigger = trigger;
        this->config.timeout = timeout;
        this->config.callback = watchdog_callback;
    }

    /// @brief This function is to help debug and verify your timeout has the values of trigger 
    /// and timeout that you expect, will print it to the serial terminal
    void print_config() {
        Serial.println("Trigger");
        Serial.println(this->config.trigger);
        Serial.println("Timeout");
        Serial.println(this->config.timeout);
    }

    /// @brief this function is to recongiure the trigger and timeout values, if you need the watchdog for another use than previouly
    /// @param trigger (0-128 seconds)The trigger variable, is in seconds, that will call the callback function for the watchdog when it is trigger seconds away from a timeout
    /// @param timeout (0-128 seconds)The timeout variable is the time, in seconds, when the cpu will be reset if a feed to the watchdog timer hasn't been recieved between the current time and the current time - timeout
    void reconfigure(double trigger, double timeout) {
        this->timeout = timeout;
        this->trigger = trigger;
        this->config.trigger = trigger;
        this->config.timeout = timeout;
        wdt.feed();
    }

    /// @brief this feeds the wathdog and restarts the time to avoid the timeout indicating everything is working correctly, 
    /// when this is called before a timeout, no CPU reset occurs atleast until the next timeout variable
    void feed() {
        wdt.feed();
    }
};


