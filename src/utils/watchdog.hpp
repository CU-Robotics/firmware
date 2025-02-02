#pragma once

#include <WDT_T4/Wathdog_t4.h>


/// @brief watchdog callback to indicate trigger has occured
void watchdog_callback(){
        Serial.println(millis());
}

/// @brief This watchdog timer restarts the CPU if the trigger variable has not been sensed after the timout time, this is used to safely exits out when a software error occurs
class Watchdog{
private:
    /// @brief tigger variable is used to reference how long a trigger should happen, this is in seconds, range: (0, 128), the callback will be called, note that 0 is actually 0.5 seconds, 128 seconds is actually 127.5 seconds(subtract )
    double trigger = 0;
    /// @brief  timout varible is used to reference after a certain amount a time if the watchdog has not been fed it will restart the CPU, units: seconds, range: (0, 128)
    double timeout = 1;
    /// @brief the wdt is the type to initialize the watchdog and the watchdog you used is selected, here watchdog 1 is selected which is used to reset the CPU
    WDT_T4<WDT1> wdt;
    /// @brief  struct for the watchdog configurations used to initialize the configurations of the timer, including the callback, the trigger,  and the timout
    WDT_timings_t config;
public:
    /// @brief default watchdog if no user input is given, in order to start the watchdog, myst start watchdog_start()
    Watchdog(){
        this->config.trigger = trigger; //default trigger value = 0.5 second
        this->config.timeout = trigger; //default timeout value = 1 second
        this->config.callback = watchdog_callback; 
    }
    /// @brief  watchdog constructor, which initializes the watchdog
    /// @param trigger value given by the user that will set the watchdog configuration's timer, double which valid ranges go from 0 to 128 seconds
    /// @param timeout value given by user that will set the watchdog configuration's for the timout, it is a double which valid ranges go from 0 to 128 seconds
    Watchdog(double trigger, double timeout){
        this->timeout = timeout; 
        this->trigger = trigger;
        this->config.trigger = trigger;
        this->config.timeout = timeout;
        this->config.callback = watchdog_callback;
    }
    /// @brief this must be called in order to start the watchdog, after this is called, wathcdog 1 is turned on to keep track of when trigger and timouts should occur based on previous initializations of timer and trigger
    void watchdog_start(){
        wdt.begin(config);
    }
    /// @brief this stops the watchdog, to stop using it if no longer needed
    void watchdog_stop(){
        wdt.reset();
    }
    /// @brief this feeds the wathdog and restats the time to avoid the timeout indicating software is working correctly, when this is called before a timeout, no CPU reset occurs 
    void wathdog_feed(){
        wdt.feed();
    }
};


