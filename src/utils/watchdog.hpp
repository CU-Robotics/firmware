#pragma once

#include <WDT_T4/Wathdog_t4.h>



void watchdog_callback();

class Watchdog{
private:
    double trigger;
    double timeout;
    WDT_T4<WDT1> wdt;
    WDT_timings_t config;

public:
    Watchdog(double trigger, double timeout){
        this->timeout = timeout;
        this->trigger = trigger;
        this->config.trigger = trigger;
        this->config.timeout = timeout;
        this->config.callback = watchdog_callback;
    }
    void watchdog_start(){
        wdt.begin(config);
    }
    void watchdog_stop(){
        wdt.reset();
    }
    void wathdog_feed(){
        wdt.feed();
    }
};


