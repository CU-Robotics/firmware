#ifndef STEREO_CAM_TRIGGER_H
#define STEREO_CAM_TRIGGER_H

#include <Arduino.h>
#include <avr/interrupt.h>

/// @brief define to enable FPS logging in the timer interrupt callback (debugging)
// #define LOG_STEREO_FPS

/// @brief GPIO pin number to send trigger signal to USB cameras. Can change as needed
const int TRIG_PIN = 32;

/// @brief desired pulse width of trigger signal in micros (minimum pulse width is 10 micros - check See3CAM datasheet)
const int TRIG_PULSE_WIDTH = 10;

/// @brief class to manage triggering synchronized exposures for dual USB cameras
class StereoCamTrigger {
  private:
    /// @brief desired FPS (frames per second) of the trigger signal
    int fps;
    
    /// @brief Teensyduino timer instance used to maintain signal
    IntervalTimer timer;
    
    /// @brief boolean indicating whether the signal has been stopped or not
    bool stopped = true;
    
    /// @brief current state of the GPIO trigger pin (HIGH or LOW)
    volatile int trig_pin_state = LOW;
    
    /// @brief timestamp of the last time an exposure was triggered (last time signal was set to HIGH)
    volatile uint32_t latest_exposure_timestamp = 0;
    
    /// @brief callback to pass to timer to update latest exposure timestamp
    void track_exposures();
  public:
    /// @brief constructor for StereoCamTrigger
    /// @param _fps desired FPS (frames per second) of the trigger signal
    StereoCamTrigger(int _fps): fps(_fps) {}
    
    /// @brief initialize trigger manager by starting the interval timer
    void init();
    
    /// @brief start interval timer. Begins sending trigger signal to cameras via timer interrupt
    /// @param res the desired resolution (or interval size) of the timer interrupt in micros
    void start(int res);
    
    /// @brief stop interval timer. Can start again by calling start()
    void stop();
    
    /// @brief get the latest time at which an exposure was triggered
    /// @return the latest exposure timestamp
    uint32_t get_latest_exposure_timestamp();
};

#endif