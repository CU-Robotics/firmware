#ifndef STEREO_CAM_TRIGGER_H
#define STEREO_CAM_TRIGGER_H

#include <Arduino.h>
#include <avr/interrupt.h>
#include "sensors/sensor.hpp"
#include "comms/data/stereo_cam_trigger_data.hpp"

/// @brief define to enable FPS logging in the timer interrupt callback (debugging)
// #define LOG_STEREO_FPS

/// @brief class to manage triggering synchronized exposures for dual USB cameras
class StereoCamTrigger : Sensor{
  private:
    const Cfg::StereoCamTrigger& config;

    StereoCamTriggerData comms_data;
    
    /// @brief Teensyduino timer instance used to maintain signal
    IntervalTimer timer;
    
    /// @brief boolean indicating whether the signal has been stopped or not
    bool stopped = true;
    
    /// @brief timestamp of the last time an exposure was triggered (last time signal was set to HIGH)
    volatile uint32_t latest_exposure_timestamp = 0;
    
    /// @brief callback to pass to timer to update latest exposure timestamp
    void track_exposures();
  public:
    /// @brief constructor for StereoCamTrigger
    /// @param _fps desired FPS (frames per second) of the trigger signal
    StereoCamTrigger(const Cfg::StereoCamTrigger& config): Sensor(), config(config), comms_data(config.camera_trigger_name) {}
    
    /// @brief initialize trigger manager by starting the interval timer
    void init() override;

    void read() override {};

    void send_to_comms() const override;
    
    /// @brief start interval timer. Begins sending trigger signal to cameras via timer interrupt
    /// @param res the desired resolution (or interval size) of the timer interrupt in micros
    void start(int res);
    
    /// @brief stop interval timer. Can start again by calling start()
    void stop();
};

#endif