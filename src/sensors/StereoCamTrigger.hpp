#pragma once

#include <Arduino.h>
#include <avr/interrupt.h>
#include "sensors/sensor.hpp"
#include "comms/data/stereo_cam_trigger_data.hpp"
#include "robot_state_map.hpp"
#include <memory>



/// @brief define to enable FPS logging in the timer interrupt callback (debugging)

/// @brief class to manage triggering synchronized exposures for dual USB cameras
class StereoCamTrigger : public Sensor{
  private:
    /// @brief Configuration for stereo cam trigger
    const Cfg::StereoCamTrigger& config;

    /// @brief data to be sent to comms
    StereoCamTriggerData comms_data;
    
    /// @brief Teensyduino timer instance used to maintain signal
    IntervalTimer timer;
    
    /// @brief boolean indicating whether the signal has been stopped or not
    bool stopped = true;
    
    /// @brief micros per frame
    int mpf = 0; 

    /// @brief boolean to indicate whether the first trigger has been sent since starting the interupt.
    bool first_trigger = true;

    /// @brief timestamp of the last time an exposure was triggered (last time signal was set to HIGH)
    volatile uint32_t latest_exposure_timestamp = 0;

    /// @brief the number of frames that have been triggered since the last counter reset.
    int counter = 0;
    
    /// @brief callback to pass to timer to update latest exposure timestamp
    void track_exposures();
  public:
    /// @brief constructor for StereoCamTrigger
    /// @param config configuration data for the stereo cam trigger
    StereoCamTrigger(const Cfg::StereoCamTrigger& config);
    
    /// @brief initialize trigger manager by starting the interval timer
    void init() override;
    /// @brief empty read function since the updates are done in the timer interrupt callback
    void read() override;
    /// @brief Send exposure timestamp and estimated state at exposure to comms
    /// @note This is not implemented currently
    void send_to_comms() const override;
    
    /// @brief start interval timer. Begins sending trigger signal to cameras via timer interrupt
    /// @param res the desired resolution (or interval size) of the timer interrupt in micros
    void start(int res);
    
    /// @brief stop interval timer. Can start again by calling start()
    void stop();
};
