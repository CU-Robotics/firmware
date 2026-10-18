class GyroSource {
    /// @brief Get acceleration of the gyro arounds the X axis.
    /// @return Acceleration around the X axis.
    virtual float get_gyro_X() = 0;
    /// @brief Get acceleration of the gyro arounds the Y axis.
    /// @return Acceleration around the Y axis.
    virtual float get_gyro_Y() = 0;
    /// @brief Get acceleration of the gyro arounds the Z axis.
    /// @return Acceleration around the Z axis.
    virtual float get_gyro_Z() = 0;
};