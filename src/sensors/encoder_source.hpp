class EncoderSource {
  public:
    /// @brief Get angle of the last read function.
    /// @return Read angle (radians).
    virtual float get_angle() const = 0;
};