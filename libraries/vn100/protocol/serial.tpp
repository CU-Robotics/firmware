/**
 * @file serial.tpp
 * @author Jackson Stepka (jast2434@colorado.edu) (@Pandabear1125)
 * @brief Explicit template instantiations for the Serial class' read and write register functions.
 * @date 2025-07
 *
 * Implementation based on VectorNav VN-100 IMU/AHRS Interface Control Document (Firmware v3.1.0.0)
 *
 * @license While the source code is provided and visible, it is not open source. All rights are reserved. 
 *          No one may copy, modify, or distribute this code without explicit permission from the author.
 *          For more information, please contact the author directly.
 */

#include "serial.hpp"

namespace vn
{

// template explicit instantiations for reading and writing registers
template ErrorCode Serial::read_register(WriteSettings &reg);
template ErrorCode Serial::write_register(WriteSettings &reg);

template ErrorCode Serial::read_register(RestoreFactorySettings &reg);
template ErrorCode Serial::write_register(RestoreFactorySettings &reg);

template ErrorCode Serial::read_register(Reset &reg);
template ErrorCode Serial::write_register(Reset &reg);

template ErrorCode Serial::read_register(FirmwareUpdate &reg);
template ErrorCode Serial::write_register(FirmwareUpdate &reg);

template ErrorCode Serial::read_register(KnownMagneticDisturbance &reg);
template ErrorCode Serial::write_register(KnownMagneticDisturbance &reg);

template ErrorCode Serial::read_register(KnownAccelerationDisturbance &reg);
template ErrorCode Serial::write_register(KnownAccelerationDisturbance &reg);

template ErrorCode Serial::read_register(AsyncOutputEnable &reg);
template ErrorCode Serial::write_register(AsyncOutputEnable &reg);

template ErrorCode Serial::read_register(SetGyroBias &reg);
template ErrorCode Serial::write_register(SetGyroBias &reg);

template ErrorCode Serial::read_register(PollBinaryOutputMessage &reg);
template ErrorCode Serial::write_register(PollBinaryOutputMessage &reg);

template ErrorCode Serial::read_register(UserTag &reg);
template ErrorCode Serial::write_register(UserTag &reg);

template ErrorCode Serial::read_register(Baudrate &reg);
template ErrorCode Serial::write_register(Baudrate &reg);

template ErrorCode Serial::read_register(AsyncDataOutputType &reg);
template ErrorCode Serial::write_register(AsyncDataOutputType &reg);

template ErrorCode Serial::read_register(AsyncDataOutputFreq &reg);
template ErrorCode Serial::write_register(AsyncDataOutputFreq &reg);

template ErrorCode Serial::read_register(CommunicationProtocolControl &reg);
template ErrorCode Serial::write_register(CommunicationProtocolControl &reg);

template ErrorCode Serial::read_register(SyncControl &reg);
template ErrorCode Serial::write_register(SyncControl &reg);

template ErrorCode Serial::read_register(LegacyCompatibilitySettings &reg);
template ErrorCode Serial::write_register(LegacyCompatibilitySettings &reg);

template ErrorCode Serial::read_register(BinaryOutputMessageConfig1 &reg);
template ErrorCode Serial::write_register(BinaryOutputMessageConfig1 &reg);

template ErrorCode Serial::read_register(BinaryOutputMessageConfig2 &reg);
template ErrorCode Serial::write_register(BinaryOutputMessageConfig2 &reg);

template ErrorCode Serial::read_register(BinaryOutputMessageConfig3 &reg);
template ErrorCode Serial::write_register(BinaryOutputMessageConfig3 &reg);

template ErrorCode Serial::read_register(MagneticGravityReference &reg);
template ErrorCode Serial::write_register(MagneticGravityReference &reg);

template ErrorCode Serial::read_register(VpeBasicControl &reg);
template ErrorCode Serial::write_register(VpeBasicControl &reg);

template ErrorCode Serial::read_register(VpeMagnetometerBasicTuning &reg);
template ErrorCode Serial::write_register(VpeMagnetometerBasicTuning &reg);

template ErrorCode Serial::read_register(VpeAccelerometerBasicTuning &reg);
template ErrorCode Serial::write_register(VpeAccelerometerBasicTuning &reg);

template ErrorCode Serial::read_register(FilterStartupGyroBias &reg);
template ErrorCode Serial::write_register(FilterStartupGyroBias &reg);

template ErrorCode Serial::read_register(MagnetometerCalibration &reg);
template ErrorCode Serial::write_register(MagnetometerCalibration &reg);

template ErrorCode Serial::read_register(AccelerometerCalibration &reg);
template ErrorCode Serial::write_register(AccelerometerCalibration &reg);

template ErrorCode Serial::read_register(ReferenceFrameRotation &reg);
template ErrorCode Serial::write_register(ReferenceFrameRotation &reg);

template ErrorCode Serial::read_register(DeltaThetaDeltaVelocityConfig &reg);
template ErrorCode Serial::write_register(DeltaThetaDeltaVelocityConfig &reg);

template ErrorCode Serial::read_register(GyroCalibration &reg);
template ErrorCode Serial::write_register(GyroCalibration &reg);

template ErrorCode Serial::read_register(ImuFilteringConfig &reg);
template ErrorCode Serial::write_register(ImuFilteringConfig &reg);

template ErrorCode Serial::read_register(RealTimeHsiControl &reg);
template ErrorCode Serial::write_register(RealTimeHsiControl &reg);

template ErrorCode Serial::read_register(VelocityAidingMeasurement &reg);
template ErrorCode Serial::write_register(VelocityAidingMeasurement &reg);

template ErrorCode Serial::read_register(VelocityAidingControl &reg);
template ErrorCode Serial::write_register(VelocityAidingControl &reg);

template ErrorCode Serial::read_register(ReferenceModelConfig &reg);
template ErrorCode Serial::write_register(ReferenceModelConfig &reg);

template ErrorCode Serial::read_register(HeaveBasicConfig &reg);
template ErrorCode Serial::write_register(HeaveBasicConfig &reg);

template ErrorCode Serial::read_register(Model &reg);
template ErrorCode Serial::write_register(Model &reg);

template ErrorCode Serial::read_register(HardwareVersion &reg);
template ErrorCode Serial::write_register(HardwareVersion &reg);

template ErrorCode Serial::read_register(SerialNumber &reg);
template ErrorCode Serial::write_register(SerialNumber &reg);

template ErrorCode Serial::read_register(FirmwareVersion &reg);
template ErrorCode Serial::write_register(FirmwareVersion &reg);

template ErrorCode Serial::read_register(SyncStatus &reg);
template ErrorCode Serial::write_register(SyncStatus &reg);

template ErrorCode Serial::read_register(YawPitchRoll &reg);
template ErrorCode Serial::write_register(YawPitchRoll &reg);

template ErrorCode Serial::read_register(Quaternion &reg);
template ErrorCode Serial::write_register(Quaternion &reg);

template ErrorCode Serial::read_register(QuaternionCompensatedImu &reg);
template ErrorCode Serial::write_register(QuaternionCompensatedImu &reg);

template ErrorCode Serial::read_register(YawPitchRollCompensatedImu &reg);
template ErrorCode Serial::write_register(YawPitchRollCompensatedImu &reg);

template ErrorCode Serial::read_register(YawPitchRollLinearAccelGyro &reg);
template ErrorCode Serial::write_register(YawPitchRollLinearAccelGyro &reg);

template ErrorCode Serial::read_register(YawPitchRollLinearInertialAccelGyro &reg);
template ErrorCode Serial::write_register(YawPitchRollLinearInertialAccelGyro &reg);

template ErrorCode Serial::read_register(CompensatedMagnetometer &reg);
template ErrorCode Serial::write_register(CompensatedMagnetometer &reg);

template ErrorCode Serial::read_register(CompensatedImu &reg);
template ErrorCode Serial::write_register(CompensatedImu &reg);

template ErrorCode Serial::read_register(ImuMeasurements &reg);
template ErrorCode Serial::write_register(ImuMeasurements &reg);

template ErrorCode Serial::read_register(DeltaThetaDeltaVelocity &reg);
template ErrorCode Serial::write_register(DeltaThetaDeltaVelocity &reg);

template ErrorCode Serial::read_register(RealTimeHsiResults &reg);
template ErrorCode Serial::write_register(RealTimeHsiResults &reg);

template ErrorCode Serial::read_register(HeaveAndHeaveRate &reg);
template ErrorCode Serial::write_register(HeaveAndHeaveRate &reg);

} // namespace vn
