#include "subsystems/drive/GyroIOPigeon2.h"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/core/CorePigeon2.hpp"
#include "frc/geometry/Rotation2d.h"
#include "subsystems/drive/DriveConstants.h"
#include "subsystems/drive/GyroIO.h"
#include "subsystems/drive/OdometryThread.h"
#include "units/angle.h"
#include "units/frequency.h"

GyroIOPigeon2::GyroIOPigeon2() {
  data->pigeon.GetConfigurator().Apply(
      ctre::phoenix6::configs::Pigeon2Configuration());
  data->pigeon.GetConfigurator().SetYaw(0_deg);
  data->yaw.SetUpdateFrequency(
      units::hertz_t(DriveConstants::odometryFrequency));
  data->yawVelocity.SetUpdateFrequency(50_Hz);
  data->pigeon.OptimizeBusUtilization();
  auto holder{data};
  data->yawTimestampQueue = OdometryThread::getInstance()->makeTimestampQueue();
  data->yawPositionQueue = OdometryThread::getInstance()->registerSignal(
      [holder]() { return holder->yaw.GetValue()(); });
  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      50_Hz, data->accelY, data->accelX, data->robotPitch, data->robotRoll);
}

void GyroIOPigeon2::updateInputs(GyroIOInputs &inputs) {
  inputs.connected = ctre::phoenix6::BaseStatusSignal::RefreshAll(
                         data->yaw, data->yawVelocity, data->accelX,
                         data->accelY, data->robotRoll, data->robotPitch)
                         .IsOK();
  inputs.yawPosition = frc::Rotation2d(data->yaw.GetValue());
  inputs.yawVelocity = data->yawVelocity.GetValue();
  inputs.odometryYawTimestamps = data->yawTimestampQueue->toVector();
  inputs.odometryYawPositions.clear();

  auto yawPositionsVec = data->yawPositionQueue->toVector();
  for (auto pos : yawPositionsVec) {
    inputs.odometryYawPositions.emplace_back(units::degree_t(pos));
  }

  inputs.roll = data->robotRoll.GetValue();
  inputs.pitch = data->robotPitch.GetValue();

  data->yawTimestampQueue->clear();
  data->yawPositionQueue->clear();
}