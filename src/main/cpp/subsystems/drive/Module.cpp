#include "subsystems/drive/Module.h"
#include "frc/Alert.h"
#include "frc/MathUtil.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "subsystems/drive/DriveConstants.h"
#include "subsystems/drive/ModuleIO.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/velocity.h"
#include "units/voltage.h"
#include <cmath>
#include <cstdlib>
#include <math.h>
#include <string>
#include <vector>

template <ModuleIO ModuleIO>
Module<ModuleIO>::Module(ModuleIO io, int index)
    : io(io), index(index),
      driveDisconnectedAlert("Disconnected drive motor on module " +
                                 std::to_string(index) + ".",
                             frc::Alert::AlertType::kError),
      turnDisconnectedAlert("Disconnected turn motor on module " +
                                std::to_string(index) + ".",
                            frc::Alert::AlertType::kError) {}

template <ModuleIO ModuleIO> void Module<ModuleIO>::periodic() {
  io.updateInputs(inputs);

  int sampleCount = inputs.odometryTimestamps.size();
  odometryPositions = std::vector<frc::SwerveModulePosition>(sampleCount);
  for (int i = 0; i < sampleCount; ++i) {
    units::meter_t position =
        inputs.odometryDrivePositionsRad[i] * DriveConstants::wheelRadius;
    frc::Rotation2d angle = inputs.odometryTurnPositions[i];
    odometryPositions[i] =
        frc::SwerveModulePosition{.distance = position, .angle = angle};
  }

  driveDisconnectedAlert.Set(!inputs.driveConnected);
  turnDisconnectedAlert.Set(!inputs.turnConnected);
}

template <ModuleIO ModuleIO>
void Module<ModuleIO>::runSetpoint(frc::SwerveModuleState state) {
  Module::optimize(state, inputs.turnPosition);
  state.CosineScale(inputs.turnPosition);

  io.setDriveVelocity(state.speed / DriveConstants::wheelRadius);
  io.setTurnPosition(state.angle);
}

template <ModuleIO ModuleIO>
void Module<ModuleIO>::runCharacterization(units::volt_t output) {
  io.setDriveOpenLoop(output);
  io.setTurnPosition(frc::Rotation2d());
}

template <ModuleIO ModuleIO> void Module<ModuleIO>::stop() {
  io.setDriveOpenLoop(units::volt_t(0));
  io.setTurnOpenLoop(units::volt_t(0));
}

template <ModuleIO ModuleIO> frc::Rotation2d Module<ModuleIO>::getAngle() {
  return inputs.turnPosition;
}

template <ModuleIO ModuleIO>
units::meter_t Module<ModuleIO>::getPositionMeters() {
  return inputs.drivePosition() * DriveConstants::wheelRadius;
}

template <ModuleIO ModuleIO>
units::meters_per_second_t Module<ModuleIO>::getVelocity() {
  return units::meters_per_second_t(inputs.driveVelocity() *
                                    DriveConstants::wheelRadius());
}

template <ModuleIO ModuleIO>
frc::SwerveModulePosition Module<ModuleIO>::getPosition() {
  return frc::SwerveModulePosition(getPositionMeters(), inputs.turnPosition);
}

template <ModuleIO ModuleIO>
frc::SwerveModuleState Module<ModuleIO>::getState() {
  return frc::SwerveModuleState(getVelocity(), inputs.turnPosition);
}

template <ModuleIO ModuleIO>
const std::vector<frc::SwerveModulePosition> &
Module<ModuleIO>::getOdometryPositions() {
  return odometryPositions;
}

template <ModuleIO ModuleIO>
const std::vector<double> &Module<ModuleIO>::getOdometryTimestamps() {
  return inputs.odometryTimestamps;
}

template <ModuleIO ModuleIO>
units::radian_t Module<ModuleIO>::getWheelRadiusCharacterizationPosition() {
  return inputs.drivePosition;
}

template <ModuleIO ModuleIO>
units::radians_per_second_t Module<ModuleIO>::getFFCharacterizationVelocity() {
  return inputs.driveVelocity;
}

template <ModuleIO ModuleIO>
void Module<ModuleIO>::optimize(frc::SwerveModuleState &state,
                                frc::Rotation2d currentAngle) {
  auto delta = state.angle - currentAngle;
  if (std::abs(frc::InputModulus(delta.Degrees()(), -180.0, 180.0)) > 90) {
    state.speed *= -1;
    state.angle = state.angle.RotateBy(frc::Rotation2d(units::radian_t(M_PI)));
  }
}