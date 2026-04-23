#pragma once

#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "subsystems/drive/ModuleIO.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/velocity.h"
#include "units/voltage.h"
#include <frc/Alert.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <vector>

template <ModuleIO ModuleIO> class Module {
public:
  Module(ModuleIO io, int index);

  void periodic();
  void runSetpoint(frc::SwerveModuleState state);
  void runCharacterization(units::volt_t output);
  void stop();
  frc::Rotation2d getAngle();
  units::meter_t getPositionMeters();
  units::meters_per_second_t getVelocity();
  frc::SwerveModulePosition getPosition();
  frc::SwerveModuleState getState();
  const std::vector<frc::SwerveModulePosition> &getOdometryPositions();
  const std::vector<double> &getOdometryTimestamps();
  units::radian_t getWheelRadiusCharacterizationPosition();
  units::radians_per_second_t getFFCharacterizationVelocity();
  static void optimize(frc::SwerveModuleState &state,
                       frc::Rotation2d currentAngle);

private:
  ModuleIO io;
  ModuleIOInputs inputs;
  int index;

  frc::Alert driveDisconnectedAlert;
  frc::Alert turnDisconnectedAlert;
  std::vector<frc::SwerveModulePosition> odometryPositions;
};