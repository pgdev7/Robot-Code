#pragma once

#include <type_traits>
#include <utility>
#include <vector>

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/charge.h>
#include <units/voltage.h>

struct ModuleIOInputs {
  bool driveConnected = false;
  units::radian_t drivePosition{0.0};
  units::radians_per_second_t driveVelocity{0.0};
  units::volt_t driveAppliedVoltage{0.0};
  units::ampere_t driveCurrent{0.0};

  bool turnConnected = false;
  frc::Rotation2d turnPosition{};
  units::radians_per_second_t turnVelocity{0.0};
  units::volt_t turnAppliedVoltage{0.0};
  units::ampere_t turnCurrent{0.0};

  units::radian_t turnPositionAbsolute{0.0};

  std::vector<double> odometryTimestamps;
  std::vector<double> odometryDrivePositionsRad;
  std::vector<frc::Rotation2d> odometryTurnPositions;
};

template <typename T>
concept ModuleIO = requires(T t) {
  std::is_void_v<decltype(t.updateInputs(std::declval<ModuleIOInputs &>()))>;
  std::is_void_v<decltype(t.setDriveOpenLoop(std::declval<units::volt_t>()))>;
  std::is_void_v<decltype(t.setTurnOpenLoop(std::declval<units::volt_t>()))>;
  std::is_void_v<decltype(t.setDriveVelocity(
      std::declval<units::angular_velocity::radians_per_second_t>()))>;
  std::is_void_v<decltype(t.setTurnPosition(std::declval<frc::Rotation2d>()))>;
};