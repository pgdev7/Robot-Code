#pragma once

#include "frc/geometry/Rotation2d.h"
#include <type_traits>
#include <units/angular_velocity.h>
#include <vector>

struct GyroIOInputs {
  bool connected = false;
  frc::Rotation2d yawPosition;
  units::radians_per_second_t yawVelocity{0};
  std::vector<double> odometryYawTimestamps;
  std::vector<frc::Rotation2d> odometryYawPositions;
  double roll;
  double pitch;
};

template <typename T>
concept GyroIO =
    requires(T t) { std::is_void_v<decltype(t.updateInputs(GyroIOInputs{}))>; };