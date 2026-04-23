#pragma once

#include "frc/geometry/Rotation2d.h"
#include "units/angle.h"
#include "units/time.h"
#include <type_traits>
#include <units/angular_velocity.h>
#include <utility>
#include <vector>

struct GyroIOInputs {
  bool connected = false;
  frc::Rotation2d yawPosition;
  units::radians_per_second_t yawVelocity{0};
  std::vector<units::second_t> odometryYawTimestamps;
  std::vector<frc::Rotation2d> odometryYawPositions;
  units::degree_t roll;
  units::degree_t pitch;
};

template <typename T>
concept GyroIO = requires(T t) {
  std::is_void_v<decltype(t.updateInputs(std::declval<GyroIOInputs &>()))>;
};