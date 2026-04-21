#pragma once

#include <concepts>
#include <type_traits>

#include <frc/geometry/Rotation2d.h>
#include <units/angular_velocity.h>

struct ModuleIOInputs {
    bool driveConnected = false;
     double drivePositionRad = 0.0;
     double driveVelocityRadPerSec = 0.0;
     double driveAppliedVolts = 0.0;
     double driveCurrentAmps = 0.0;

     bool turnConnected = false;
     frc::Rotation2d turnPosition{};
     double turnVelocityRadPerSec = 0.0;
     double turnAppliedVolts = 0.0;
     double turnCurrentAmps = 0.0;

     double turnPositionAbsolute = 0.0;

     double *odometryTimestamps = nullptr;
     double *odometryDrivePositionsRad = nullptr;
     frc::Rotation2d *odometryTurnPositions = nullptr;
};

template <typename T>
concept ModuleIO = requires (T t) {
    std::is_void_v<decltype(t.updateInputs(std::declval<ModuleIOInputs>()))>;
    std::is_void_v<decltype(t.setDriveOpenLoop(std::declval<double>()))>;
    std::is_void_v<decltype(t.setTurnOpenLoop(std::declval<double>()))>;
    std::is_void_v<decltype(t.setDriveVelocity(std::declval<units::angular_velocity::radians_per_second_t>()))>;
    std::is_void_v<decltype(t.setTurnPosition(std::declval<frc::Rotation2d>()))>;
};