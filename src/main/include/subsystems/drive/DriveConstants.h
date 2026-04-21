#pragma once

#include <units/velocity.h>
#include <units/length.h>
#include <units/math.h>
#include <units/angle.h>
#include <units/voltage.h>
#include <units/current.h>
#include <units/length.h>
#include <units/mass.h>

#include <cmath>
#include <tuple>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include "Constants.h"
#include <ctre/phoenix6/swerve/SwerveModuleConstants.hpp>
#include <frc/system/plant/DCMotor.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/config/ModuleConfig.h>

#include "Schematic.h"

namespace DriveConstants
{
    inline constexpr units::velocity::meters_per_second_t maxSpeed{11.8};
    inline constexpr double odometryFrequency = 100.0;
    inline constexpr units::meter_t trackWidth{0.59005};
    inline constexpr units::meter_t wheelBase{0.5207};
    inline constexpr units::meter_t driveBaseRadius = units::math::hypot(trackWidth / 2.0, wheelBase / 2.0);
    inline const std::vector<frc::Translation2d> moduleTranslations = {
        frc::Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        frc::Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        frc::Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        frc::Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)};

    // Zeroed rotation values for each module, see setup instructions
    const auto [frontLeftZeroRotation, frontRightZeroRtoation, backLeftZeroRotation, backRightZeroRotation] = []() -> std::tuple<frc::Rotation2d, frc::Rotation2d, frc::Rotation2d, frc::Rotation2d>
    {
        switch (Constants::getRobot())
        {
        case RobotType::ROBOT_2026_COMP:
            return {frc::Rotation2d(units::radian_t(0.126 + M_PI)), {units::radian_t(0.759 + 0.03)}, {units::radian_t(-1.630)}, {units::radian_t(-2.072)}};
        default:
            return {};
        };
    }();

    // Drive motor configuration
    inline constexpr ctre::phoenix6::swerve::ClosedLoopOutputType driveClosedLoopOutput = ctre::phoenix6::swerve::ClosedLoopOutputType::Voltage;
    inline constexpr units::ampere_t driveMotorCurrentLimit{50};
    inline const units::meter_t wheelRadius = units::inch_t(1.905);
    inline constexpr double driveMotorReduction = 8.16;
    inline const frc::DCMotor driveGearbox = frc::DCMotor::KrakenX60FOC(1);

    // Drive encoder configuration
    inline constexpr double driveEncoderPositionFactor =
        2 * M_PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    inline constexpr double driveEncoderVelocityFactor =
        (2 * M_PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    // Drive PID configuration
    inline constexpr double driveKp = 2.525;
    inline constexpr double driveKd = 0.0;
    inline constexpr double driveKs = 0.5;
    inline constexpr double driveKv = 0.0;
    inline constexpr double driveKa = 0.0;
    inline constexpr double driveSimP = 0.3;
    inline constexpr double driveSimD = 0.0;
    inline constexpr double driveSimKs = 0.0;
    inline constexpr double driveSimKv = 0.0789;

    // Turn motor configuration
    inline constexpr bool turnInverted = false;
    inline constexpr int turnMotorCurrentLimit = 10;
    inline constexpr double turnMotorReduction = 26;
    inline constexpr frc::DCMotor turnGearbox = frc::DCMotor::NEO(1);

    // Turn encoder configuration
    inline constexpr bool turnEncoderInverted = false;
    inline constexpr double turnEncoderPositionFactor =
        2 * M_PI / turnMotorReduction; // Rotations -> Radians
    inline constexpr double turnEncoderVelocityFactor =
        (2 * M_PI) / 60.0 / turnMotorReduction; // RPM -> Rad/Sec

    // Turn PID configuration
    inline constexpr double turnKp = 3.5;
    inline constexpr double turnKd = 0.0;
    inline constexpr double turnSimP = 9.0;
    inline constexpr double turnSimD = 0.0;
    inline constexpr double turnPIDMinInput = -M_PI; // Radians
    inline constexpr double turnPIDMaxInput = M_PI;  // Radians

    // PathPlanner configuration
    inline constexpr units::kilogram_t robotMass{51};
    inline constexpr units::kilogram_square_meter_t robotMOI{6.883};
    inline constexpr double wheelCOF = 1.2;
    inline const pathplanner::RobotConfig ppConfig{
        robotMass,
        robotMOI,
        pathplanner::ModuleConfig(
            wheelRadius,
            maxSpeed,
            wheelCOF,
            driveGearbox,
            driveMotorReduction,
            driveMotorCurrentLimit,
            1),
        moduleTranslations};

    // Requirements for RobotTilt to trigger
    inline constexpr double rollThreshhold = 10.0;
    inline constexpr double pitchThreshold = 10.0;
};