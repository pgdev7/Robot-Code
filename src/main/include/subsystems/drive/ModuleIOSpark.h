#pragma once

#include <concepts>
#include <type_traits>

#include <frc/geometry/Rotation2d.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/Encoder.h>
#include <units/angular_velocity.h>
#include "ModuleIO.h"
#include <rev/SparkMax.h>
#include <rev/SparkLowLevel.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <units/angle.h>
#include <queue>
#include <frc/filter/Debouncer.h>

template <int module = 0>
class ModuleIOSpark
{
public:
    ModuleIOSpark();

    void updateInputs(ModuleIOInputs inputs);
    void setDriveOpenLoop(double output);
    void setTurnOpenLoop(double output);
    void setDriveVelocity(units::angular_velocity::radians_per_second_t velocity);
    void setTurnPosition(frc::Rotation2d rotation);

private:
    frc::Rotation2d zeroRotation;

    rev::spark::SparkMax driveSpark{0,
                                    rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkRelativeEncoder driveEncoder;
    rev::spark::SparkClosedLoopController driveController;

    rev::spark::SparkMax turnSpark{0,
                                   rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkRelativeEncoder turnEncoder;
    rev::spark::SparkClosedLoopController turnController;

    ctre::phoenix6::hardware::CANcoder turnEncoderAbsolute;

    ctre::phoenix6::StatusSignal<units::turn_t> turnPositionAbsolute;

    std::queue<double> timestampQueue;
    std::queue<double> drivePositionQueue;
    std::queue<double> turnPositionQueue;

    frc::Debouncer driveConnectedDebounce{0.5};
    frc::Debouncer turnConnectedDebounce{0.5};
};

static_assert(ModuleIO<ModuleIOSpark<0>>, "ModuleIO based classes must implement all features");
static_assert(ModuleIO<ModuleIOSpark<1>>, "ModuleIO based classes must implement all features");
static_assert(ModuleIO<ModuleIOSpark<2>>, "ModuleIO based classes must implement all features");
static_assert(ModuleIO<ModuleIOSpark<3>>, "ModuleIO based classes must implement all features");
static_assert(ModuleIO<ModuleIOSpark<4>>, "ModuleIO based classes must implement all features");