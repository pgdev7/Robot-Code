#pragma once

#include <memory>

#include "ModuleIO.h"
#include "units/time.h"
#include "units/voltage.h"
#include "util/ArrayBlockingQueue.h"
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <frc/Encoder.h>
#include <frc/filter/Debouncer.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/motorcontrol/Spark.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkLowLevel.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

template <int module = 0> class ModuleIOSpark {
public:
  ModuleIOSpark();

  void updateInputs(ModuleIOInputs &inputs);
  void setDriveOpenLoop(units::volt_t output);
  void setTurnOpenLoop(units::volt_t output);
  void setDriveVelocity(units::angular_velocity::radians_per_second_t velocity);
  void setTurnPosition(frc::Rotation2d rotation);

private:
  struct Data {
    frc::Rotation2d zeroRotation;

    rev::spark::SparkMax driveSpark{
        0, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkRelativeEncoder driveEncoder;
    rev::spark::SparkClosedLoopController driveController;

    rev::spark::SparkMax turnSpark{
        0, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkRelativeEncoder turnEncoder;
    rev::spark::SparkClosedLoopController turnController;

    ctre::phoenix6::hardware::CANcoder turnEncoderAbsolute;

    ctre::phoenix6::StatusSignal<units::turn_t> turnPositionAbsolute;

    std::shared_ptr<ArrayBlockingQueue<units::second_t>> timestampQueue;
    std::shared_ptr<ArrayBlockingQueue<double>> drivePositionQueue;
    std::shared_ptr<ArrayBlockingQueue<double>> turnPositionQueue;

    frc::Debouncer driveConnectedDebounce{units::second_t(0.5)};
    frc::Debouncer turnConnectedDebounce{units::second_t(0.5)};
  };

  std::shared_ptr<Data> data = std::make_shared<Data>();
};

static_assert(ModuleIO<ModuleIOSpark<0>>,
              "ModuleIO based classes must implement all features");
static_assert(ModuleIO<ModuleIOSpark<1>>,
              "ModuleIO based classes must implement all features");
static_assert(ModuleIO<ModuleIOSpark<2>>,
              "ModuleIO based classes must implement all features");
static_assert(ModuleIO<ModuleIOSpark<3>>,
              "ModuleIO based classes must implement all features");
static_assert(ModuleIO<ModuleIOSpark<4>>,
              "ModuleIO based classes must implement all features");