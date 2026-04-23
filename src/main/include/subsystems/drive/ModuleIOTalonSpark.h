#pragma once

#include <memory>

#include "ModuleIO.h"
#include "subsystems/drive/ModuleIO.h"
#include "units/current.h"
#include "units/time.h"
#include "units/voltage.h"
#include "util/ArrayBlockingQueue.h"
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/PositionTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/controls/TorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/VelocityTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
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

template <int module = 0> class ModuleIOTalonSpark {
public:
  ModuleIOTalonSpark();

  void updateInputs(ModuleIOInputs &inputs);
  void setDriveOpenLoop(units::volt_t output);
  void setTurnOpenLoop(units::volt_t output);
  void setDriveVelocity(units::angular_velocity::radians_per_second_t velocity);
  void setTurnPosition(frc::Rotation2d rotation);

private:
  struct Data {
    frc::Rotation2d zeroRotation;

    // Hardware objects
    ctre::phoenix6::hardware::TalonFX driveTalon;

    rev::spark::SparkMax turnSpark{
        0, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkRelativeEncoder turnEncoder;
    rev::spark::SparkClosedLoopController turnController;

    ctre::phoenix6::hardware::CANcoder turnEncoderAbsolute;

    // Voltage control requests
    ctre::phoenix6::controls::VoltageOut voltageRequest;
    ctre::phoenix6::controls::PositionVoltage positionVoltageRequest;
    ctre::phoenix6::controls::VelocityVoltage velocityVoltageRequest;

    // Torque-current control requests
    ctre::phoenix6::controls::TorqueCurrentFOC torqueCurrentRequest;
    ctre::phoenix6::controls::PositionTorqueCurrentFOC
        positionTorqueCurrentRequest;
    ctre::phoenix6::controls::VelocityTorqueCurrentFOC
        velocityTorqueCurrentRequest;

    // Inputs from drive motor
    ctre::phoenix6::StatusSignal<units::turn_t> drivePosition;
    ctre::phoenix6::StatusSignal<units::radians_per_second_t> driveVelocity;
    ctre::phoenix6::StatusSignal<units::volt_t> driveAppliedVoltage;
    ctre::phoenix6::StatusSignal<units::ampere_t> driveCurrent;

    // Input from turn encoder absolute
    ctre::phoenix6::StatusSignal<units::turn_t> turnPositionAbsolute;

    // Queue inputs from odometry thread
    std::shared_ptr<ArrayBlockingQueue<units::second_t>> timestampQueue;
    std::shared_ptr<ArrayBlockingQueue<double>> drivePositionQueue;
    std::shared_ptr<ArrayBlockingQueue<double>> turnPositionQueue;

    // Connection debouncers
    frc::Debouncer driveConnectedDebounce{units::second_t(0.5)};
    frc::Debouncer turnConnectedDebounce{units::second_t(0.5)};
  };

  std::shared_ptr<Data> data = std::make_shared<Data>();
};

static_assert(ModuleIO<ModuleIOTalonSpark<0>>,
              "ModuleIO based classes must implement all features");
static_assert(ModuleIO<ModuleIOTalonSpark<1>>,
              "ModuleIO based classes must implement all features");
static_assert(ModuleIO<ModuleIOTalonSpark<2>>,
              "ModuleIO based classes must implement all features");
static_assert(ModuleIO<ModuleIOTalonSpark<3>>,
              "ModuleIO based classes must implement all features");