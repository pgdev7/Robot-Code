#include "subsystems/drive/ModuleIOSpark.h"
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/core/CoreCANcoder.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "frc/geometry/Rotation2d.h"
#include "rev/ClosedLoopTypes.h"
#include "rev/ConfigureTypes.h"
#include "rev/config/ClosedLoopConfig.h"
#include "rev/config/EncoderConfig.h"
#include "rev/config/SignalsConfig.h"
#include "subsystems/drive/DriveConstants.h"
#include "subsystems/drive/ModuleIO.h"
#include "subsystems/drive/OdometryThread.h"

#include <rev/REVLibError.h>
#include <rev/SparkBase.h>
#include <rev/SparkLowLevel.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>

#include "Schematic.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/frequency.h"
#include "units/voltage.h"
#include "util/SparkUtil.h"
#include <units/angle.h>

#include <frc/MathUtil.h>
#include <utility>
#include <vector>

template <int module> ModuleIOSpark<module>::ModuleIOSpark() {
  switch (module) {
  case 0: {
    data->zeroRotation = DriveConstants::frontLeftZeroRotation;
    break;
  }
  case 1: {
    data->zeroRotation = DriveConstants::frontRightZeroRtoation;
    break;
  }
  case 2: {
    data->zeroRotation = DriveConstants::backLeftZeroRotation;
    break;
  }
  case 3: {
    data->zeroRotation = DriveConstants::backRightZeroRotation;
    break;
  }
  };

  int driveSparkCanId = 0;
  switch (module) {
  case 0: {
    driveSparkCanId = Schematic.frontLeftDriveCanId;
    break;
  }
  case 1: {
    driveSparkCanId = Schematic.frontRightDriveCanId;
    break;
  }
  case 2: {
    driveSparkCanId = Schematic.backLeftDriveCanId;
    break;
  }
  case 3: {
    driveSparkCanId = Schematic.backRightDriveCanId;
    break;
  }
  }

  data->driveSpark = std::move(rev::spark::SparkMax(
      driveSparkCanId, rev::spark::SparkLowLevel::MotorType::kBrushless));
  data->driveEncoder = data->driveSpark.GetEncoder();

  int turnSparkCanId = 0;
  switch (module) {
  case 0: {
    turnSparkCanId = Schematic.frontLeftTurnCanId;
    break;
  }
  case 1: {
    turnSparkCanId = Schematic.frontRightTurnCanId;
    break;
  }
  case 2: {
    turnSparkCanId = Schematic.backLeftTurnCanId;
    break;
  }
  case 3: {
    turnSparkCanId = Schematic.backRightTurnCanId;
    break;
  }
  }

  data->turnSpark = std::move(rev::spark::SparkMax(
      turnSparkCanId, rev::spark::SparkLowLevel::MotorType::kBrushless));
  data->turnEncoder = data->turnSpark.GetEncoder();

  int turnAbsoluteEncoderCanId = 0;
  switch (module) {
  case 0: {
    turnAbsoluteEncoderCanId = Schematic.frontLeftAbsoluteEncoderCanId;
    break;
  }
  case 1: {
    turnAbsoluteEncoderCanId = Schematic.frontRightAbsoluteEncoderCanId;
    break;
  }
  case 2: {
    turnAbsoluteEncoderCanId = Schematic.backLeftAbsoluteEncoderCanId;
    break;
  }
  case 3: {
    turnAbsoluteEncoderCanId = Schematic.backRightAbsoluteEncoderCanId;
    break;
  }
  }

  data->turnEncoderAbsolute =
      std::move(ctre::phoenix6::hardware::CANcoder(turnAbsoluteEncoderCanId));

  data->driveController = data->driveSpark.GetClosedLoopController();
  data->turnController = data->turnSpark.GetClosedLoopController();

  auto driveConfig =
      rev::spark::SparkBaseConfig()
          .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
          .SmartCurrentLimit(DriveConstants::driveMotorCurrentLimit.value())
          .VoltageCompensation(12.0)
          .Apply(rev::spark::EncoderConfig()
                     .PositionConversionFactor(
                         DriveConstants::driveEncoderPositionFactor)
                     .VelocityConversionFactor(
                         DriveConstants::driveEncoderVelocityFactor)
                     .UvwMeasurementPeriod(10)
                     .UvwAverageDepth(2))
          .Apply(rev::spark::ClosedLoopConfig()
                     .SetFeedbackSensor(
                         rev::spark::FeedbackSensor::kPrimaryEncoder)
                     .Pidf(DriveConstants::driveKp, 0.0,
                           DriveConstants::driveKd, 0.0))
          .Apply(rev::spark::SignalsConfig()
                     .PrimaryEncoderPositionAlwaysOn(true)
                     .PrimaryEncoderPositionPeriodMs(static_cast<int>(
                         1000 / DriveConstants::odometryFrequency))
                     .PrimaryEncoderVelocityAlwaysOn(true)
                     .PrimaryEncoderVelocityPeriodMs(20)
                     .AppliedOutputPeriodMs(20)
                     .BusVoltagePeriodMs(20)
                     .OutputCurrentPeriodMs(20));

  tryUntilOk(5, [&, this]() {
    return data->driveSpark.Configure(driveConfig,
                                      rev::ResetMode::kResetSafeParameters,
                                      rev::PersistMode::kPersistParameters);
  });
  tryUntilOk(5, [this]() { return data->driveEncoder.SetPosition(0.0); });

  auto turnConfig =
      rev::spark::SparkBaseConfig()
          .Inverted(DriveConstants::turnInverted)
          .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
          .SmartCurrentLimit(DriveConstants::turnMotorCurrentLimit)
          .VoltageCompensation(12.0)
          .Apply(rev::spark::EncoderConfig()
                     .PositionConversionFactor(
                         DriveConstants::turnEncoderPositionFactor)
                     .VelocityConversionFactor(
                         DriveConstants::turnEncoderVelocityFactor)
                     .UvwAverageDepth(2))
          .Apply(
              rev::spark::ClosedLoopConfig()
                  .SetFeedbackSensor(
                      rev::spark::FeedbackSensor::kPrimaryEncoder)
                  .PositionWrappingEnabled(true)
                  .PositionWrappingInputRange(DriveConstants::turnPIDMinInput(),
                                              DriveConstants::turnPIDMaxInput())
                  .Pidf(DriveConstants::turnKp, 0.0, DriveConstants::turnKd,
                        0.0))
          .Apply(rev::spark::SignalsConfig()
                     .AbsoluteEncoderPositionAlwaysOn(true)
                     .AbsoluteEncoderPositionPeriodMs(static_cast<int>(
                         1000 / DriveConstants::odometryFrequency))
                     .AbsoluteEncoderVelocityAlwaysOn(true)
                     .AbsoluteEncoderVelocityPeriodMs(20)
                     .AppliedOutputPeriodMs(20)
                     .BusVoltagePeriodMs(20)
                     .OutputCurrentPeriodMs(20));

  tryUntilOk(5, [&, this]() {
    return data->turnSpark.Configure(turnConfig,
                                     rev::ResetMode::kResetSafeParameters,
                                     rev::PersistMode::kPersistParameters);
  });

  ctre::phoenix6::configs::CANcoderConfiguration cancoderConfig;
  cancoderConfig.MagnetSensor.MagnetOffset =
      units::turn_t(0); // This is done later in the code
  cancoderConfig.MagnetSensor.SensorDirection =
      DriveConstants::turnEncoderInverted
          ? ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive
          : ctre::phoenix6::signals::SensorDirectionValue::
                CounterClockwise_Positive;
  data->turnEncoderAbsolute.GetConfigurator().Apply(cancoderConfig);
  data->turnPositionAbsolute = data->turnEncoderAbsolute.GetPosition();

  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      units::hertz_t(50.0), data->turnPositionAbsolute);

  auto holder{data};

  // Create odometry queues
  data->timestampQueue = OdometryThread::getInstance()->makeTimestampQueue();
  data->drivePositionQueue = OdometryThread::getInstance()->registerSignal(
      &data->driveSpark,
      [holder]() { return holder->driveEncoder.GetPosition(); });
  data->turnPositionQueue = OdometryThread::getInstance()->registerSignal(
      &data->turnSpark,
      [holder]() { return holder->turnEncoder.GetPosition(); });

  tryUntilOk(5, [this]() {
    return data->turnEncoder.SetPosition(
        units::radian_t(data->turnEncoderAbsolute.GetAbsolutePosition()));
  });
}

template <int module>
void ModuleIOSpark<module>::updateInputs(ModuleIOInputs &inputs) {
  ctre::phoenix6::BaseStatusSignal::RefreshAll(data->turnPositionAbsolute);
  inputs.turnPositionAbsolute =
      units::radian_t(data->turnPositionAbsolute.GetValue());

  // Update drive inputs
  sparkStickyFault = false;
  ifOk(
      data->driveSpark, [this]() { return data->driveEncoder.GetPosition(); },
      [&](double value) { inputs.drivePosition = units::radian_t(value); });
  ifOk(
      data->driveSpark, [this]() { return data->driveEncoder.GetVelocity(); },
      [&](double value) {
        inputs.driveVelocity = units::radians_per_second_t(value);
      });
  ifOk(
      data->driveSpark,
      [this]() {
        return std::vector<double>{data->driveSpark.GetAppliedOutput(),
                                   data->driveSpark.GetBusVoltage()};
      },
      [&](std::vector<double> values) {
        inputs.driveAppliedVoltage = units::volt_t(values[0] * values[1]);
      });
  ifOk(
      data->driveSpark,
      [this]() { return data->driveSpark.GetOutputCurrent(); },
      [&](double value) { inputs.driveCurrent = units::ampere_t(value); });
  inputs.driveConnected =
      data->driveConnectedDebounce.Calculate(!sparkStickyFault);

  // Update turn inputs
  sparkStickyFault = false;
  ifOk(
      data->turnSpark, [this]() { return data->turnEncoder.GetPosition(); },
      [&, this](double value) {
        inputs.turnPosition =
            frc::Rotation2d(units::radian_t(value)) - data->zeroRotation;
      });
  ifOk(
      data->turnSpark, [this]() { return data->turnEncoder.GetVelocity(); },
      [&](double value) {
        inputs.turnVelocity = units::radians_per_second_t(value);
      });
  ifOk(
      data->turnSpark,
      [this]() {
        return std::vector<double>{data->turnSpark.getAppliedOutput(),
                                   data->turnSpark.getBusVoltage()};
      },
      [&](std::vector<double> values) {
        inputs.turnAppliedVoltage = units::volt_t(values[0] * values[1]);
      });
  ifOk(
      data->turnSpark, [this]() { return data->turnSpark.getOutputCurrent(); },
      [&](double value) { inputs.turnCurrent = units::ampere_t(value); });
  inputs.turnConnected =
      data->turnConnectedDebounce.Calculate(!sparkStickyFault);

  // Update odometry inputs
  inputs.odometryTimestamps = data->timeStampQueue;
  inputs.odometryDrivePositionsRad = data->drivePositionQueue.toVector();
  inputs.odometryTurnPositions.clear();
  auto turnPositionQueue = data->turnPositionQueue.toVector();
  for (auto position : turnPositionQueue) {
    inputs.odometryTurnPositions.emplace_back(frc::Rotation2d(position) -
                                              data->zeroRotation);
  }
  data->timestampQueue.clear();
  data->drivePositionQueue.clear();
  data->turnPositionQueue.clear();
}

template <int module>
void ModuleIOSpark<module>::setDriveOpenLoop(units::volt_t output) {
  data->driveSpark.SetVoltage(output());
}

template <int module>
void ModuleIOSpark<module>::setTurnOpenLoop(units::volt_t output) {
  data->turnSpark.SetVoltage(output());
}

template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

template <int module>
void ModuleIOSpark<module>::setDriveVelocity(
    units::angular_velocity::radians_per_second_t velocity) {
  double ffVolts = DriveConstants::driveKs * sgn(velocity()) +
                   DriveConstants::driveKv * velocity();
  data->driveController.SetReference(
      velocity(), rev::spark::SparkBase::ControlType::kVelocity,
      rev::spark::ClosedLoopSlot::kSlot0, ffVolts,
      rev::spark::SparkClosedLoopController::ArbFFUnits::kVoltage);
}

template <int module>
void ModuleIOSpark<module>::setTurnPosition(frc::Rotation2d rotation) {
  double setpoint = frc::InputModulus((rotation + data->zeroRotation).Radians(),
                                      DriveConstants::turnPIDMinInput,
                                      DriveConstants::turnPIDMaxInput);
  data->turnController.setReference(
      setpoint, rev::spark::SparkBase::ControlType::kPosition);
}