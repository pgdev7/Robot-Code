#include "subsystems/drive/ModuleIOTalonSpark.h"
#include "Schematic.h"
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/configs/Slot0Configs.hpp"
#include "ctre/phoenix6/core/CoreCANcoder.hpp"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "ctre/phoenix6/swerve/SwerveModuleConstants.hpp"
#include "frc/MathUtil.h"
#include "frc/geometry/Rotation2d.h"
#include "rev/ClosedLoopTypes.h"
#include "rev/ConfigureTypes.h"
#include "rev/SparkLowLevel.h"
#include "rev/SparkMax.h"
#include "rev/config/ClosedLoopConfig.h"
#include "rev/config/EncoderConfig.h"
#include "rev/config/SignalsConfig.h"
#include "rev/config/SparkBaseConfig.h"
#include "subsystems/drive/DriveConstants.h"
#include "subsystems/drive/ModuleIO.h"
#include "subsystems/drive/OdometryThread.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/frequency.h"
#include "units/voltage.h"
#include "util/PhoenixUtil.h"
#include "util/SparkUtil.h"
#include <utility>
#include <vector>

template <int module> ModuleIOTalonSpark<module>::ModuleIOTalonSpark() {
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

  int driveTalonCanId = 0;
  switch (module) {
  case 0: {
    driveTalonCanId = Schematic.frontLeftDriveCanId;
    break;
  }
  case 1: {
    driveTalonCanId = Schematic.frontRightDriveCanId;
    break;
  }
  case 2: {
    driveTalonCanId = Schematic.backLeftDriveCanId;
    break;
  }
  case 3: {
    driveTalonCanId = Schematic.backRightDriveCanId;
    break;
  }
  }

  data->driveTalon =
      std::move(ctre::phoenix6::hardware::TalonFX(driveTalonCanId));

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

  data->turnController = data->turnSpark.GetClosedLoopController();

  ctre::phoenix6::configs::TalonFXConfiguration driveConfig;
  driveConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  driveConfig.Feedback.SensorToMechanismRatio =
      DriveConstants::driveMotorReduction;
  driveConfig.TorqueCurrent.PeakForwardTorqueCurrent =
      DriveConstants::driveMotorCurrentLimit;
  driveConfig.TorqueCurrent.PeakReverseTorqueCurrent =
      -DriveConstants::driveMotorCurrentLimit;
  driveConfig.CurrentLimits.StatorCurrentLimit =
      DriveConstants::driveMotorCurrentLimit;
  driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  driveConfig.MotorOutput.Inverted =
      ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  ctre::phoenix6::configs::Slot0Configs slot0config;
  slot0config.kP = DriveConstants::driveKp;
  slot0config.kI = 0;
  slot0config.kD = DriveConstants::driveKd;
  slot0config.kS = DriveConstants::driveKs;
  slot0config.kV = DriveConstants::driveKv;
  slot0config.kA = DriveConstants::driveKa;
  slot0config.StaticFeedforwardSign =
      ctre::phoenix6::signals::StaticFeedforwardSignValue::UseClosedLoopSign;
  driveConfig.WithSlot0(slot0config);
  PhoenixUtil::tryUntilOk(5, [&, this]() {
    return data->driveTalon.getConfigurator().apply(driveConfig, 0.25);
  });
  PhoenixUtil::tryUntilOk(
      5, [&, this]() { return data->driveTalon.setPosition(0.0, 0.25); });

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

  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      units::hertz_t(50.0), data->turnPositionAbsolute);

  auto holder{data};

  // Create odometry queues
  data->timestampQueue = OdometryThread::getInstance()->makeTimestampQueue();

  // Create turn status signals
  data->turnPositionQueue = OdometryThread::getInstance()->registerSignal(
      &data->turnSpark,
      [holder]() { return holder->turnEncoder.GetPosition(); });
  data->turnPositionAbsolute = data->turnEncoderAbsolute.GetPosition();

  // Initialize turn relative encoder
  tryUntilOk(5, [this]() {
    return data->turnEncoder.SetPosition(
        units::radian_t(data->turnEncoderAbsolute.GetAbsolutePosition()));
  });

  // Create drive status signals
  data->drivePosition = data->driveTalon.GetPosition();
  data->drivePositionQueue = OdometryThread::getInstance()->registerSignal(
      [holder]() { return holder->driveTalon.GetPosition(); });
  data->driveVelocity = data->driveTalon.GetVelocity();
  data->driveAppliedVolts = data->driveTalon.GetMotorVoltage();
  data->driveCurrent = data->driveTalon.GetStatorCurrent();

  // Configure periodic frames
  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      DriveConstants::odometryFrequency, data->drivePosition);
  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      50.0, data->driveVelocity, data->driveAppliedVolts, data->driveCurrent,
      data->turnPositionAbsolute);
  ctre::phoenix6::hardware::ParentDevice::OptimizeBusUtilizationForAll(
      data->driveTalon, data->turnEncoderAbsolute);
}

template <int module>
void ModuleIOTalonSpark<module>::updateInputs(ModuleIOInputs &inputs) {
  // Refresh all signals
  auto driveStatus = ctre::phoenix6::BaseStatusSignal::RefreshAll(
      data->drivePosition, data->driveVelocity, data->driveAppliedVolts,
      data->driveCurrent);
  ctre::phoenix6::BaseStatusSignal::RefreshAll(data->turnPositionAbsolute);
  inputs.turnPositionAbsolute =
      units::radian_t(data->turnPositionAbsolute.GetValue());

  // Update drive inputs
  inputs.driveConnected =
      data->driveConnectedDebounce.calculate(driveStatus.isOK());
  inputs.drivePosition = data->drivePosition;
  inputs.driveVelocity = data->driveVelocity;
  inputs.driveAppliedVoltage = data->driveAppliedVoltage;
  inputs.driveCurrent = data->driveCurrent;

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
void ModuleIOTalonSpark<module>::setDriveOpenLoop(units::volt_t output) {
  switch (DriveConstants::driveClosedLoopOutput) {
  case ctre::phoenix6::swerve::ClosedLoopOutputType::Voltage: {
    data->voltageRequest.WithOutput(output);
    break;
  }
  case ctre::phoenix6::swerve::ClosedLoopOutputType::TorqueCurrentFOC: {
    data->torqueCurrentRequest.WithOutput(output);
    break;
  }
  }
}

template <int module>
void ModuleIOTalonSpark<module>::setTurnOpenLoop(units::volt_t output) {
  data->turnSpark.SetVoltage(output());
}

template <int module>
void ModuleIOTalonSpark<module>::setDriveVelocity(
    units::radians_per_second_t velocity) {
  switch (DriveConstants::driveClosedLoopOutput) {
  case ctre::phoenix6::swerve::ClosedLoopOutputType::Voltage: {
    data->velocityVoltageRequest.WithVelocity(velocity);
    break;
  }
  case ctre::phoenix6::swerve::ClosedLoopOutputType::TorqueCurrentFOC: {
    data->velocityTorqueCurrentRequest.WithVelocity(velocity);
    break;
  }
  }
}

template <int module>
void ModuleIOTalonSpark<module>::setTurnPosition(frc::Rotation2d rotation) {
  double setpoint = frc::InputModulus((rotation + data->zeroRotation).Radians(),
                                      DriveConstants::turnPIDMinInput,
                                      DriveConstants::turnPIDMaxInput);
  data->turnController.setReference(
      setpoint, rev::spark::SparkBase::ControlType::kPosition);
}