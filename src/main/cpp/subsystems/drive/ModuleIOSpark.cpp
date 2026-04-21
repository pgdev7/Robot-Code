#include "subsystems/drive/ModuleIOSpark.h"
#include "subsystems/drive/DriveConstants.h"

#include <rev/SparkLowLevel.h>
#include <rev/SparkMax.h>
#include <rev/SparkBase.h>
#include <rev/SparkRelativeEncoder.h>

#include "Schematic.h"

template <int module>
ModuleIOSpark<module>::ModuleIOSpark()
{
    switch (module)
    {
    case 0:
    {
        zeroRotation = DriveConstants::frontLeftZeroRotation;
        break;
    }
    case 1:
    {
        zeroRotation = DriveConstants::frontRightZeroRtoation;
        break;
    }
    case 2:
    {
        zeroRotation = DriveConstants::backLeftZeroRotation;
        break;
    }
    case 3:
    {
        zeroRotation = DriveConstants::backRightZeroRotation;
        break;
    }
    };

    int driveSparkCanId = 0;
    switch (module)
    {
    case 0:
    {
        driveSparkCanId = Schematic.frontLeftDriveCanId;
        break;
    }
    case 1:
    {
        driveSparkCanId = Schematic.frontRightDriveCanId;
        break;
    }
    case 2:
    {
        driveSparkCanId = Schematic.backLeftDriveCanId;
        break;
    }
    case 3:
    {
        driveSparkCanId = Schematic.backRightDriveCanId;
        break;
    }
    }

    driveSpark = std::move(
        rev::spark::SparkMax(driveSparkCanId,
                             rev::spark::SparkLowLevel::MotorType::kBrushless));
    driveEncoder = driveSpark.GetEncoder();

    int turnSparkCanId = 0;
    switch (module)
    {
    case 0:
    {
        turnSparkCanId = Schematic.frontLeftTurnCanId;
        break;
    }
    case 1:
    {
        turnSparkCanId = Schematic.frontRightTurnCanId;
        break;
    }
    case 2:
    {
        turnSparkCanId = Schematic.backLeftTurnCanId;
        break;
    }
    case 3:
    {
        turnSparkCanId = Schematic.backRightTurnCanId;
        break;
    }
    }

    turnSpark = std::move(rev::spark::SparkMax(turnSparkCanId, rev::spark::SparkLowLevel::MotorType::kBrushless));
    turnEncoder = turnSpark.GetEncoder();

    int turnAbsoluteEncoderCanId = 0;
    switch (module)
    {
    case 0:
    {
        turnAbsoluteEncoderCanId = Schematic.frontLeftAbsoluteEncoderCanId;
        break;
    }
    case 1:
    {
        turnAbsoluteEncoderCanId = Schematic.frontRightAbsoluteEncoderCanId;
        break;
    }
    case 2:
    {
        turnAbsoluteEncoderCanId = Schematic.backLeftAbsoluteEncoderCanId;
        break;
    }
    case 3:
    {
        turnAbsoluteEncoderCanId = Schematic.backRightAbsoluteEncoderCanId;
        break;
    }
    }

    turnEncoderAbsolute = std::move(ctre::phoenix6::hardware::CANcoder(turnAbsoluteEncoderCanId));

    driveController = driveSpark.GetClosedLoopController();
    turnController = turnSpark.GetClosedLoopController();

    auto driveConfig = rev::spark::SparkBaseConfig()
                           .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
                           .SmartCurrentLimit(DriveConstants::driveMotorCurrentLimit.value())
                           .VoltageCompensation(12.0)
                           .Apply(rev::spark::EncoderConfig()
                                      .PositionConversionFactor(DriveConstants::driveEncoderPositionFactor)
                                      .VelocityConversionFactor(DriveConstants::driveEncoderVelocityFactor)
                                      .UvwMeasurementPeriod(10)
                                      .UvwAverageDepth(2))
                           .Apply(rev::spark::ClosedLoopConfig()
                                      .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
                                      .Pidf(DriveConstants::driveKp, 0.0, DriveConstants::driveKd, 0.0))
                           .Apply(rev::spark::SignalsConfig()
                                      .PrimaryEncoderPositionAlwaysOn(true)
                                      .PrimaryEncoderPositionPeriodMs(static_cast<int>(1000 / DriveConstants::odometryFrequency))
                                      .PrimaryEncoderVelocityAlwaysOn(true)
                                      .PrimaryEncoderVelocityPeriodMs(20)
                                      .AppliedOutputPeriodMs(20)
                                      .BusVoltagePeriodMs(20)
                                      .OutputCurrentPeriodMs(20));

    for (int i = 0; i < 5; ++i)
    {
        driveSpark.Configure(driveConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
    }
    for (int i = 0; i < 5; ++i)
    {
        driveEncoder.SetPosition(0.0);
    }

    auto turnConfig = rev::spark::SparkBaseConfig()
                          .Inverted(DriveConstants::turnInverted)
                          .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
                          .SmartCurrentLimit(DriveConstants::turnMotorCurrentLimit)
                          .VoltageCompensation(12.0)
                          .Apply(rev::spark::EncoderConfig()
                                     .PositionConversionFactor(DriveConstants::turnEncoderPositionFactor)
                                     .VelocityConversionFactor(DriveConstants::turnEncoderVelocityFactor)
                                     .UvwAverageDepth(2))
                          .Apply(rev::spark::ClosedLoopConfig()
                                     .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
                                     .PositionWrappingEnabled(true)
                                     .PositionWrappingInputRange(DriveConstants::turnPIDMinInput, DriveConstants::turnPIDMaxInput)
                                     .Pidf(DriveConstants::turnKp, 0.0, DriveConstants::turnKd, 0.0))
                          .Apply(rev::spark::SignalsConfig()
                                     .AbsoluteEncoderPositionAlwaysOn(true)
                                     .AbsoluteEncoderPositionPeriodMs(static_cast<int>(1000 / DriveConstants::odometryFrequency))
                                     .AbsoluteEncoderVelocityAlwaysOn(true)
                                     .AbsoluteEncoderVelocityPeriodMs(20)
                                     .AppliedOutputPeriodMs(20)
                                     .BusVoltagePeriodMs(20)
                                     .OutputCurrentPeriodMs(20));

    for (int i = 0; i < 5; ++i)
    {
        turnSpark.Configure(turnConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
    }

    ctre::phoenix6::configs::CANcoderConfiguration cancoderConfig;
    cancoderConfig.MagnetSensor.MagnetOffset = 0; // This is done later in the code
    cancoderConfig.MagnetSensor.SensorDirection =
        DriveConstants::turnEncoderInverted
            ? ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive
            : ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
    turnEncoderAbsolute.GetConfigurator().Apply(cancoderConfig);
    turnPositionAbsolute = turnEncoderAbsolute.GetPosition();

    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(units::hertz_t(50.0), turnPositionAbsolute);
}