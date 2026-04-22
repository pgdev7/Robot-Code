#include "frc/controller/PIDController.h"
#include "frc/geometry/Rotation2d.h"
#include "subsystems/drive/DriveConstants.h"
#include "subsystems/drive/ModuleIO.h"
#include "units/angular_velocity.h"
#include "units/voltage.h"
#include <frc/simulation/DCMotorSim.h>
#include <frc2/command/PIDCommand.h>

class ModuleIOSim {
public:
  void updateInputs(ModuleIOInputs &inputs);
  void setDriveOpenLoop(units::volt_t output);
  void setTurnOpenLoop(units::volt_t output);
  void setDriveVelocity(units::angular_velocity::radians_per_second_t velocity);
  void setTurnPosition(frc::Rotation2d rotation);

private:
  const frc::sim::DCMotorSim driveSim;
  const frc::sim::DCMotorSim turnSim;
  bool driveClosedLoop = false;
  bool turnClosedLoop = false;
  frc::PIDController driveController{DriveConstants::driveSimP, 0,
                                     DriveConstants::driveSimD};
  frc::PIDController turnController{DriveConstants::turnSimP, 0,
                                    DriveConstants::turnSimD};
  double driveFFVolts = 0.0;
  double driveAppliedVolts = 0.0;
  double turnAppliedVolts = 0.0;
};