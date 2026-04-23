#include "subsystems/drive/ModuleIOSim.h"
#include "frc/geometry/Rotation2d.h"
#include "subsystems/drive/DriveConstants.h"
#include "subsystems/drive/ModuleIO.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/moment_of_inertia.h"
#include "units/time.h"
#include "units/voltage.h"
#include "util/Math.h"
#include <algorithm>
#include <cstdlib>
#include <frc/MathUtil.h>
#include <frc/Timer.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/math.h>

ModuleIOSim::ModuleIOSim()
    : driveSim(frc::LinearSystemId::DCMotorSystem(
                   DriveConstants::driveGearbox, 0.025_kg_sq_m,
                   DriveConstants::driveMotorReduction),
               DriveConstants::driveGearbox),
      turnSim(frc::LinearSystemId::DCMotorSystem(
                  DriveConstants::turnGearbox, 0.025_kg_sq_m,
                  DriveConstants::turnMotorReduction),
              DriveConstants::turnGearbox) {}

void ModuleIOSim::updateInputs(ModuleIOInputs &inputs) {
  if (driveClosedLoop) {
    driveAppliedVolts = driveFF + units::volt_t(driveController.Calculate(
                                      driveSim.GetAngularVelocity()()));
  } else {
    driveController.Reset();
  }

  if (turnClosedLoop) {
    turnAppliedVolts =
        units::volt_t(turnController.Calculate(turnSim.GetAngularPosition()()));
  } else {
    turnController.Reset();
  }

  driveSim.SetInputVoltage(std::clamp(driveAppliedVolts, -12_V, 12_V));
  turnSim.SetInputVoltage(std::clamp(turnAppliedVolts, -12_V, 12_V));
  driveSim.Update(0.02_s);
  turnSim.Update(0.02_s);

  inputs.driveConnected = true;
  inputs.drivePosition = driveSim.GetAngularPosition();
  inputs.driveVelocity = driveSim.GetAngularVelocity();
  inputs.driveAppliedVoltage = driveAppliedVolts;
  inputs.driveCurrent = units::ampere_t(std::abs(driveSim.GetCurrentDraw()()));

  inputs.turnConnected = true;
  inputs.turnPosition = frc::Rotation2d(turnSim.GetAngularPosition());
  inputs.turnVelocity = turnSim.GetAngularVelocity();
  inputs.turnAppliedVoltage = turnAppliedVolts;
  inputs.turnCurrent = units::ampere_t(std::abs(turnSim.GetCurrentDraw()()));

  inputs.odometryTimestamps = {frc::Timer::GetFPGATimestamp()()};
  inputs.odometryDrivePositionsRad = {inputs.drivePosition()};
  inputs.odometryTurnPositions = {inputs.turnPosition};
}

void ModuleIOSim::setDriveOpenLoop(units::volt_t output) {
  driveClosedLoop = false;
  driveAppliedVolts = output;
}

void ModuleIOSim::setTurnOpenLoop(units::volt_t output) {
  turnClosedLoop = false;
  turnAppliedVolts = output;
}

void ModuleIOSim::setDriveVelocity(
    units::angular_velocity::radians_per_second_t velocity) {
  driveClosedLoop = true;
  driveFF =
      units::volt_t(DriveConstants::driveSimKs) * units::volt_t(sgn(velocity)) +
      units::volt_t(DriveConstants::driveSimKv) * units::volt_t(velocity());
  driveController.SetSetpoint(velocity());
}

void ModuleIOSim::setTurnPosition(frc::Rotation2d rotation) {
  turnClosedLoop = true;
  turnController.SetSetpoint(rotation.Radians()());
}