#pragma once

#include "Constants.h"

struct _Schematic
{
    // Drive (CAN Ids)
    const int pigeonCanId;

    const int frontLeftDriveCanId;
    const int backLeftDriveCanId;
    const int frontRightDriveCanId;
    const int backRightDriveCanId;

    const int frontLeftTurnCanId;
    const int backLeftTurnCanId;
    const int frontRightTurnCanId;
    const int backRightTurnCanId;

    const int frontLeftAbsoluteEncoderCanId;
    const int backLeftAbsoluteEncoderCanId;
    const int frontRightAbsoluteEncoderCanId;
    const int backRightAbsoluteEncoderCanId;

    // shooter subsystem
    const int shooterTopLeftMotorCanId;
    const int shooterTopRightMotorCanId;
    const int shooterBottomLeftMotorCanId;
    const int shooterBottomRightMotorCanId;

    // magic carpet sub-system
    const int magicCarpetCanId;

    //  indexer subsystem
    const int indexerFeedupCanId;

    // intake subsystem
    const int intakeMotorCanId;
    const int intakeExtendCanId;

    // climber subsystem
    const int climberMotorCanId;
};

inline _Schematic schematicInitialize()
{
    switch (Constants::getRobot())
    {
    case RobotType::ROBOT_2026_COMP:
        return {17, 5, 6, 3, 4, 1, 2, 7, 8, 20, 19, 18, 21, 33, 35, 34, 36, 15, 9, 23, 14, 22};
    }
    return {};
}

inline const _Schematic Schematic = schematicInitialize();