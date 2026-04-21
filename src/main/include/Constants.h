// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

#include <frc/RobotBase.h>
#include <unordered_map>
#include <string>
#include <initializer_list>

enum class RobotType
{
    None,
    ROBOT_2025_COMP,
    ROBOT_2025_TEST,
    ROBOT_BRIEFCASE,
    ROBOT_2026_COMP,
    ROBOT_SIMBOT
};

enum class Mode
{
    REAL,
    REPLAY,
    SIM
};

namespace Constants
{

    inline constexpr bool tuningMode = false;

    inline const std::unordered_map<RobotType, std::string> logFolders = {
        {RobotType::ROBOT_BRIEFCASE, "/media/sda1"},
        {RobotType::ROBOT_2026_COMP, "/media/sda1"},
        {RobotType::ROBOT_2025_COMP, "/media/sda1"}};

    inline bool HALDisabled = false;

    RobotType getRobot();

    Mode getMode();

    void disableHAL();

    void main(std::initializer_list<std::string> args);

} // namespace Constants
