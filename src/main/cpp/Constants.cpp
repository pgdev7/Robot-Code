#include "Constants.h"

#include <iostream>
#include <fstream>
#include <string>

#include <frc/RobotBase.h>
#include <frc/DriverStation.h>
#include <frc/Errors.h>

namespace Constants {
inline constexpr RobotType robotTypeOverride = RobotType::None;

inline const std::string robotFilename = "/home/lvuser/robot";
inline RobotType cachedRobotTypeFromRoborio = RobotType::None;

RobotType readRobotTypeFromRoborio() {
    if (robotTypeOverride != RobotType::None) {
        return robotTypeOverride;
    }
    RobotType robot = RobotType::None;
    std::string robotTypeString;

    std::ifstream file{robotFilename};
    if (file.is_open()) {
        std::getline(file, robotTypeString);
        if (robotTypeString == "ROBOT_2025_COMP") {
            robot = RobotType::ROBOT_2025_COMP;
        } else if (robotTypeString == "ROBOT_2025_TEST") {
            robot = RobotType::ROBOT_2025_TEST;
        } else if (robotTypeString == "ROBOT_BRIEFCASE") {
            robot = RobotType::ROBOT_BRIEFCASE;
        } else if (robotTypeString == "ROBOT_2026_COMP") {
            robot = RobotType::ROBOT_2026_COMP;
        } else if (robotTypeString == "ROBOT_SIMBOT") {
            robot = RobotType::ROBOT_SIMBOT;
        }
        file.close();

        if (robot == RobotType::None) {
            throw std::runtime_error("Read invalid RobotType value '" + robotTypeString + "' from file '" + robotFilename + "'.");
        }
    } else {
        throw std::runtime_error("Could not load robot type from file '" + robotFilename + "'.");
    }

    std::cout << "Using RobotType '" + robotTypeString + "'\n";

    cachedRobotTypeFromRoborio = robot;
    return robot;
}

RobotType getRobot() {
    if (cachedRobotTypeFromRoborio == RobotType::None) {
        cachedRobotTypeFromRoborio = readRobotTypeFromRoborio();
    }

    RobotType robot = cachedRobotTypeFromRoborio;

    if (frc::RobotBase::IsReal()) {
        if (robot == RobotType::ROBOT_SIMBOT) {
            FRC_ReportWarning("Invalid robot selected, using competition robot as default.", false);
            return RobotType::ROBOT_2026_COMP;
        } else {
            return robot;
        }
    }
}

Mode getMode() {
    switch (getRobot()) {
        case RobotType::ROBOT_BRIEFCASE:
        case RobotType::ROBOT_2025_COMP:
        case RobotType::ROBOT_2025_TEST:
        case RobotType::ROBOT_2026_COMP:
        return (frc::RobotBase::IsReal()) ? Mode::REAL : Mode::REPLAY;
        case RobotType::ROBOT_SIMBOT:
        return Mode::SIM;
        default:
        return Mode::REAL;
    }
}

void disableHAL() {
    HALDisabled = true;
}
void main(std::initializer_list<std::string> args)
{
    if (robotTypeOverride == RobotType::ROBOT_SIMBOT) {
        std::cerr << "Cannot deploy, invalid robot selected: ROBOT_SIMBOT\n";
        exit(1);
    }
}
}