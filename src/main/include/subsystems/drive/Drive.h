#pragma once

#include <frc2/command/SubsystemBase.h>
#include <mutex>

#include "GyroIO.h"
#include "ModuleIO.h"

class Drive : public frc2::SubsystemBase {
public:
  static std::mutex odometryLock;

  Drive();
  template <GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO,
            ModuleIO blModuleIO, ModuleIO brModuleIo>
  Drive() {}
};