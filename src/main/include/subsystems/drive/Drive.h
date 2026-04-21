#pragma once

#include <frc2/command/SubsystemBase.h>

#include "GyroIO.h"
#include "ModuleIO.h"

class Drive : public frc2::SubsystemBase {
    public:
    Drive();
    template <GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIo brModuleIo>
    Drive() {

    }
};