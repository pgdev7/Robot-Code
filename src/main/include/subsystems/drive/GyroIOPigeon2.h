#include "Schematic.h"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "frc/Alert.h"
#include "subsystems/drive/GyroIO.h"
#include "units/acceleration.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/time.h"
#include "util/ArrayBlockingQueue.h"
#include <ctre/phoenix6/Pigeon2.hpp>
#include <memory>

class GyroIOPigeon2 {
public:
  GyroIOPigeon2();

  void updateInputs(GyroIOInputs &inputs);

private:
  struct Data {
    frc::Alert impactAlert =
        frc::Alert("Impact Detected, lowering elevator to prevent flipping.",
                   frc::Alert::AlertType::kWarning);
    ctre::phoenix6::hardware::Pigeon2 pigeon{Schematic.pigeonCanId};
    ctre::phoenix6::StatusSignal<units::degree_t> yaw = pigeon.GetYaw();
    std::shared_ptr<ArrayBlockingQueue<double>> yawPositionQueue;
    std::shared_ptr<ArrayBlockingQueue<units::second_t>> yawTimestampQueue;
    ctre::phoenix6::StatusSignal<units::degrees_per_second_t> yawVelocity =
        pigeon.GetAngularVelocityZWorld();
    ctre::phoenix6::StatusSignal<units::standard_gravity_t> accelX =
        pigeon.GetAccelerationX();
    ctre::phoenix6::StatusSignal<units::standard_gravity_t> accelY =
        pigeon.GetAccelerationY();
    ctre::phoenix6::StatusSignal<units::degree_t> robotPitch =
        pigeon.GetPitch();
    ctre::phoenix6::StatusSignal<units::degree_t> robotRoll = pigeon.GetRoll();
  };

  std::shared_ptr<Data> data = std::make_shared<Data>();
};

static_assert(GyroIO<GyroIOPigeon2>,
              "GyroIO based classes must implement all features");