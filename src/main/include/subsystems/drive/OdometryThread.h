#include "units/time.h"
#include "util/ArrayBlockingQueue.h"
#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <functional>
#include <memory>
#include <rev/SparkBase.h>
#include <units/angle.h>
#include <vector>

class OdometryThread {
public:
  static std::shared_ptr<OdometryThread> getInstance();

  void start();

  std::shared_ptr<ArrayBlockingQueue<double>>
  registerSignal(rev::spark::SparkBase *spark, std::function<double()> signal);
  std::shared_ptr<ArrayBlockingQueue<double>>
  registerSignal(ctre::phoenix6::BaseStatusSignal *signal);
  std::shared_ptr<ArrayBlockingQueue<double>>
  registerSignal(std::function<double()> signal);
  std::shared_ptr<ArrayBlockingQueue<units::second_t>> makeTimestampQueue();

  void run();

private:
  static std::shared_ptr<OdometryThread> instance;
  static bool isCANFD;

  std::vector<rev::spark::SparkBase *> sparks;
  std::vector<std::function<double()>> sparkSignals;
  std::vector<ctre::phoenix6::BaseStatusSignal *> phoenixSignals;
  std::vector<std::function<double()>> genericSignals;
  std::vector<std::shared_ptr<ArrayBlockingQueue<double>>> sparkQueues;
  std::vector<std::shared_ptr<ArrayBlockingQueue<double>>> phoenixQueues;
  std::vector<std::shared_ptr<ArrayBlockingQueue<double>>> genericQueues;
  std::vector<std::shared_ptr<ArrayBlockingQueue<units::second_t>>>
      timestampQueues;
};