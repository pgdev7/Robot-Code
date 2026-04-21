#include <thread>
#include <mutex>
#include <vector>
#include <shared_mutex>
#include <functional>
#include <queue>
#include <rev/SparkBase.h>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/CANBus.hpp>
#include <memory>
#include <units/angle.h>

class OdometryThread
{
public:
    static OdometryThread getInstance();

    void start();

    std::queue<double> registerSignal(rev::spark::SparkBase spark, std::function<double()> signal);
    std::queue<double> registerSignal(ctre::phoenix6::StatusSignal<units::radian_t> signal);
    std::queue<double> registerSignal(std::function<double()> signal);
    std::queue<double> makeTimestampQueue();

private:
    static OdometryThread instance;
    static bool isCANFD;

    std::vector<rev::spark::SparkBase> sparks;
    std::vector<std::function<double()>> sparkSignals;
    std::vector<ctre::phoenix6::BaseStatusSignal> phoenixSignals;
    std::vector<std::function<double()>> genericSignals;
    std::vector<std::queue<double>> sparkQueues;
    std::vector<std::queue<double>> phoenixQueues;
    std::vector<std::queue<double>> genericQueues;
    std::vector<std::queue<double>> timestampQueues;
};