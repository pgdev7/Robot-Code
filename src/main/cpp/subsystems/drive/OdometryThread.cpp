#include "subsystems/drive/OdometryThread.h"

#include <ctre/phoenix6/CANBus.hpp>
#include <memory>
#include <mutex>
#include <thread>

std::recursive_mutex signalsLock;
OdometryThread OdometryThread::instance;
std::thread thread;
bool OdometryThread::isCANFD = ctre::phoenix6::CANBus("rio").IsNetworkFD();

OdometryThread OdometryThread::getInstance()
{
    return instance;
}

void OdometryThread::start()
{
    if (timestampQueues.size() > 0)
    {
        thread = std::thread([&]() {});
        thread.detach();
    }
}
std::queue<double> OdometryThread::registerSignal(rev::spark::SparkBase spark, std::function<double()> signal)
{
    std::queue<double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    Drive.odometryLock.lock();
    try
    {
        sparks.add(spark);
        sparkSignals.add(signal);
        sparkQueues.add(queue);
    }
    catch (...)
    {
    }
    signalsLock.unlock();
    Drive.odometryLock.unlock();
    return queue;
}