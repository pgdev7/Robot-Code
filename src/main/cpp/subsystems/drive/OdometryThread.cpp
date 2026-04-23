#include "subsystems/drive/OdometryThread.h"

#include "ctre/phoenix6/StatusSignal.hpp"
#include "rev/REVLibError.h"
#include "rev/SparkBase.h"
#include "subsystems/drive/Drive.h"
#include "subsystems/drive/DriveConstants.h"
#include "units/time.h"
#include "util/ArrayBlockingQueue.h"
#include <chrono>
#include <ctre/phoenix6/CANBus.hpp>
#include <frc/RobotController.h>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

std::recursive_mutex signalsLock;
std::shared_ptr<OdometryThread> OdometryThread::instance;
std::thread thread;
bool OdometryThread::isCANFD = ctre::phoenix6::CANBus("rio").IsNetworkFD();

std::shared_ptr<OdometryThread> OdometryThread::getInstance() {
  return instance;
}

void OdometryThread::start() {
  if (timestampQueues.size() > 0) {
    thread = std::thread([&]() {});
    thread.detach();
  }
}
std::shared_ptr<ArrayBlockingQueue<double>>
OdometryThread::registerSignal(rev::spark::SparkBase *spark,
                               std::function<double()> signal) {
  std::shared_ptr<ArrayBlockingQueue<double>> queue =
      std::make_shared<ArrayBlockingQueue<double>>(20);
  signalsLock.lock();
  Drive::odometryLock.lock();
  try {
    sparks.push_back(spark);
    sparkSignals.push_back(signal);
    sparkQueues.emplace_back(std::move(queue));
  } catch (...) {
  }
  signalsLock.unlock();
  Drive::odometryLock.unlock();
  return queue;
}

/** Registers a Phoenix signal to be read from the thread. */
std::shared_ptr<ArrayBlockingQueue<double>>
OdometryThread::registerSignal(ctre::phoenix6::BaseStatusSignal *signal) {
  std::shared_ptr<ArrayBlockingQueue<double>> queue =
      std::make_shared<ArrayBlockingQueue<double>>(20);
  signalsLock.lock();
  Drive::odometryLock.lock();
  try {
    phoenixSignals.push_back(signal);
    phoenixQueues.push_back(queue);
  } catch (...) {
  }
  signalsLock.unlock();
  Drive::odometryLock.unlock();
  return queue;
}

/** Registers a generic signal to be read from the thread. */
std::shared_ptr<ArrayBlockingQueue<double>>
OdometryThread::registerSignal(std::function<double()> signal) {
  std::shared_ptr<ArrayBlockingQueue<double>> queue =
      std::make_shared<ArrayBlockingQueue<double>>(20);
  signalsLock.lock();
  Drive::odometryLock.lock();
  try {
    genericSignals.push_back(signal);
    genericQueues.push_back(queue);
  } catch (...) {
  }
  signalsLock.unlock();
  Drive::odometryLock.unlock();
  return queue;
}

/** Returns a new queue that returns timestamp values for each sample. */
std::shared_ptr<ArrayBlockingQueue<units::second_t>>
OdometryThread::makeTimestampQueue() {
  std::shared_ptr<ArrayBlockingQueue<units::second_t>> queue =
      std::make_shared<ArrayBlockingQueue<units::second_t>>(20);
  Drive::odometryLock.lock();
  try {
    timestampQueues.push_back(queue);
  } catch (...) {
  }
  Drive::odometryLock.unlock();
  return queue;
}

void OdometryThread::run() {
  while (true) {
    // Save new data to queues
    signalsLock.lock();
    try {
      if (isCANFD && phoenixSignals.size() > 0) {
        ctre::phoenix6::BaseStatusSignal::WaitForAll(
            2_s / DriveConstants::odometryFrequency, phoenixSignals);
      } else {
        // "waitForAll" does not support blocking on multiple signals with a bus
        // that is not CAN FD, regardless of Pro licensing. No reasoning for
        // this behavior is provided by the documentation.
        std::this_thread::sleep_for(std::chrono::milliseconds(
            (long)(1000.0 / DriveConstants::odometryFrequency)));
        if (phoenixSignals.size() > 0)
          ctre::phoenix6::BaseStatusSignal::RefreshAll(phoenixSignals);
      }
    } catch (...) {
    }
    signalsLock.unlock();

    // Save new data to queues
    Drive::odometryLock.lock();
    try {
      // Get sample timestamp
      units::second_t timestamp{
          units::microsecond_t(frc::RobotController::GetFPGATime())};

      // Read Spark values, mark invalid in case of error
      double *sparkValues = new double[sparkSignals.size()];
      bool isValid = true;
      for (int i = 0; i < sparkSignals.size(); i++) {
        sparkValues[i] = sparkSignals.at(i)();
        if (sparks.at(i)->GetLastError() != rev::REVLibError::kOk) {
          isValid = false;
        }
      }

      // Get phoenix timestamp
      units::second_t totalLatency{0.0};
      for (auto signal : phoenixSignals) {
        totalLatency += signal->GetTimestamp().GetLatency();
      }
      if (phoenixSignals.size() > 0) {
        timestamp -= totalLatency;
      }

      // If valid, add values to queues
      if (isValid) {
        for (int i = 0; i < sparkSignals.size(); i++) {
          sparkQueues.at(i)->offer(sparkValues[i]);
        }
        for (int i = 0; i < phoenixSignals.size(); i++) {
          phoenixQueues.at(i)->offer(phoenixSignals[i]->GetValueAsDouble());
        }
        for (int i = 0; i < genericSignals.size(); i++) {
          genericQueues.at(i)->offer(genericSignals.at(i)());
        }
        for (int i = 0; i < timestampQueues.size(); i++) {
          timestampQueues.at(i)->offer(timestamp);
        }
      }
    } catch (...) {
    }
    Drive::odometryLock.unlock();
  }
}