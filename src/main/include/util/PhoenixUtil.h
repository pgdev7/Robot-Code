#pragma once

#include <ctre/phoenix/StatusCodes.h>
#include <functional>

namespace PhoenixUtil {
inline void tryUntilOk(int maxAttempts,
                       std::function<ctre::phoenix::StatusCode()> command) {
  for (int i = 0; i < maxAttempts; i++) {
    auto error = command();
    if (error.IsOK())
      break;
  }
}
} // namespace PhoenixUtil