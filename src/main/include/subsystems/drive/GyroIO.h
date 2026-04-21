#pragma once

#include <concepts>
#include <type_traits>

struct GyroIOInputs {
    bool connected = false;
};

template <typename T>
concept GyroIO = requires (T t) {
    std::is_void_v<decltype(t.updateInputs(GyroIOInputs{}))>;
};