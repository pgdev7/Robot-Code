#pragma once

template <typename T> inline int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}
