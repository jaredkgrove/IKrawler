#pragma once

namespace Utils {

/**
 * @brief Clamp a value between min and max
 */
template <typename T> constexpr T clamp(T value, T min, T max) {
  return (value < min) ? min : (value > max) ? max : value;
}

/**
 * @brief Wrap a phase value to keep it within [0.0, 1.0)
 *
 * Useful for gait phases that cycle continuously.
 */
inline float wrapPhase(float phase) {
  while (phase >= 1.0f) {
    phase -= 1.0f;
  }
  while (phase < 0.0f) {
    phase += 1.0f;
  }
  return phase;
}

} // namespace Utils
