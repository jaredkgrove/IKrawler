#pragma once
#include <iostream>
#include <string>

// Global counters (must be defined in the main test file)
extern int tests_failed;
extern int tests_run;

#define EXPECT_NEAR(val1, val2, abs_error)                                     \
  do {                                                                         \
    float diff = std::abs((val1) - (val2));                                    \
    if (diff > (abs_error)) {                                                  \
      std::cout << "[FAILED] " << #val1 << " (" << (val1) << ") != " << #val2  \
                << " (" << (val2) << ") within " << abs_error << "\n";         \
      std::cout << "         at " << __FILE__ << ":" << __LINE__ << "\n";      \
      tests_failed++;                                                          \
    }                                                                          \
  } while (0)

#define EXPECT_TRUE(condition)                                                 \
  do {                                                                         \
    if (!(condition)) {                                                        \
      std::cout << "[FAILED] " << #condition << " is false\n";                 \
      std::cout << "         at " << __FILE__ << ":" << __LINE__ << "\n";      \
      tests_failed++;                                                          \
    }                                                                          \
  } while (0)

#define EXPECT_FALSE(condition) EXPECT_TRUE(!(condition))

#define TEST(name) void name()

inline void PRINT_RESULT(const std::string &suiteName) {
  std::cout << "--------------------------------\n";
  if (tests_failed == 0) {
    std::cout << "ALL " << suiteName << " TESTS PASSED\n";
  } else {
    std::cout << tests_failed << " TESTS FAILED\n";
  }
}
