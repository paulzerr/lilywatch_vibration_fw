#pragma once

#include <cstdint>

namespace app {
namespace cfg {

constexpr char kWatchId[] = "W00";

constexpr uint32_t kBootAwakeMs = 3000;
constexpr uint32_t kPowerLoopDelayMs = 1000;
constexpr int kPmMinCpuMhz = 40;
constexpr uint16_t kBleAdvIntervalMs = 500;

}  // namespace cfg
}  // namespace app
