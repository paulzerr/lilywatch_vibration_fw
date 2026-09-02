#include "app.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <esp_err.h>
#include <esp_idf_version.h>
#include <esp_pm.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <esp_private/pm_impl.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <sdkconfig.h>

#include "config.h"

namespace app {
namespace {

#if defined(CONFIG_PM_ENABLE) && CONFIG_PM_ENABLE
constexpr bool kBuildPmEnabled = true;
#else
constexpr bool kBuildPmEnabled = false;
#endif

#if defined(CONFIG_FREERTOS_USE_TICKLESS_IDLE) && CONFIG_FREERTOS_USE_TICKLESS_IDLE
constexpr bool kBuildTicklessEnabled = true;
#else
constexpr bool kBuildTicklessEnabled = false;
#endif

#if (defined(CONFIG_BT_CTRL_MODEM_SLEEP) && CONFIG_BT_CTRL_MODEM_SLEEP) ||       \
    (defined(CONFIG_BTDM_CTRL_MODEM_SLEEP) && CONFIG_BTDM_CTRL_MODEM_SLEEP) ||   \
    (defined(CONFIG_BT_CTRL_SLEEP_MODE_EFF) && CONFIG_BT_CTRL_SLEEP_MODE_EFF) || \
    (defined(CONFIG_BT_LE_SLEEP_ENABLE) && CONFIG_BT_LE_SLEEP_ENABLE)
constexpr bool kBuildBleSleepConfigured = true;
#else
constexpr bool kBuildBleSleepConfigured = false;
#endif

#if defined(CONFIG_LILY_POWER_DIAGNOSTICS) && CONFIG_LILY_POWER_DIAGNOSTICS
constexpr bool kBuildPowerDiagnosticsEnabled = true;
#else
constexpr bool kBuildPowerDiagnosticsEnabled = false;
#endif

#if defined(CONFIG_PM_PROFILING) && CONFIG_PM_PROFILING
constexpr bool kBuildPmProfilingEnabled = true;
#else
constexpr bool kBuildPmProfilingEnabled = false;
#endif

#if defined(CONFIG_LILY_POWER_DIAGNOSTICS) && CONFIG_LILY_POWER_DIAGNOSTICS && \
    defined(CONFIG_PM_PROFILING) && CONFIG_PM_PROFILING
#define APP_POWER_DIAG_PM_STATS 1
#else
#define APP_POWER_DIAG_PM_STATS 0
#endif

struct PowerRuntime {
  esp_pm_lock_handle_t lock = nullptr;
  bool lockHeld = false;
  bool idleLockReleasedOnce = false;
  uint32_t noLightSleepReleaseCount = 0;
  uint32_t bootHoldUntil = 0;
  esp_err_t pmConfigResult = ESP_ERR_NOT_SUPPORTED;
  esp_err_t lockCreateResult = ESP_ERR_NOT_SUPPORTED;
  esp_err_t lockApplyResult = ESP_OK;
  uint8_t resetReason = 0;
  uint8_t wakeCause = 0;
  PowerSnapshot snapshot = {PowerState::Awake,
                            kFailIdleNotObserved,
                            0,
                            0,
                            err16(ESP_ERR_NOT_SUPPORTED),
                            err16(ESP_ERR_NOT_SUPPORTED),
                            0,
                            0,
                            0,
                            kBleLpClockUnknown,
                            0};
};

PowerRuntime g;
portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;

bool bootHoldActive() {
  return static_cast<int32_t>(g.bootHoldUntil - uptimeMillis()) > 0;
}

uint8_t bleLpClockSource() {
#if (defined(CONFIG_BT_CTRL_LPCLK_SEL_MAIN_XTAL) && CONFIG_BT_CTRL_LPCLK_SEL_MAIN_XTAL) || \
    (defined(CONFIG_BT_CTRL_LOW_POWER_CLOCK_MAIN_XTAL) && CONFIG_BT_CTRL_LOW_POWER_CLOCK_MAIN_XTAL)
  return kBleLpClockMainXtal;
#elif (defined(CONFIG_BT_CTRL_LPCLK_SEL_EXT_32K_XTAL) && CONFIG_BT_CTRL_LPCLK_SEL_EXT_32K_XTAL) || \
    (defined(CONFIG_BT_CTRL_LOW_POWER_CLOCK_EXTERNAL_32K_XTAL) && \
     CONFIG_BT_CTRL_LOW_POWER_CLOCK_EXTERNAL_32K_XTAL)
  return kBleLpClockExternal32k;
#elif (defined(CONFIG_BT_CTRL_LPCLK_SEL_8M) && CONFIG_BT_CTRL_LPCLK_SEL_8M) || \
    (defined(CONFIG_BT_CTRL_LOW_POWER_CLOCK_RC_FAST) && CONFIG_BT_CTRL_LOW_POWER_CLOCK_RC_FAST)
  return kBleLpClockInternalRc;
#else
  return kBleLpClockUnknown;
#endif
}

void applyLock(bool holdLock) {
  if (g.lock == nullptr || holdLock == g.lockHeld) {
    return;
  }

  g.lockApplyResult = holdLock ? esp_pm_lock_acquire(g.lock) : esp_pm_lock_release(g.lock);
  if (g.lockApplyResult == ESP_OK) {
    g.lockHeld = holdLock;
    if (!holdLock) {
      g.idleLockReleasedOnce = true;
      ++g.noLightSleepReleaseCount;
    }
  }
}

FailureReason baseFailure(const PowerInputs &inputs) {
  if (!inputs.boardReady) {
    return kFailBoardInit;
  }
  if (!inputs.usbReadOk) {
    return kFailUsbRead;
  }
  if (!kBuildPmEnabled) {
    return kFailPmBuildDisabled;
  }
  if (!kBuildTicklessEnabled) {
    return kFailTicklessDisabled;
  }
  if (g.pmConfigResult != ESP_OK) {
    return kFailPmConfig;
  }
  if (g.lockCreateResult != ESP_OK || g.lock == nullptr) {
    return kFailLockCreate;
  }
  if (!kBuildBleSleepConfigured) {
    return kFailBleSleepDisabled;
  }
  if (g.lockApplyResult != ESP_OK) {
    return kFailLockApply;
  }
  return kFailNone;
}

uint32_t reasonFlags(const PowerInputs &inputs, FailureReason failure) {
  uint32_t reasons = 0;
  if (inputs.usbPresent) {
    reasons |= kReasonUsbPresent;
  }
  if (!inputs.usbReadOk) {
    reasons |= kReasonUsbReadFailed;
  }
  if (inputs.bleConnected) {
    reasons |= kReasonBleConnected;
  }
  if (inputs.bootHold) {
    reasons |= kReasonBootHold;
  }
  if (inputs.commandActive) {
    reasons |= kReasonCommandActive;
  }
  if (inputs.exportActive) {
    reasons |= kReasonExportActive;
  }
  if (!inputs.boardReady) {
    reasons |= kReasonBoardInitFailed;
  }
  if (failure != kFailNone) {
    reasons |= kReasonPowerFailure;
  }
  return reasons;
}

uint32_t diagFlags(const PowerInputs &inputs, FailureReason runtimeFailure) {
  uint32_t diag = 0;
  if (kBuildPmEnabled) {
    diag |= kDiagPmBuildEnabled;
  }
  if (kBuildTicklessEnabled) {
    diag |= kDiagTicklessEnabled;
  }
  if (kBuildBleSleepConfigured) {
    diag |= kDiagBleSleepConfigured;
  }
  if (g.pmConfigResult == ESP_OK) {
    diag |= kDiagPmConfigured;
  }
  if (g.lockCreateResult == ESP_OK && g.lock != nullptr) {
    diag |= kDiagLockCreated;
  }
  if (g.lockHeld) {
    diag |= kDiagLockHeld;
  }
  if (g.idleLockReleasedOnce) {
    diag |= kDiagIdleLockReleasedOnce;
  }
  if (inputs.usbReadOk) {
    diag |= kDiagUsbReadOk;
  }
  if (inputs.boardReady) {
    diag |= kDiagBoardReady;
  }
  if (kBuildPowerDiagnosticsEnabled) {
    diag |= kDiagPowerDiagnosticsAvailable;
  }
  if (kBuildPmProfilingEnabled) {
    diag |= kDiagPowerDiagnosticsProfiling;
  }
  if (runtimeFailure == kFailNone && g.idleLockReleasedOnce) {
    diag |= kDiagPowerSavingOk;
  }
  return diag;
}

uint16_t saturate16(uint32_t value) {
  return value > 0xFFFFu ? 0xFFFFu : static_cast<uint16_t>(value);
}

#if APP_POWER_DIAG_PM_STATS
uint32_t usToMsSaturated(uint64_t us) {
  const uint64_t ms = us / 1000u;
  return ms > 0xFFFFFFFFu ? 0xFFFFFFFFu : static_cast<uint32_t>(ms);
}

bool parseModeTimeUs(const char *dump, const char *mode, uint64_t *out) {
  if (dump == nullptr || mode == nullptr || out == nullptr) {
    return false;
  }

  const size_t modeLen = std::strlen(mode);
  const char *line = dump;
  while (*line != '\0') {
    while (*line == '\n' || *line == '\r') {
      ++line;
    }
    while (*line != '\0' && std::isspace(static_cast<unsigned char>(*line)) && *line != '\n') {
      ++line;
    }
    if (std::strncmp(line, mode, modeLen) == 0 &&
        std::isspace(static_cast<unsigned char>(line[modeLen]))) {
      const char *cursor = line + modeLen;
      while (*cursor != '\0' && std::isspace(static_cast<unsigned char>(*cursor)) &&
             *cursor != '\n') {
        ++cursor;
      }
      while (*cursor != '\0' && !std::isspace(static_cast<unsigned char>(*cursor))) {
        ++cursor;
      }
      while (*cursor != '\0' && std::isspace(static_cast<unsigned char>(*cursor)) &&
             *cursor != '\n') {
        ++cursor;
      }

      char *end = nullptr;
      const unsigned long long value = std::strtoull(cursor, &end, 10);
      if (end != cursor) {
        *out = static_cast<uint64_t>(value);
        return true;
      }
      return false;
    }

    const char *next = std::strchr(line, '\n');
    if (next == nullptr) {
      break;
    }
    line = next + 1;
  }

  return false;
}
#endif

}  // namespace

void powerInit() {
  g.resetReason = static_cast<uint8_t>(esp_reset_reason());
  g.wakeCause = static_cast<uint8_t>(esp_sleep_get_wakeup_cause());
  g.bootHoldUntil = uptimeMillis() + cfg::kBootAwakeMs;

#if defined(CONFIG_PM_ENABLE) && CONFIG_PM_ENABLE
  esp_pm_config_esp32s3_t config = {};
  config.max_freq_mhz = 80;
  config.min_freq_mhz = cfg::kPmMinCpuMhz;
  config.light_sleep_enable = true;
  if (config.min_freq_mhz > config.max_freq_mhz) {
    config.min_freq_mhz = config.max_freq_mhz;
  }

  g.pmConfigResult = esp_pm_configure(&config);
  if (g.pmConfigResult == ESP_OK) {
    g.lockCreateResult = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "active", &g.lock);
    if (g.lockCreateResult == ESP_OK) {
      applyLock(true);
    }
  }
#endif
}

void powerUpdate(const PowerInputs &inputs) {
  PowerInputs local = inputs;
  local.bootHold = inputs.bootHold || bootHoldActive();

  FailureReason runtimeFailure = baseFailure(local);
  const bool mustStayAwake =
      local.usbPresent || local.bleConnected || local.bootHold || local.commandActive ||
      local.exportActive || runtimeFailure != kFailNone;
  PowerState state = runtimeFailure != kFailNone
                         ? PowerState::PowerSavingFailed
                         : (mustStayAwake ? PowerState::Awake : PowerState::BatteryIdleReachable);

  applyLock(state != PowerState::BatteryIdleReachable);
  runtimeFailure = baseFailure(local);
  if (runtimeFailure != kFailNone) {
    state = PowerState::PowerSavingFailed;
  }

  FailureReason reportFailure = runtimeFailure;
  if (reportFailure == kFailNone && !g.idleLockReleasedOnce) {
    reportFailure = kFailIdleNotObserved;
  }

  PowerSnapshot snapshot = {state,
                            reportFailure,
                            reasonFlags(local, runtimeFailure),
                            diagFlags(local, runtimeFailure),
                            err16(g.pmConfigResult),
                            err16(g.lockCreateResult),
                            err16(g.lockApplyResult),
                            g.resetReason,
                            g.wakeCause,
                            bleLpClockSource(),
                            boardBatteryMillivolts()};

  portENTER_CRITICAL(&gMux);
  g.snapshot = snapshot;
  portEXIT_CRITICAL(&gMux);
}

PowerSnapshot powerSnapshot() {
  portENTER_CRITICAL(&gMux);
  const PowerSnapshot snapshot = g.snapshot;
  portEXIT_CRITICAL(&gMux);
  return snapshot;
}

PowerDiagnosticsSnapshot powerDiagnosticsSnapshot() {
  PowerDiagnosticsSnapshot snapshot = {};
  snapshot.statsResult = err16(ESP_ERR_NOT_SUPPORTED);
  snapshot.uptimeSeconds = uptimeMillis() / 1000u;

  portENTER_CRITICAL(&gMux);
  snapshot.noLightSleepLockCount = g.lockHeld ? 1 : 0;
  snapshot.noLightSleepReleaseCount = saturate16(g.noLightSleepReleaseCount);
  portEXIT_CRITICAL(&gMux);

  if (kBuildPowerDiagnosticsEnabled) {
    snapshot.flags |= kPowerDiagBuildEnabled;
  }
  if (kBuildPmProfilingEnabled) {
    snapshot.flags |= kPowerDiagPmProfilingEnabled;
  }

#if APP_POWER_DIAG_PM_STATS
  constexpr size_t kStatsBufferSize = 512;
  char dump[kStatsBufferSize] = {};
  FILE *stream = fmemopen(dump, sizeof(dump), "w");
  if (stream == nullptr) {
    snapshot.statsResult = err16(ESP_FAIL);
    return snapshot;
  }

  esp_pm_impl_dump_stats(stream);
  const int closeResult = std::fclose(stream);
  snapshot.statsResult = err16(closeResult == 0 ? ESP_OK : ESP_FAIL);
  if (std::strlen(dump) >= sizeof(dump) - 1) {
    snapshot.flags |= kPowerDiagDumpTruncated;
  }

  uint64_t sleepUs = 0;
  uint64_t apbMinUs = 0;
  uint64_t apbMaxUs = 0;
  uint64_t cpuMaxUs = 0;
  const bool sleepParsed = parseModeTimeUs(dump, "SLEEP", &sleepUs);
  const bool apbMinParsed = parseModeTimeUs(dump, "APB_MIN", &apbMinUs);
  const bool apbMaxParsed = parseModeTimeUs(dump, "APB_MAX", &apbMaxUs);
  const bool cpuMaxParsed = parseModeTimeUs(dump, "CPU_MAX", &cpuMaxUs);

  if (sleepParsed) {
    snapshot.flags |= kPowerDiagSleepStatsSeen;
  }
  if (apbMinParsed) {
    snapshot.flags |= kPowerDiagApbMinStatsSeen;
  }
  if (apbMaxParsed) {
    snapshot.flags |= kPowerDiagApbMaxStatsSeen;
  }
  if (cpuMaxParsed) {
    snapshot.flags |= kPowerDiagCpuMaxStatsSeen;
  }

  const bool parsedAllModes = sleepParsed && apbMinParsed && apbMaxParsed && cpuMaxParsed;
  if (parsedAllModes && closeResult == 0) {
    const uint64_t totalUs = sleepUs + apbMinUs + apbMaxUs + cpuMaxUs;
    snapshot.flags |= kPowerDiagStatsValid;
    if (sleepUs > 0) {
      snapshot.flags |= kPowerDiagLightSleepSeen;
    }
    snapshot.totalProfiledMs = usToMsSaturated(totalUs);
    snapshot.lightSleepMs = usToMsSaturated(sleepUs);
    snapshot.apbMinMs = usToMsSaturated(apbMinUs);
    snapshot.cpuMaxMs = usToMsSaturated(cpuMaxUs);
    if (totalUs > 0) {
      uint64_t percentX100 = (sleepUs * 10000u) / totalUs;
      if (percentX100 > 10000u) {
        percentX100 = 10000u;
      }
      snapshot.lightSleepPercentX100 = static_cast<uint16_t>(percentX100);
    }
  } else {
    snapshot.flags |= kPowerDiagParseFailed;
  }

#endif

  return snapshot;
}

}  // namespace app
