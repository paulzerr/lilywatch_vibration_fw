#include "app.h"

#include <algorithm>

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <freertos/queue.h>
#include <freertos/task.h>

namespace app {
namespace {

constexpr char kTag[] = "haptic";

constexpr uint8_t kDrvRegMode = 0x01;
constexpr uint8_t kDrvRegRtpInput = 0x02;
constexpr uint8_t kDrvRegLibrary = 0x03;
constexpr uint8_t kDrvModeRealtime = 0x05;
constexpr uint8_t kDrvModeStandby = 0x40;
constexpr uint8_t kDrvLibraryErm = 0x01;
constexpr uint8_t kRtpAmplitude = 0x7F;

constexpr uint16_t kMinDurationMs = 10;
constexpr uint16_t kMaxDurationMs = 2000;
constexpr uint16_t kMaxOffTimeMs = 2000;
constexpr uint8_t kMaxRepeats = 10;
constexpr uint8_t kMaxHoldAwakeSeconds = 30;

struct HapticCommand {
  uint16_t durationMs;
  uint8_t repeats;
  uint16_t offTimeMs;
  uint8_t holdAwakeSeconds;
};

QueueHandle_t gQueue = nullptr;
StaticQueue_t gQueueState;
uint8_t gQueueStorage[sizeof(HapticCommand)];
portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;
bool gCommandQueuedOrRunning = false;
uint32_t gHoldAwakeUntilMs = 0;

bool holdActive(uint32_t now) {
  return static_cast<int32_t>(gHoldAwakeUntilMs - now) > 0;
}

uint32_t commandWindowMs(const HapticCommand &command) {
  const uint32_t repeats = std::max<uint8_t>(command.repeats, 1);
  const uint32_t activeMs = repeats * command.durationMs;
  const uint32_t offMs = repeats > 1 ? (repeats - 1) * command.offTimeMs : 0;
  return activeMs + offMs + static_cast<uint32_t>(command.holdAwakeSeconds) * 1000u + 500u;
}

void setCommandActive(bool active, uint32_t holdUntilMs) {
  portENTER_CRITICAL(&gMux);
  gCommandQueuedOrRunning = active;
  if (static_cast<int32_t>(holdUntilMs - gHoldAwakeUntilMs) > 0) {
    gHoldAwakeUntilMs = holdUntilMs;
  }
  portEXIT_CRITICAL(&gMux);
}

bool drvWrite(uint8_t reg, uint8_t value) {
  return boardHapticWrite(reg, value);
}

bool startDriver() {
  if (!boardHapticPower(true)) {
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(5));
  return drvWrite(kDrvRegLibrary, kDrvLibraryErm) && drvWrite(kDrvRegMode, kDrvModeRealtime) &&
         drvWrite(kDrvRegRtpInput, 0);
}

void stopDriver() {
  drvWrite(kDrvRegRtpInput, 0);
  drvWrite(kDrvRegMode, kDrvModeStandby);
  boardHapticPower(false);
}

void runCommand(const HapticCommand &command) {
  if (!startDriver()) {
    ESP_LOGW(kTag, "DRV2605 start failed");
    stopDriver();
    return;
  }

  for (uint8_t i = 0; i < command.repeats; ++i) {
    drvWrite(kDrvRegRtpInput, kRtpAmplitude);
    vTaskDelay(pdMS_TO_TICKS(command.durationMs));
    drvWrite(kDrvRegRtpInput, 0);
    if (i + 1 < command.repeats && command.offTimeMs > 0) {
      vTaskDelay(pdMS_TO_TICKS(command.offTimeMs));
    }
  }

  stopDriver();
}

void task(void *arg) {
  (void)arg;
  HapticCommand command = {};
  while (true) {
    if (xQueueReceive(gQueue, &command, portMAX_DELAY) == pdTRUE) {
      setCommandActive(true, uptimeMillis() + commandWindowMs(command));
      ESP_LOGI(kTag, "vibrate duration=%u repeats=%u off=%u hold=%u",
               static_cast<unsigned>(command.durationMs), static_cast<unsigned>(command.repeats),
               static_cast<unsigned>(command.offTimeMs),
               static_cast<unsigned>(command.holdAwakeSeconds));
      runCommand(command);
      const uint32_t holdUntil = uptimeMillis() + static_cast<uint32_t>(command.holdAwakeSeconds) * 1000u;
      portENTER_CRITICAL(&gMux);
      gCommandQueuedOrRunning = false;
      if (static_cast<int32_t>(holdUntil - gHoldAwakeUntilMs) > 0) {
        gHoldAwakeUntilMs = holdUntil;
      }
      portEXIT_CRITICAL(&gMux);
    }
  }
}

bool validCommand(const HapticCommand &command) {
  return command.durationMs >= kMinDurationMs && command.durationMs <= kMaxDurationMs &&
         command.repeats >= 1 && command.repeats <= kMaxRepeats &&
         command.offTimeMs <= kMaxOffTimeMs &&
         command.holdAwakeSeconds <= kMaxHoldAwakeSeconds;
}

}  // namespace

void hapticInit() {
  if (gQueue == nullptr) {
    gQueue = xQueueCreateStatic(1, sizeof(HapticCommand), gQueueStorage, &gQueueState);
    if (gQueue == nullptr || xTaskCreate(task, "haptic", 3072, nullptr, 4, nullptr) != pdPASS) {
      gQueue = nullptr;
    }
  }
}

bool hapticStartVibration(uint16_t durationMs, uint8_t repeats, uint16_t offTimeMs,
                          uint8_t holdAwakeSeconds) {
  HapticCommand command = {durationMs, repeats, offTimeMs, holdAwakeSeconds};
  if (!validCommand(command) || gQueue == nullptr) {
    return false;
  }

  const uint32_t holdUntil = uptimeMillis() + commandWindowMs(command);
  setCommandActive(true, holdUntil);
  if (xQueueSend(gQueue, &command, 0) != pdTRUE) {
    setCommandActive(false, uptimeMillis());
    return false;
  }
  return true;
}

bool hapticCommandActive() {
  const uint32_t now = uptimeMillis();
  portENTER_CRITICAL(&gMux);
  const bool active = gCommandQueuedOrRunning || holdActive(now);
  portEXIT_CRITICAL(&gMux);
  return active;
}

}  // namespace app
