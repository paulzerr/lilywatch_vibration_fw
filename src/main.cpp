#include "app.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"

namespace {

constexpr char kTag[] = "main";

app::PowerInputs readInputs(bool boardReady) {
  bool usbReadOk = false;
  const bool usbPresent = boardReady ? app::boardUsbPresent(&usbReadOk) : true;
  return {usbPresent,
          usbReadOk,
          app::bleConnected(),
          false,
          boardReady,
          app::hapticCommandActive() || app::bleCommandActive(),
          app::batteryLogExportActive()};
}

}  // namespace

extern "C" void app_main(void) {
  const bool boardReady = app::boardInit();
  app::powerInit();
  app::batteryLogInit();
  app::hapticInit();
  app::powerUpdate(readInputs(boardReady));
  app::bleInit();

  ESP_LOGI(kTag, "v5 firmware started");

  while (true) {
    app::batteryLogService();
    app::powerUpdate(readInputs(boardReady));
    vTaskDelay(pdMS_TO_TICKS(app::cfg::kPowerLoopDelayMs));
  }
}
