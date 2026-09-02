#include "app.h"

#include <cassert>
#include <cstring>

#include <esp_bt.h>
#include <esp_err.h>
#include <esp_idf_version.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <host/ble_att.h>
#include <host/ble_hs.h>
#include <host/ble_uuid.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <nvs_flash.h>
#include <os/os_mbuf.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
#include <esp_nimble_hci.h>
#endif

#include "config.h"

namespace app {
namespace {

constexpr char kTag[] = "ble";
constexpr uint16_t kAdvIntervalUnits = static_cast<uint16_t>(cfg::kBleAdvIntervalMs * 8 / 5);

const ble_uuid128_t kServiceUuid =
    BLE_UUID128_INIT(0x11, 0x28, 0x7b, 0xe5, 0xf2, 0x97, 0xe6, 0xa4, 0xbf, 0x4e, 0xdd,
                     0x3f, 0xd0, 0xf6, 0xc0, 0xf9);
const ble_uuid128_t kStatusUuid =
    BLE_UUID128_INIT(0x05, 0x05, 0x9d, 0x9f, 0x8b, 0x9d, 0x79, 0x9f, 0x5b, 0x4c, 0x39,
                     0x6c, 0x1f, 0x57, 0x6f, 0x0d);
const ble_uuid128_t kPowerDiagnosticsUuid =
    BLE_UUID128_INIT(0x05, 0x05, 0x9d, 0x9f, 0x8b, 0x9d, 0x79, 0x9f, 0x5b, 0x4c, 0x39,
                     0x6c, 0x20, 0x57, 0x6f, 0x0d);
const ble_uuid128_t kCommandUuid =
    BLE_UUID128_INIT(0x05, 0x05, 0x9d, 0x9f, 0x8b, 0x9d, 0x79, 0x9f, 0x5b, 0x4c, 0x39,
                     0x6c, 0x21, 0x57, 0x6f, 0x0d);
const ble_uuid128_t kBatteryLatestUuid =
    BLE_UUID128_INIT(0x05, 0x05, 0x9d, 0x9f, 0x8b, 0x9d, 0x79, 0x9f, 0x5b, 0x4c, 0x39,
                     0x6c, 0x22, 0x57, 0x6f, 0x0d);
const ble_uuid128_t kLogCtrlUuid =
    BLE_UUID128_INIT(0x69, 0x0c, 0x6f, 0x6d, 0x3f, 0xa4, 0xe9, 0x9a, 0xd1, 0x4d, 0x85,
                     0xc4, 0x13, 0x21, 0xa7, 0xb3);
const ble_uuid128_t kLogDataUuid =
    BLE_UUID128_INIT(0x5d, 0x6a, 0x1c, 0x17, 0xa4, 0x61, 0xd5, 0x8b, 0xab, 0x42, 0x76,
                     0xe9, 0x8d, 0x3a, 0x30, 0x67);

bool gConnected = false;
uint32_t gCommandHoldUntilMs = 0;
uint8_t gOwnAddrType = 0;
portMUX_TYPE gBleMux = portMUX_INITIALIZER_UNLOCKED;

void setConnected(bool connected) {
  portENTER_CRITICAL(&gBleMux);
  gConnected = connected;
  portEXIT_CRITICAL(&gBleMux);
}

void holdCommandAwake(uint8_t seconds) {
  if (seconds == 0) {
    return;
  }
  const uint32_t until = uptimeMillis() + static_cast<uint32_t>(seconds) * 1000u;
  portENTER_CRITICAL(&gBleMux);
  if (static_cast<int32_t>(until - gCommandHoldUntilMs) > 0) {
    gCommandHoldUntilMs = until;
  }
  portEXIT_CRITICAL(&gBleMux);
}

size_t buildStatus(uint8_t *out, size_t capacity) {
  if (out == nullptr || capacity < kStatusPacketSize) {
    return 0;
  }

  const PowerSnapshot power = powerSnapshot();
  out[0] = kPacketStatus;
  out[1] = kStatusSchema;
  out[2] = static_cast<uint8_t>(power.state);
  out[3] = static_cast<uint8_t>(power.failure);
  writeLe32(&out[4], power.reasons);
  writeLe32(&out[8], power.diag);
  writeLe16(&out[12], static_cast<uint16_t>(power.pmConfigResult));
  writeLe16(&out[14], static_cast<uint16_t>(power.lockCreateResult));
  writeLe16(&out[16], static_cast<uint16_t>(power.lockApplyResult));
  out[18] = power.resetReason;
  out[19] = power.wakeCause;
  out[20] = power.bleLpClockSource;
  out[21] = 0;
  writeLe16(&out[22], power.batteryMillivolts);
  return kStatusPacketSize;
}

size_t buildPowerDiagnostics(uint8_t *out, size_t capacity) {
  if (out == nullptr || capacity < kPowerDiagnosticsPacketSize) {
    return 0;
  }

  const PowerDiagnosticsSnapshot diagnostics = powerDiagnosticsSnapshot();
  out[0] = kPacketPowerDiagnostics;
  out[1] = kPowerDiagnosticsSchema;
  writeLe16(&out[2], diagnostics.flags);
  writeLe16(&out[4], static_cast<uint16_t>(diagnostics.statsResult));
  writeLe32(&out[6], diagnostics.uptimeSeconds);
  writeLe32(&out[10], diagnostics.totalProfiledMs);
  writeLe32(&out[14], diagnostics.lightSleepMs);
  writeLe32(&out[18], diagnostics.apbMinMs);
  writeLe32(&out[22], diagnostics.cpuMaxMs);
  writeLe16(&out[26], diagnostics.lightSleepPercentX100);
  writeLe16(&out[28], diagnostics.noLightSleepLockCount);
  writeLe16(&out[30], diagnostics.noLightSleepReleaseCount);
  return kPowerDiagnosticsPacketSize;
}

uint16_t readLe16(const uint8_t *in) {
  return static_cast<uint16_t>(in[0]) | (static_cast<uint16_t>(in[1]) << 8);
}

uint32_t readLe32(const uint8_t *in) {
  return static_cast<uint32_t>(in[0]) | (static_cast<uint32_t>(in[1]) << 8) |
         (static_cast<uint32_t>(in[2]) << 16) | (static_cast<uint32_t>(in[3]) << 24);
}

int copyWritePayload(ble_gatt_access_ctxt *ctxt, uint8_t *out, uint16_t capacity,
                     uint16_t *length) {
  const uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
  if (len > capacity) {
    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
  }
  const int rc = os_mbuf_copydata(ctxt->om, 0, len, out);
  if (rc != 0) {
    return BLE_ATT_ERR_UNLIKELY;
  }
  *length = len;
  return 0;
}

int statusAccess(uint16_t connHandle, uint16_t attrHandle, ble_gatt_access_ctxt *ctxt, void *arg) {
  (void)connHandle;
  (void)attrHandle;
  (void)arg;
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
    return BLE_ATT_ERR_UNLIKELY;
  }

  uint8_t packet[kStatusPacketSize] = {};
  const size_t len = buildStatus(packet, sizeof(packet));
  return os_mbuf_append(ctxt->om, packet, len) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

int powerDiagnosticsAccess(uint16_t connHandle, uint16_t attrHandle, ble_gatt_access_ctxt *ctxt,
                           void *arg) {
  (void)connHandle;
  (void)attrHandle;
  (void)arg;
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
    return BLE_ATT_ERR_UNLIKELY;
  }

  uint8_t packet[kPowerDiagnosticsPacketSize] = {};
  const size_t len = buildPowerDiagnostics(packet, sizeof(packet));
  return os_mbuf_append(ctxt->om, packet, len) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

int batteryLatestAccess(uint16_t connHandle, uint16_t attrHandle, ble_gatt_access_ctxt *ctxt,
                        void *arg) {
  (void)connHandle;
  (void)attrHandle;
  (void)arg;
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
    return BLE_ATT_ERR_UNLIKELY;
  }

  uint8_t packet[kBatteryLatestPacketSize] = {};
  const size_t len = batteryLogBuildLatestPacket(packet, sizeof(packet));
  return os_mbuf_append(ctxt->om, packet, len) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

int commandAccess(uint16_t connHandle, uint16_t attrHandle, ble_gatt_access_ctxt *ctxt, void *arg) {
  (void)connHandle;
  (void)attrHandle;
  (void)arg;
  if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
    return BLE_ATT_ERR_UNLIKELY;
  }

  uint8_t payload[7] = {};
  uint16_t len = 0;
  const int copyResult = copyWritePayload(ctxt, payload, sizeof(payload), &len);
  if (copyResult != 0) {
    return copyResult;
  }
  switch (payload[0]) {
    case kCommandVibrate: {
      if (len != 7) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
      }
      const uint16_t durationMs = readLe16(&payload[1]);
      const uint8_t repeats = payload[3];
      const uint16_t offTimeMs = readLe16(&payload[4]);
      const uint8_t holdAwakeSeconds = payload[6];
      return hapticStartVibration(durationMs, repeats, offTimeMs, holdAwakeSeconds)
                 ? 0
                 : BLE_ATT_ERR_UNLIKELY;
    }

    case kCommandHoldAwake:
      if (len != 2 || payload[1] > 30) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
      }
      holdCommandAwake(payload[1]);
      return 0;

    default:
      return BLE_ATT_ERR_UNLIKELY;
  }
}

int logControlAccess(uint16_t connHandle, uint16_t attrHandle, ble_gatt_access_ctxt *ctxt,
                     void *arg) {
  (void)connHandle;
  (void)attrHandle;
  (void)arg;
  if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
    return BLE_ATT_ERR_UNLIKELY;
  }

  uint8_t payload[5] = {};
  uint16_t len = 0;
  const int copyResult = copyWritePayload(ctxt, payload, sizeof(payload), &len);
  if (copyResult != 0) {
    return copyResult;
  }
  if (len < 1) {
    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
  }

  switch (payload[0]) {
    case 0x01:
      if (len != 1) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
      }
      batteryLogBeginExport();
      return 0;

    case 0x02:
      if (len != 1) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
      }
      batteryLogClear();
      return 0;

    case 0x03:
      if (len != 5) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
      }
      return batteryLogSetTime(readLe32(&payload[1])) ? 0 : BLE_ATT_ERR_UNLIKELY;

    default:
      return BLE_ATT_ERR_UNLIKELY;
  }
}

int logDataAccess(uint16_t connHandle, uint16_t attrHandle, ble_gatt_access_ctxt *ctxt,
                  void *arg) {
  (void)connHandle;
  (void)attrHandle;
  (void)arg;
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
    return BLE_ATT_ERR_UNLIKELY;
  }

  uint8_t packet[kLogDataPacketSize] = {};
  const size_t len = batteryLogBuildNextPacket(packet, sizeof(packet));
  return os_mbuf_append(ctxt->om, packet, len) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

const ble_gatt_chr_def kCharacteristics[] = {
    {
        .uuid = &kStatusUuid.u,
        .access_cb = statusAccess,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_READ,
        .min_key_size = 0,
        .val_handle = nullptr,
    },
    {
        .uuid = &kPowerDiagnosticsUuid.u,
        .access_cb = powerDiagnosticsAccess,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_READ,
        .min_key_size = 0,
        .val_handle = nullptr,
    },
    {
        .uuid = &kCommandUuid.u,
        .access_cb = commandAccess,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_WRITE,
        .min_key_size = 0,
        .val_handle = nullptr,
    },
    {
        .uuid = &kBatteryLatestUuid.u,
        .access_cb = batteryLatestAccess,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_READ,
        .min_key_size = 0,
        .val_handle = nullptr,
    },
    {
        .uuid = &kLogCtrlUuid.u,
        .access_cb = logControlAccess,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_WRITE,
        .min_key_size = 0,
        .val_handle = nullptr,
    },
    {
        .uuid = &kLogDataUuid.u,
        .access_cb = logDataAccess,
        .arg = nullptr,
        .descriptors = nullptr,
        .flags = BLE_GATT_CHR_F_READ,
        .min_key_size = 0,
        .val_handle = nullptr,
    },
    {},
};

const ble_gatt_svc_def kServices[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &kServiceUuid.u,
        .includes = nullptr,
        .characteristics = kCharacteristics,
    },
    {},
};

void advertise();

int gapEvent(ble_gap_event *event, void *arg) {
  (void)arg;
  switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
      if (event->connect.status == 0) {
        setConnected(true);
        ESP_LOGI(kTag, "connected");
      } else {
        ESP_LOGW(kTag, "connect failed; status=%d", event->connect.status);
        advertise();
      }
      return 0;

    case BLE_GAP_EVENT_DISCONNECT:
      setConnected(false);
      ESP_LOGI(kTag, "disconnected; reason=%d", event->disconnect.reason);
      advertise();
      return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
      advertise();
      return 0;

    default:
      return 0;
  }
}

void advertise() {
  ble_hs_adv_fields fields = {};
  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  fields.tx_pwr_lvl_is_present = 1;
  fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
  fields.name = reinterpret_cast<const uint8_t *>(cfg::kWatchId);
  fields.name_len = std::strlen(cfg::kWatchId);
  fields.name_is_complete = 1;
  fields.uuids128 = &kServiceUuid;
  fields.num_uuids128 = 1;
  fields.uuids128_is_complete = 1;

  int rc = ble_gap_adv_set_fields(&fields);
  if (rc != 0) {
    ESP_LOGE(kTag, "set adv fields failed: %d", rc);
    return;
  }

  ble_gap_adv_params params = {};
  params.conn_mode = BLE_GAP_CONN_MODE_UND;
  params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  params.itvl_min = kAdvIntervalUnits;
  params.itvl_max = kAdvIntervalUnits;

  rc = ble_gap_adv_start(gOwnAddrType, nullptr, BLE_HS_FOREVER, &params, gapEvent, nullptr);
  if (rc != 0) {
    ESP_LOGE(kTag, "start advertising failed: %d", rc);
  }
}

void onReset(int reason) {
  ESP_LOGE(kTag, "host reset: %d", reason);
  setConnected(false);
}

void onSync() {
  int rc = ble_hs_id_infer_auto(0, &gOwnAddrType);
  if (rc != 0) {
    ESP_LOGE(kTag, "infer address failed: %d", rc);
    return;
  }
  advertise();
}

void hostTask(void *param) {
  (void)param;
  nimble_port_run();
  nimble_port_freertos_deinit();
}

void initNvs() {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
}

}  // namespace

void bleInit() {
  initNvs();

  const esp_err_t memRelease = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  if (memRelease != ESP_OK && memRelease != ESP_ERR_INVALID_STATE) {
    ESP_ERROR_CHECK(memRelease);
  }

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
  ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());
#endif
  ESP_ERROR_CHECK(nimble_port_init());
  ESP_ERROR_CHECK(ble_att_set_preferred_mtu(128));

  ble_hs_cfg.reset_cb = onReset;
  ble_hs_cfg.sync_cb = onSync;

  ble_svc_gap_init();
  ble_svc_gatt_init();
  ESP_ERROR_CHECK(ble_svc_gap_device_name_set(cfg::kWatchId));

  int rc = ble_gatts_count_cfg(kServices);
  assert(rc == 0);
  rc = ble_gatts_add_svcs(kServices);
  assert(rc == 0);

  nimble_port_freertos_init(hostTask);
}

bool bleConnected() {
  portENTER_CRITICAL(&gBleMux);
  const bool connected = gConnected;
  portEXIT_CRITICAL(&gBleMux);
  return connected;
}

bool bleCommandActive() {
  portENTER_CRITICAL(&gBleMux);
  const bool active = static_cast<int32_t>(gCommandHoldUntilMs - uptimeMillis()) > 0;
  portEXIT_CRITICAL(&gBleMux);
  return active;
}

}  // namespace app
