#include "app.h"

#include <algorithm>
#include <cstring>
#include <ctime>
#include <sys/time.h>

#include <esp_err.h>
#include <esp_log.h>
#include <esp_partition.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace app {
namespace {

constexpr char kTag[] = "battery_log";
constexpr char kPartitionLabel[] = "batlog";
constexpr uint16_t kSampleIntervalSeconds = 60;
constexpr uint16_t kCapacity = 2720;
constexpr uint32_t kFortyHoursSeconds = 40u * 60u * 60u;
constexpr uint32_t kExportIdleTimeoutMs = 120000;
constexpr uint32_t kSaneUnixSeconds = 1704067200u;
constexpr uint32_t kFnv32Init = 2166136261u;
constexpr uint32_t kFnv32Prime = 16777619u;
constexpr uint32_t kControlMagic = 0x474C424Cu;  // LBLG
constexpr uint32_t kRecordMagic = 0x524C424Cu;   // LBLR
constexpr uint8_t kStorageSchema = 1;
constexpr uint8_t kControlFlagsNone = 0;
constexpr uint8_t kMetaFlagTimeValid = 1u << 0;
constexpr uint8_t kMetaFlagStorageReady = 1u << 1;
constexpr uint8_t kLatestFlagAvailable = 1u << 0;
constexpr uint8_t kRecordFlagTimeKnown = 1u << 0;
constexpr uint8_t kRecordFlagBatteryValid = 1u << 1;
constexpr uint8_t kPersistClean = 0;
constexpr uint8_t kPersistWriteOk = 1;
constexpr uint8_t kPersistWriteFailed = 2;
constexpr uint8_t kPersistRestoreOk = 3;
constexpr uint8_t kPersistRestoreFailed = 4;
constexpr uint8_t kPersistSchemaMismatch = 5;
constexpr uint8_t kPersistStorageUnavailable = 6;
constexpr uint8_t kExportErrorNotStarted = 1;
constexpr uint8_t kExportErrorPreparing = 2;
constexpr size_t kFlashSectorSize = 4096;
constexpr size_t kControlRecordSize = 32;
constexpr size_t kStoredRecordSize = 24;
constexpr uint16_t kControlRecordCount = kFlashSectorSize / kControlRecordSize;
constexpr uint16_t kSlotsPerSector = kFlashSectorSize / kStoredRecordSize;
constexpr uint16_t kDataSectors = kCapacity / kSlotsPerSector;
constexpr size_t kDataOffset = kFlashSectorSize;
constexpr size_t kStorageBytes = kDataOffset + kDataSectors * kFlashSectorSize;

static_assert(kCapacity * kSampleIntervalSeconds >= kFortyHoursSeconds,
              "battery log capacity must cover at least 40 hours");
static_assert(kCapacity == kDataSectors * kSlotsPerSector,
              "capacity must align to whole flash sectors");
static_assert(kLogDataRecordSize == 11, "wire record size changed unexpectedly");
static_assert(kLogDataPacketSize >= 2 + kLogDataRecordsPerPacket * kLogDataRecordSize,
              "data packet buffer is too small");

enum class ExportStage : uint8_t {
  Idle = 0,
  Preparing = 1,
  Metadata = 2,
  Data = 3,
  End = 4,
};

struct BatteryLogRecord {
  uint32_t sampleSeq;
  uint32_t sampleUnixSeconds;
  uint16_t batteryMillivolts;
  uint8_t flags;
};

struct LogEntry {
  BatteryLogRecord record;
  uint16_t storageSlot;
};

struct ControlState {
  uint32_t generation = 1;
  uint32_t nextSeq = 1;
  uint16_t nextSlot = 0;
  uint8_t flags = kControlFlagsNone;
};

struct BatteryLogRuntime {
  StaticSemaphore_t mutexStorage;
  SemaphoreHandle_t mutex = nullptr;
  const esp_partition_t *partition = nullptr;
  LogEntry entries[kCapacity] = {};
  uint16_t count = 0;
  uint32_t generation = 1;
  uint32_t nextSeq = 1;
  uint16_t nextSlot = 0;
  uint32_t nextSampleMs = 0;
  bool timeValid = false;
  bool storageReady = false;
  uint8_t lastPersistResult = kPersistClean;
  bool exportActive = false;
  ExportStage exportStage = ExportStage::Idle;
  uint16_t exportCount = 0;
  uint16_t exportCursor = 0;
  uint32_t exportHash = kFnv32Init;
  uint32_t exportDeadlineMs = 0;
};

BatteryLogRuntime g;

void lock() {
  if (g.mutex != nullptr) {
    xSemaphoreTake(g.mutex, portMAX_DELAY);
  }
}

void unlock() {
  if (g.mutex != nullptr) {
    xSemaphoreGive(g.mutex);
  }
}

uint16_t readLe16Local(const uint8_t *in) {
  return static_cast<uint16_t>(in[0]) | (static_cast<uint16_t>(in[1]) << 8);
}

uint32_t readLe32Local(const uint8_t *in) {
  return static_cast<uint32_t>(in[0]) | (static_cast<uint32_t>(in[1]) << 8) |
         (static_cast<uint32_t>(in[2]) << 16) | (static_cast<uint32_t>(in[3]) << 24);
}

uint32_t fnvByte(uint32_t value, uint8_t byte) {
  value ^= byte;
  return value * kFnv32Prime;
}

uint32_t fnvBytes(const uint8_t *data, size_t length) {
  uint32_t value = kFnv32Init;
  for (size_t i = 0; i < length; ++i) {
    value = fnvByte(value, data[i]);
  }
  return value;
}

uint32_t hashRecord(uint32_t hash, const BatteryLogRecord &record) {
  uint8_t packed[kLogDataRecordSize] = {};
  writeLe32(&packed[0], record.sampleSeq);
  writeLe32(&packed[4], record.sampleUnixSeconds);
  writeLe16(&packed[8], record.batteryMillivolts);
  packed[10] = record.flags;
  for (uint8_t byte : packed) {
    hash = fnvByte(hash, byte);
  }
  return hash;
}

uint32_t hashEntries(uint16_t count) {
  uint32_t hash = kFnv32Init;
  for (uint16_t i = 0; i < count; ++i) {
    hash = hashRecord(hash, g.entries[i].record);
  }
  return hash;
}

bool unixTimeValid(uint32_t unixSeconds) {
  return unixSeconds >= kSaneUnixSeconds;
}

uint32_t currentUnixSeconds(bool *valid) {
  const std::time_t now = std::time(nullptr);
  const bool ok = now >= 0 && unixTimeValid(static_cast<uint32_t>(now));
  if (valid != nullptr) {
    *valid = ok;
  }
  return ok ? static_cast<uint32_t>(now) : 0;
}

bool allErased(const uint8_t *data, size_t length) {
  for (size_t i = 0; i < length; ++i) {
    if (data[i] != 0xFF) {
      return false;
    }
  }
  return true;
}

size_t slotOffset(uint16_t slot) {
  const uint16_t sector = slot / kSlotsPerSector;
  const uint16_t sectorSlot = slot % kSlotsPerSector;
  return kDataOffset + static_cast<size_t>(sector) * kFlashSectorSize +
         static_cast<size_t>(sectorSlot) * kStoredRecordSize;
}

uint16_t slotSector(uint16_t slot) {
  return static_cast<uint16_t>(slot / kSlotsPerSector);
}

bool readStorage(size_t offset, void *out, size_t length) {
  return g.partition != nullptr && offset + length <= g.partition->size &&
         esp_partition_read(g.partition, offset, out, length) == ESP_OK;
}

bool writeStorage(size_t offset, const void *data, size_t length) {
  return g.partition != nullptr && offset + length <= g.partition->size &&
         esp_partition_write(g.partition, offset, data, length) == ESP_OK;
}

bool eraseStorage(size_t offset, size_t length) {
  return g.partition != nullptr && offset + length <= g.partition->size &&
         esp_partition_erase_range(g.partition, offset, length) == ESP_OK;
}

bool rangeErased(size_t offset, size_t length) {
  uint8_t buffer[64] = {};
  size_t consumed = 0;
  while (consumed < length) {
    const size_t chunk = std::min(sizeof(buffer), length - consumed);
    if (!readStorage(offset + consumed, buffer, chunk)) {
      return false;
    }
    if (!allErased(buffer, chunk)) {
      return false;
    }
    consumed += chunk;
  }
  return true;
}

void packControl(const ControlState &state, uint8_t *out) {
  std::memset(out, 0, kControlRecordSize);
  writeLe32(&out[0], kControlMagic);
  out[4] = kStorageSchema;
  out[5] = kStoredRecordSize;
  writeLe16(&out[6], kCapacity);
  writeLe16(&out[8], kSampleIntervalSeconds);
  out[10] = state.flags;
  out[11] = 0;
  writeLe32(&out[12], state.generation);
  writeLe32(&out[16], state.nextSeq);
  writeLe16(&out[20], state.nextSlot);
  writeLe32(&out[28], fnvBytes(out, 28));
}

bool unpackControl(const uint8_t *raw, ControlState *state, bool *schemaMismatch) {
  if (allErased(raw, kControlRecordSize)) {
    return false;
  }
  if (readLe32Local(&raw[0]) != kControlMagic) {
    if (schemaMismatch != nullptr) {
      *schemaMismatch = true;
    }
    return false;
  }
  const bool shapeOk = raw[4] == kStorageSchema && raw[5] == kStoredRecordSize &&
                       readLe16Local(&raw[6]) == kCapacity &&
                       readLe16Local(&raw[8]) == kSampleIntervalSeconds;
  const bool hashOk = readLe32Local(&raw[28]) == fnvBytes(raw, 28);
  if (!shapeOk || !hashOk) {
    if (schemaMismatch != nullptr) {
      *schemaMismatch = true;
    }
    return false;
  }
  state->flags = raw[10];
  state->generation = readLe32Local(&raw[12]);
  state->nextSeq = readLe32Local(&raw[16]);
  state->nextSlot = readLe16Local(&raw[20]);
  if (state->generation == 0 || state->generation == UINT32_MAX) {
    state->generation = 1;
  }
  if (state->nextSeq == 0 || state->nextSeq == UINT32_MAX) {
    state->nextSeq = 1;
  }
  if (state->nextSlot >= kCapacity) {
    state->nextSlot = 0;
  }
  return true;
}

bool findLatestControl(ControlState *state, bool *schemaMismatch) {
  uint8_t raw[kControlRecordSize] = {};
  bool found = false;
  for (uint16_t i = 0; i < kControlRecordCount; ++i) {
    if (!readStorage(static_cast<size_t>(i) * kControlRecordSize, raw, sizeof(raw))) {
      return false;
    }
    ControlState candidate = {};
    if (unpackControl(raw, &candidate, schemaMismatch)) {
      *state = candidate;
      found = true;
    }
  }
  return found;
}

bool appendControl(const ControlState &state) {
  uint8_t raw[kControlRecordSize] = {};
  size_t offset = kFlashSectorSize;
  for (uint16_t i = 0; i < kControlRecordCount; ++i) {
    const size_t candidateOffset = static_cast<size_t>(i) * kControlRecordSize;
    if (!readStorage(candidateOffset, raw, sizeof(raw))) {
      return false;
    }
    if (allErased(raw, sizeof(raw))) {
      offset = candidateOffset;
      break;
    }
  }
  if (offset == kFlashSectorSize) {
    if (!eraseStorage(0, kFlashSectorSize)) {
      return false;
    }
    offset = 0;
  }

  packControl(state, raw);
  if (!writeStorage(offset, raw, sizeof(raw))) {
    return false;
  }
  uint8_t verify[kControlRecordSize] = {};
  return readStorage(offset, verify, sizeof(verify)) &&
         std::memcmp(raw, verify, sizeof(raw)) == 0;
}

void packStoredRecord(const BatteryLogRecord &record, uint16_t slot, uint8_t *out) {
  (void)slot;
  std::memset(out, 0, kStoredRecordSize);
  writeLe32(&out[0], kRecordMagic);
  writeLe32(&out[4], g.generation);
  writeLe32(&out[8], record.sampleSeq);
  writeLe32(&out[12], record.sampleUnixSeconds);
  writeLe16(&out[16], record.batteryMillivolts);
  out[18] = record.flags;
  out[19] = 0;
  writeLe32(&out[20], fnvBytes(out, 20));
}

bool unpackStoredRecord(const uint8_t *raw, uint16_t slot, LogEntry *entry) {
  if (allErased(raw, kStoredRecordSize) || readLe32Local(&raw[0]) != kRecordMagic ||
      readLe32Local(&raw[4]) != g.generation ||
      readLe32Local(&raw[20]) != fnvBytes(raw, 20)) {
    return false;
  }

  BatteryLogRecord record = {};
  record.sampleSeq = readLe32Local(&raw[8]);
  record.sampleUnixSeconds = readLe32Local(&raw[12]);
  record.batteryMillivolts = readLe16Local(&raw[16]);
  record.flags = raw[18];
  if (record.sampleSeq == 0 || record.sampleSeq == UINT32_MAX) {
    return false;
  }

  entry->record = record;
  entry->storageSlot = slot;
  return true;
}

void resetVolatileLogState() {
  g.count = 0;
  g.nextSeq = 1;
  g.nextSlot = 0;
  g.exportActive = false;
  g.exportStage = ExportStage::Idle;
  g.exportCount = 0;
  g.exportCursor = 0;
  g.exportHash = kFnv32Init;
  g.exportDeadlineMs = 0;
}

void removeEntriesInSector(uint16_t sector) {
  uint16_t writeIndex = 0;
  for (uint16_t i = 0; i < g.count; ++i) {
    if (slotSector(g.entries[i].storageSlot) != sector) {
      if (writeIndex != i) {
        g.entries[writeIndex] = g.entries[i];
      }
      ++writeIndex;
    }
  }
  g.count = writeIndex;
}

void addEntry(const BatteryLogRecord &record, uint16_t slot) {
  if (g.count >= kCapacity) {
    std::memmove(&g.entries[0], &g.entries[1], sizeof(LogEntry) * (kCapacity - 1));
    g.count = kCapacity - 1;
  }
  g.entries[g.count++] = {record, slot};
}

bool persistFreshControl(uint8_t resultCode) {
  ControlState state = {};
  state.generation = g.generation;
  state.nextSeq = g.nextSeq;
  state.nextSlot = g.nextSlot;
  const bool ok = appendControl(state);
  g.lastPersistResult = ok ? resultCode : kPersistWriteFailed;
  return ok;
}

bool initializeFreshStorage(uint8_t resultCode) {
  if (!eraseStorage(0, kStorageBytes)) {
    g.lastPersistResult = kPersistRestoreFailed;
    return false;
  }
  resetVolatileLogState();
  g.generation = 1;
  g.storageReady = persistFreshControl(resultCode);
  return g.storageReady;
}

bool restoreRecords(const ControlState &control) {
  g.generation = control.generation;
  resetVolatileLogState();
  g.nextSeq = control.nextSeq;
  g.nextSlot = control.nextSlot;

  uint8_t raw[kStoredRecordSize] = {};
  for (uint16_t slot = 0; slot < kCapacity; ++slot) {
    if (!readStorage(slotOffset(slot), raw, sizeof(raw))) {
      return false;
    }
    LogEntry entry = {};
    if (unpackStoredRecord(raw, slot, &entry)) {
      addEntry(entry.record, entry.storageSlot);
    }
  }

  std::sort(g.entries, g.entries + g.count, [](const LogEntry &left, const LogEntry &right) {
    return left.record.sampleSeq < right.record.sampleSeq;
  });
  if (g.count > 0) {
    const LogEntry &latest = g.entries[g.count - 1];
    g.nextSeq = latest.record.sampleSeq == UINT32_MAX ? 1 : latest.record.sampleSeq + 1;
    g.nextSlot = static_cast<uint16_t>((latest.storageSlot + 1) % kCapacity);
  }
  return true;
}

void restoreStorage() {
  g.partition =
      esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, kPartitionLabel);
  if (g.partition == nullptr || g.partition->size < kStorageBytes) {
    resetVolatileLogState();
    g.storageReady = false;
    g.lastPersistResult = kPersistStorageUnavailable;
    ESP_LOGE(kTag, "battery log partition unavailable");
    return;
  }

  bool schemaMismatch = false;
  ControlState control = {};
  const bool foundControl = findLatestControl(&control, &schemaMismatch);
  if (!foundControl) {
    const bool erased = rangeErased(0, kStorageBytes);
    if (schemaMismatch || !erased) {
      ESP_LOGW(kTag, "battery log storage schema mismatch; clearing partition");
      initializeFreshStorage(kPersistSchemaMismatch);
    } else {
      resetVolatileLogState();
      g.generation = 1;
      g.storageReady = persistFreshControl(kPersistClean);
    }
    return;
  }

  if (!restoreRecords(control)) {
    resetVolatileLogState();
    g.storageReady = false;
    g.lastPersistResult = kPersistRestoreFailed;
    ESP_LOGE(kTag, "battery log restore failed");
    return;
  }

  g.storageReady = true;
  g.lastPersistResult = kPersistRestoreOk;
  ESP_LOGI(kTag, "restored %u battery log records", g.count);
}

bool persistRecord(const BatteryLogRecord &record, uint16_t slot) {
  if (!g.storageReady) {
    g.lastPersistResult = kPersistStorageUnavailable;
    return false;
  }

  if ((slot % kSlotsPerSector) == 0) {
    const uint16_t sector = slotSector(slot);
    if (!eraseStorage(kDataOffset + static_cast<size_t>(sector) * kFlashSectorSize,
                      kFlashSectorSize)) {
      g.lastPersistResult = kPersistWriteFailed;
      return false;
    }
    removeEntriesInSector(sector);
  }

  uint8_t raw[kStoredRecordSize] = {};
  packStoredRecord(record, slot, raw);
  const size_t offset = slotOffset(slot);
  if (!writeStorage(offset, raw, sizeof(raw))) {
    g.lastPersistResult = kPersistWriteFailed;
    return false;
  }

  uint8_t verify[kStoredRecordSize] = {};
  if (!readStorage(offset, verify, sizeof(verify)) ||
      std::memcmp(raw, verify, sizeof(raw)) != 0) {
    g.lastPersistResult = kPersistWriteFailed;
    return false;
  }
  g.lastPersistResult = kPersistWriteOk;
  return true;
}

void appendRecord(uint16_t batteryMillivolts, uint32_t sampleUnixSeconds, bool timeKnown) {
  BatteryLogRecord record = {};
  record.sampleSeq = g.nextSeq;
  record.sampleUnixSeconds = timeKnown ? sampleUnixSeconds : 0;
  record.batteryMillivolts = batteryMillivolts;
  if (timeKnown) {
    record.flags |= kRecordFlagTimeKnown;
  }
  if (batteryMillivolts > 0) {
    record.flags |= kRecordFlagBatteryValid;
  }

  const uint16_t slot = g.nextSlot;
  if (!persistRecord(record, slot)) {
    return;
  }

  addEntry(record, slot);
  g.nextSeq = g.nextSeq == UINT32_MAX ? 1 : g.nextSeq + 1;
  g.nextSlot = static_cast<uint16_t>((g.nextSlot + 1) % kCapacity);
}

bool serviceDue(uint32_t now) {
  return static_cast<int32_t>(now - g.nextSampleMs) >= 0;
}

bool exportTimedOut(uint32_t now) {
  return g.exportActive && static_cast<int32_t>(now - g.exportDeadlineMs) >= 0;
}

void cancelExport() {
  g.exportActive = false;
  g.exportStage = ExportStage::Idle;
  g.exportCount = 0;
  g.exportCursor = 0;
  g.exportHash = kFnv32Init;
  g.exportDeadlineMs = 0;
}

void extendExportDeadline(uint32_t now) {
  g.exportDeadlineMs = now + kExportIdleTimeoutMs;
}

void writeError(uint8_t *out, uint8_t code) {
  out[0] = kPacketLogError;
  out[1] = code;
}

size_t writeMetadata(uint8_t *out) {
  out[0] = kPacketLogMetadata;
  out[1] = kLogSchema;
  writeLe16(&out[2], g.exportCount);
  writeLe16(&out[4], kCapacity);
  writeLe16(&out[6], kSampleIntervalSeconds);
  writeLe32(&out[8], g.exportHash);
  writeLe32(&out[12], g.nextSeq);
  out[16] = g.lastPersistResult;
  out[17] = (g.timeValid ? kMetaFlagTimeValid : 0) |
            (g.storageReady ? kMetaFlagStorageReady : 0);
  return kLogMetadataPacketSize;
}

void writeRecordPayload(uint8_t *out, const BatteryLogRecord &record) {
  writeLe32(&out[0], record.sampleSeq);
  writeLe32(&out[4], record.sampleUnixSeconds);
  writeLe16(&out[8], record.batteryMillivolts);
  out[10] = record.flags;
}

size_t writeData(uint8_t *out) {
  const uint16_t remaining = static_cast<uint16_t>(g.exportCount - g.exportCursor);
  const uint8_t recordCount =
      static_cast<uint8_t>(std::min<uint16_t>(remaining, kLogDataRecordsPerPacket));
  out[0] = kPacketLogData;
  out[1] = recordCount;
  for (uint8_t i = 0; i < recordCount; ++i) {
    writeRecordPayload(&out[2 + i * kLogDataRecordSize],
                       g.entries[g.exportCursor + i].record);
  }
  g.exportCursor = static_cast<uint16_t>(g.exportCursor + recordCount);
  if (g.exportCursor >= g.exportCount) {
    g.exportStage = ExportStage::End;
  }
  return 2 + static_cast<size_t>(recordCount) * kLogDataRecordSize;
}

size_t writeEnd(uint8_t *out) {
  out[0] = kPacketLogEnd;
  out[1] = kLogSchema;
  writeLe32(&out[2], g.exportHash);
  writeLe16(&out[6], g.exportCount);
  cancelExport();
  return kLogEndPacketSize;
}

size_t writeLatest(uint8_t *out) {
  out[0] = kPacketBatteryLatest;
  out[1] = kBatteryLatestSchema;
  out[2] = 0;
  out[3] = 0;
  if (g.count == 0) {
    writeLe32(&out[4], 0);
    writeLe32(&out[8], 0);
    writeLe16(&out[12], 0);
    return kBatteryLatestPacketSize;
  }

  const BatteryLogRecord &record = g.entries[g.count - 1].record;
  out[2] = kLatestFlagAvailable;
  out[3] = record.flags;
  writeLe32(&out[4], record.sampleSeq);
  writeLe32(&out[8], record.sampleUnixSeconds);
  writeLe16(&out[12], record.batteryMillivolts);
  return kBatteryLatestPacketSize;
}

void prepareExportIfNeeded() {
  if (g.exportActive && g.exportStage == ExportStage::Preparing) {
    g.exportHash = hashEntries(g.exportCount);
    g.exportStage = ExportStage::Metadata;
  }
}

void loadRtcIntoSystemClock() {
  uint32_t rtcUnixSeconds = 0;
  if (!boardRtcUnixSeconds(&rtcUnixSeconds) || !unixTimeValid(rtcUnixSeconds)) {
    return;
  }

  timeval tv = {};
  tv.tv_sec = static_cast<time_t>(rtcUnixSeconds);
  if (settimeofday(&tv, nullptr) == 0) {
    ESP_LOGI(kTag, "loaded PCF8563 RTC time");
  }
}

}  // namespace

void batteryLogInit() {
  if (g.mutex == nullptr) {
    g.mutex = xSemaphoreCreateMutexStatic(&g.mutexStorage);
  }
  lock();
  restoreStorage();
  loadRtcIntoSystemClock();
  g.nextSampleMs = uptimeMillis() + kSampleIntervalSeconds * 1000u;
  g.timeValid = unixTimeValid(currentUnixSeconds(nullptr));
  unlock();
}

void batteryLogService() {
  const uint32_t now = uptimeMillis();
  lock();
  if (exportTimedOut(now)) {
    cancelExport();
  }
  if (g.exportActive) {
    prepareExportIfNeeded();
    unlock();
    return;
  }
  if (!serviceDue(now)) {
    unlock();
    return;
  }
  g.nextSampleMs = now + kSampleIntervalSeconds * 1000u;
  unlock();

  const uint16_t millivolts = boardBatteryMillivolts();
  if (millivolts == 0) {
    return;
  }
  bool timeKnown = false;
  const uint32_t unixSeconds = currentUnixSeconds(&timeKnown);

  lock();
  g.timeValid = g.timeValid || timeKnown;
  appendRecord(millivolts, unixSeconds, timeKnown);
  unlock();
}

void batteryLogClear() {
  lock();
  const uint32_t nextGeneration = g.generation == UINT32_MAX - 1 ? 1 : g.generation + 1;
  resetVolatileLogState();
  g.generation = nextGeneration;
  g.nextSampleMs = uptimeMillis() + kSampleIntervalSeconds * 1000u;
  g.storageReady = persistFreshControl(kPersistWriteOk);
  unlock();
}

bool batteryLogSetTime(uint32_t unixSeconds) {
  if (!unixTimeValid(unixSeconds)) {
    return false;
  }
  timeval tv = {};
  tv.tv_sec = static_cast<time_t>(unixSeconds);
  if (settimeofday(&tv, nullptr) != 0) {
    return false;
  }
  if (!boardRtcSetUnixSeconds(unixSeconds)) {
    return false;
  }
  lock();
  g.timeValid = true;
  unlock();
  return true;
}

void batteryLogBeginExport() {
  lock();
  g.exportCount = g.count;
  g.exportCursor = 0;
  g.exportHash = kFnv32Init;
  g.exportActive = true;
  g.exportStage = ExportStage::Preparing;
  extendExportDeadline(uptimeMillis());
  unlock();
}

size_t batteryLogBuildNextPacket(uint8_t *out, size_t capacity) {
  if (out == nullptr || capacity < kLogErrorPacketSize) {
    return 0;
  }

  lock();
  const uint32_t now = uptimeMillis();
  if (exportTimedOut(now)) {
    cancelExport();
  }
  if (!g.exportActive) {
    writeError(out, kExportErrorNotStarted);
    unlock();
    return kLogErrorPacketSize;
  }

  size_t written = 0;
  switch (g.exportStage) {
    case ExportStage::Preparing:
      writeError(out, kExportErrorPreparing);
      written = kLogErrorPacketSize;
      break;

    case ExportStage::Metadata:
      if (capacity >= kLogMetadataPacketSize) {
        written = writeMetadata(out);
        g.exportStage = g.exportCount > 0 ? ExportStage::Data : ExportStage::End;
      }
      break;

    case ExportStage::Data:
      if (capacity >= kLogDataPacketSize) {
        written = writeData(out);
      }
      break;

    case ExportStage::End:
      if (capacity >= kLogEndPacketSize) {
        written = writeEnd(out);
      }
      break;

    case ExportStage::Idle:
      writeError(out, kExportErrorNotStarted);
      written = kLogErrorPacketSize;
      break;
  }

  if (g.exportActive && written > 0) {
    extendExportDeadline(now);
  }
  unlock();
  return written;
}

size_t batteryLogBuildLatestPacket(uint8_t *out, size_t capacity) {
  if (out == nullptr || capacity < kBatteryLatestPacketSize) {
    return 0;
  }

  lock();
  const size_t written = writeLatest(out);
  unlock();
  return written;
}

bool batteryLogExportActive() {
  lock();
  if (exportTimedOut(uptimeMillis())) {
    cancelExport();
  }
  const bool active = g.exportActive;
  unlock();
  return active;
}

}  // namespace app
