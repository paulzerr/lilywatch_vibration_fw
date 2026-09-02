#pragma once

#include <cstddef>
#include <cstdint>

namespace app {

constexpr uint8_t kPacketStatus = 0x10;
constexpr uint8_t kPacketPowerDiagnostics = 0x11;
constexpr uint8_t kPacketBatteryLatest = 0x12;
constexpr uint8_t kPacketLogEnd = 0x01;
constexpr uint8_t kPacketLogMetadata = 0x02;
constexpr uint8_t kPacketLogData = 0x03;
constexpr uint8_t kPacketLogError = 0xEE;
constexpr uint8_t kCommandVibrate = 0x01;
constexpr uint8_t kCommandHoldAwake = 0x02;
constexpr uint8_t kLogSchema = 6;
constexpr uint8_t kStatusSchema = 6;
constexpr uint8_t kPowerDiagnosticsSchema = 1;
constexpr uint8_t kBatteryLatestSchema = 1;
constexpr size_t kLogMetadataPacketSize = 18;
constexpr size_t kLogDataRecordSize = 11;
constexpr size_t kLogDataRecordsPerPacket = 8;
constexpr size_t kLogDataPacketSize = 2 + kLogDataRecordsPerPacket * kLogDataRecordSize;
constexpr size_t kLogEndPacketSize = 8;
constexpr size_t kLogErrorPacketSize = 2;
constexpr size_t kStatusPacketSize = 24;
constexpr size_t kPowerDiagnosticsPacketSize = 32;
constexpr size_t kBatteryLatestPacketSize = 14;

enum class PowerState : uint8_t {
  Awake = 1,
  BatteryIdleReachable = 2,
  PowerSavingFailed = 3,
};

enum ReasonFlag : uint32_t {
  kReasonUsbPresent = 1u << 0,
  kReasonUsbReadFailed = 1u << 1,
  kReasonBleConnected = 1u << 2,
  kReasonBootHold = 1u << 3,
  kReasonPowerFailure = 1u << 4,
  kReasonBoardInitFailed = 1u << 5,
  kReasonCommandActive = 1u << 6,
  kReasonExportActive = 1u << 7,
};

enum DiagFlag : uint32_t {
  kDiagPowerSavingOk = 1u << 0,
  kDiagPmBuildEnabled = 1u << 1,
  kDiagTicklessEnabled = 1u << 2,
  kDiagBleSleepConfigured = 1u << 3,
  kDiagPmConfigured = 1u << 4,
  kDiagLockCreated = 1u << 5,
  kDiagLockHeld = 1u << 6,
  kDiagIdleLockReleasedOnce = 1u << 7,
  kDiagUsbReadOk = 1u << 8,
  kDiagBoardReady = 1u << 9,
  kDiagPowerDiagnosticsAvailable = 1u << 10,
  kDiagPowerDiagnosticsProfiling = 1u << 11,
};

enum PowerDiagnosticFlag : uint16_t {
  kPowerDiagBuildEnabled = 1u << 0,
  kPowerDiagPmProfilingEnabled = 1u << 1,
  kPowerDiagStatsValid = 1u << 2,
  kPowerDiagLightSleepSeen = 1u << 3,
  kPowerDiagDumpTruncated = 1u << 4,
  kPowerDiagParseFailed = 1u << 5,
  kPowerDiagSleepStatsSeen = 1u << 6,
  kPowerDiagApbMinStatsSeen = 1u << 7,
  kPowerDiagApbMaxStatsSeen = 1u << 8,
  kPowerDiagCpuMaxStatsSeen = 1u << 9,
};

enum FailureReason : uint8_t {
  kFailNone = 0,
  kFailPmBuildDisabled = 1,
  kFailTicklessDisabled = 2,
  kFailPmConfig = 3,
  kFailLockCreate = 4,
  kFailBleSleepDisabled = 5,
  kFailLockApply = 6,
  kFailUsbRead = 7,
  kFailIdleNotObserved = 8,
  kFailBoardInit = 9,
};

enum BleLpClockSource : uint8_t {
  kBleLpClockUnknown = 0,
  kBleLpClockMainXtal = 1,
  kBleLpClockExternal32k = 2,
  kBleLpClockInternalRc = 3,
};

struct PowerInputs {
  bool usbPresent;
  bool usbReadOk;
  bool bleConnected;
  bool bootHold;
  bool boardReady;
  bool commandActive;
  bool exportActive;
};

struct PowerSnapshot {
  PowerState state;
  FailureReason failure;
  uint32_t reasons;
  uint32_t diag;
  int16_t pmConfigResult;
  int16_t lockCreateResult;
  int16_t lockApplyResult;
  uint8_t resetReason;
  uint8_t wakeCause;
  uint8_t bleLpClockSource;
  uint16_t batteryMillivolts;
};

struct PowerDiagnosticsSnapshot {
  uint16_t flags;
  int16_t statsResult;
  uint32_t uptimeSeconds;
  uint32_t totalProfiledMs;
  uint32_t lightSleepMs;
  uint32_t apbMinMs;
  uint32_t cpuMaxMs;
  uint16_t lightSleepPercentX100;
  uint16_t noLightSleepLockCount;
  uint16_t noLightSleepReleaseCount;
};

bool boardInit();
uint16_t boardBatteryMillivolts();
bool boardUsbPresent(bool *readOk);
bool boardHapticPower(bool enabled);
bool boardHapticWrite(uint8_t reg, uint8_t value);
bool boardRtcUnixSeconds(uint32_t *unixSeconds);
bool boardRtcSetUnixSeconds(uint32_t unixSeconds);

void powerInit();
void powerUpdate(const PowerInputs &inputs);
PowerSnapshot powerSnapshot();
PowerDiagnosticsSnapshot powerDiagnosticsSnapshot();

void bleInit();
bool bleConnected();
bool bleCommandActive();

void hapticInit();
bool hapticStartVibration(uint16_t durationMs, uint8_t repeats, uint16_t offTimeMs,
                          uint8_t holdAwakeSeconds);
bool hapticCommandActive();

void batteryLogInit();
void batteryLogService();
void batteryLogClear();
bool batteryLogSetTime(uint32_t unixSeconds);
void batteryLogBeginExport();
size_t batteryLogBuildNextPacket(uint8_t *out, size_t capacity);
size_t batteryLogBuildLatestPacket(uint8_t *out, size_t capacity);
bool batteryLogExportActive();

void writeLe16(uint8_t *out, uint16_t value);
void writeLe32(uint8_t *out, uint32_t value);
uint32_t uptimeMillis();
int16_t err16(int err);

}  // namespace app
