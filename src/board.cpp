#include "app.h"

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <ctime>

namespace app {
namespace {

constexpr char kTag[] = "board";

constexpr i2c_port_t kI2cPort = I2C_NUM_0;
constexpr gpio_num_t kI2cSda = GPIO_NUM_10;
constexpr gpio_num_t kI2cScl = GPIO_NUM_11;
constexpr gpio_num_t kTftBacklight = GPIO_NUM_45;

constexpr uint8_t kAxp = 0x34;
constexpr uint8_t kAxpRegStatus1 = 0x00;
constexpr uint8_t kAxpRegType = 0x03;
constexpr uint8_t kAxpRegChargeCtrl = 0x18;
constexpr uint8_t kAxpRegVoff = 0x24;
constexpr uint8_t kAxpRegAdcCtrl = 0x30;
constexpr uint8_t kAxpRegBatVoltHi = 0x34;
constexpr uint8_t kAxpRegBatVoltLo = 0x35;
constexpr uint8_t kAxpRegIntEn1 = 0x40;
constexpr uint8_t kAxpRegIntEn2 = 0x41;
constexpr uint8_t kAxpRegIntEn3 = 0x42;
constexpr uint8_t kAxpRegIntSts1 = 0x48;
constexpr uint8_t kAxpRegIntSts2 = 0x49;
constexpr uint8_t kAxpRegIntSts3 = 0x4A;
constexpr uint8_t kAxpRegPrechargeCurrent = 0x61;
constexpr uint8_t kAxpRegConstantChargeCurrent = 0x62;
constexpr uint8_t kAxpRegTerminationCurrent = 0x63;
constexpr uint8_t kAxpRegChargeVoltage = 0x64;
constexpr uint8_t kAxpRegBatDetect = 0x68;
constexpr uint8_t kAxpRegLdoOnOff0 = 0x90;
constexpr uint8_t kAxpRegBldo2Voltage = 0x97;

constexpr uint8_t kAxpType2101 = 0x4A;
constexpr uint8_t kAxpVbusGood = 1u << 5;
constexpr uint8_t kAxpBatteryPresent = 1u << 3;
constexpr uint8_t kAxpChargeEnable = 1u << 1;
constexpr uint8_t kAxpVoffMask = 0x07;
constexpr uint8_t kAxpVoff3v3 = 0x07;
constexpr uint8_t kAxpPrechargeCurrentMask = 0x0F;
constexpr uint8_t kAxpPrecharge125mA = 0x05;
constexpr uint8_t kAxpConstantChargeCurrentMask = 0x1F;
constexpr uint8_t kAxpConstantCharge300mA = 0x09;
constexpr uint8_t kAxpTerminationCurrentMask = 0x1F;
constexpr uint8_t kAxpTerminationEnable = 1u << 4;
constexpr uint8_t kAxpTermination125mA = 0x05;
constexpr uint8_t kAxpChargeVoltageMask = 0x07;
constexpr uint8_t kAxpChargeVoltage4v35 = 0x04;
constexpr uint8_t kAxpBldo2Enable = 1u << 5;
constexpr uint8_t kAxpBldo2_3v3 = 28;
constexpr uint8_t kDrv2605 = 0x5A;
constexpr uint8_t kPcf8563 = 0x51;
constexpr uint8_t kPcfRegSeconds = 0x02;
constexpr uint8_t kPcfRegMinutes = 0x03;
constexpr uint8_t kPcfRegHours = 0x04;
constexpr uint8_t kPcfRegDays = 0x05;
constexpr uint8_t kPcfRegWeekdays = 0x06;
constexpr uint8_t kPcfRegMonths = 0x07;
constexpr uint8_t kPcfRegYears = 0x08;
constexpr uint8_t kPcfVoltageLow = 1u << 7;

bool gI2cReady = false;
StaticSemaphore_t gI2cMutexBuffer;
SemaphoreHandle_t gI2cMutex = nullptr;

bool takeI2c() {
  return gI2cMutex == nullptr || xSemaphoreTake(gI2cMutex, pdMS_TO_TICKS(100)) == pdTRUE;
}

void giveI2c() {
  if (gI2cMutex != nullptr) {
    xSemaphoreGive(gI2cMutex);
  }
}

esp_err_t i2cInit() {
  if (gI2cReady) {
    return ESP_OK;
  }

  i2c_config_t config = {};
  config.mode = I2C_MODE_MASTER;
  config.sda_io_num = kI2cSda;
  config.scl_io_num = kI2cScl;
  config.sda_pullup_en = GPIO_PULLUP_ENABLE;
  config.scl_pullup_en = GPIO_PULLUP_ENABLE;
  config.master.clk_speed = 400000;

  esp_err_t err = i2c_param_config(kI2cPort, &config);
  if (err != ESP_OK) {
    return err;
  }
  err = i2c_driver_install(kI2cPort, config.mode, 0, 0, 0);
  if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
    if (gI2cMutex == nullptr) {
      gI2cMutex = xSemaphoreCreateMutexStatic(&gI2cMutexBuffer);
    }
    gI2cReady = true;
    return ESP_OK;
  }
  return err;
}

esp_err_t writeReg(uint8_t address, uint8_t reg, uint8_t value) {
  esp_err_t err = i2cInit();
  if (err != ESP_OK) {
    return err;
  }

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, static_cast<uint8_t>((address << 1) | I2C_MASTER_WRITE), true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write_byte(cmd, value, true);
  i2c_master_stop(cmd);
  if (takeI2c()) {
    err = i2c_master_cmd_begin(kI2cPort, cmd, pdMS_TO_TICKS(50));
    giveI2c();
  } else {
    err = ESP_ERR_TIMEOUT;
  }
  i2c_cmd_link_delete(cmd);
  return err;
}

int readReg(uint8_t address, uint8_t reg) {
  esp_err_t err = i2cInit();
  if (err != ESP_OK) {
    return -1;
  }

  uint8_t value = 0;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, static_cast<uint8_t>((address << 1) | I2C_MASTER_WRITE), true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, static_cast<uint8_t>((address << 1) | I2C_MASTER_READ), true);
  i2c_master_read_byte(cmd, &value, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  if (takeI2c()) {
    err = i2c_master_cmd_begin(kI2cPort, cmd, pdMS_TO_TICKS(50));
    giveI2c();
  } else {
    err = ESP_ERR_TIMEOUT;
  }
  i2c_cmd_link_delete(cmd);
  return err == ESP_OK ? value : -1;
}

bool axpWrite(uint8_t reg, uint8_t value) {
  return writeReg(kAxp, reg, value) == ESP_OK;
}

int axpRead(uint8_t reg) {
  return readReg(kAxp, reg);
}

bool axpUpdate(uint8_t reg, uint8_t mask, uint8_t bits) {
  const int current = axpRead(reg);
  if (current < 0) {
    return false;
  }
  const uint8_t next = (static_cast<uint8_t>(current) & ~mask) | (bits & mask);
  return axpWrite(reg, next);
}

bool axpFieldEquals(uint8_t reg, uint8_t mask, uint8_t expected) {
  const int value = axpRead(reg);
  return value >= 0 && (static_cast<uint8_t>(value) & mask) == (expected & mask);
}

void clearPmuIrq() {
  axpWrite(kAxpRegIntSts1, 0xFF);
  axpWrite(kAxpRegIntSts2, 0xFF);
  axpWrite(kAxpRegIntSts3, 0xFF);
}

uint8_t bcdToBinary(uint8_t value) {
  return static_cast<uint8_t>((value & 0x0F) + ((value >> 4) & 0x0F) * 10);
}

uint8_t binaryToBcd(uint8_t value) {
  return static_cast<uint8_t>(((value / 10) << 4) | (value % 10));
}

bool leapYear(int year) {
  return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

uint8_t daysInMonth(int year, uint8_t month) {
  static constexpr uint8_t kDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (month == 2 && leapYear(year)) {
    return 29;
  }
  return month >= 1 && month <= 12 ? kDays[month - 1] : 0;
}

bool validDateTime(int year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute,
                   uint8_t second) {
  return year >= 2024 && year <= 2099 && month >= 1 && month <= 12 && day >= 1 &&
         day <= daysInMonth(year, month) && hour <= 23 && minute <= 59 && second <= 59;
}

int64_t daysFromCivil(int year, unsigned month, unsigned day) {
  year -= month <= 2;
  const int era = (year >= 0 ? year : year - 399) / 400;
  const unsigned yearOfEra = static_cast<unsigned>(year - era * 400);
  const unsigned dayOfYear =
      (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
  const unsigned dayOfEra =
      yearOfEra * 365 + yearOfEra / 4 - yearOfEra / 100 + dayOfYear;
  return static_cast<int64_t>(era) * 146097 + static_cast<int64_t>(dayOfEra) - 719468;
}

uint8_t weekdayFromUnix(uint32_t unixSeconds) {
  const uint32_t days = unixSeconds / 86400u;
  return static_cast<uint8_t>((days + 4u) % 7u);
}

}  // namespace

bool boardInit() {
  gpio_reset_pin(kTftBacklight);
  gpio_set_direction(kTftBacklight, GPIO_MODE_OUTPUT);
  gpio_set_level(kTftBacklight, 0);

  const esp_err_t i2c = i2cInit();
  if (i2c != ESP_OK) {
    ESP_LOGE(kTag, "i2c init failed: %s", esp_err_to_name(i2c));
    return false;
  }

  const int chip = axpRead(kAxpRegType);
  if (chip != kAxpType2101) {
    ESP_LOGE(kTag, "unexpected AXP chip id: 0x%02x", chip < 0 ? 0 : chip);
    return false;
  }

  // Program the complete charge profile explicitly instead of inheriting PMU defaults.
  // Keep charging disabled until every limit is in place; a failed setup then fails safe.
  const bool configured =
      axpWrite(kAxpRegIntEn1, 0) && axpWrite(kAxpRegIntEn2, 0) &&
      axpWrite(kAxpRegIntEn3, 0) &&
      axpUpdate(kAxpRegChargeCtrl, kAxpChargeEnable, 0) &&
      axpUpdate(kAxpRegBatDetect, 0x01, 0x01) &&
      axpUpdate(kAxpRegAdcCtrl, 0x01, 0x01) &&
      axpUpdate(kAxpRegVoff, kAxpVoffMask, kAxpVoff3v3) &&
      axpUpdate(kAxpRegPrechargeCurrent, kAxpPrechargeCurrentMask, kAxpPrecharge125mA) &&
      axpUpdate(kAxpRegConstantChargeCurrent, kAxpConstantChargeCurrentMask,
                kAxpConstantCharge300mA) &&
      axpUpdate(kAxpRegTerminationCurrent, kAxpTerminationCurrentMask,
                kAxpTerminationEnable | kAxpTermination125mA) &&
      axpUpdate(kAxpRegChargeVoltage, kAxpChargeVoltageMask, kAxpChargeVoltage4v35) &&
      axpUpdate(kAxpRegChargeCtrl, kAxpChargeEnable, kAxpChargeEnable) &&
      axpUpdate(kAxpRegLdoOnOff0, kAxpBldo2Enable, 0);
  const bool verified =
      configured && axpFieldEquals(kAxpRegChargeCtrl, kAxpChargeEnable, kAxpChargeEnable) &&
      axpFieldEquals(kAxpRegVoff, kAxpVoffMask, kAxpVoff3v3) &&
      axpFieldEquals(kAxpRegPrechargeCurrent, kAxpPrechargeCurrentMask,
                     kAxpPrecharge125mA) &&
      axpFieldEquals(kAxpRegConstantChargeCurrent, kAxpConstantChargeCurrentMask,
                     kAxpConstantCharge300mA) &&
      axpFieldEquals(kAxpRegTerminationCurrent, kAxpTerminationCurrentMask,
                     kAxpTerminationEnable | kAxpTermination125mA) &&
      axpFieldEquals(kAxpRegChargeVoltage, kAxpChargeVoltageMask, kAxpChargeVoltage4v35);
  clearPmuIrq();
  if (!verified) {
    ESP_LOGE(kTag, "AXP2101 setup failed");
    return false;
  }

  ESP_LOGI(kTag,
           "AXP2101 ready: VOFF=3.3 V, charge=300 mA, precharge=125 mA, "
           "termination=125 mA, target=4.35 V");
  return true;
}

uint16_t boardBatteryMillivolts() {
  const int status = axpRead(kAxpRegStatus1);
  if (status < 0 || (status & kAxpBatteryPresent) == 0) {
    return 0;
  }
  const int hi = axpRead(kAxpRegBatVoltHi);
  const int lo = axpRead(kAxpRegBatVoltLo);
  if (hi < 0 || lo < 0) {
    return 0;
  }
  return static_cast<uint16_t>(((static_cast<uint16_t>(hi) & 0x1F) << 8) |
                               static_cast<uint16_t>(lo));
}

bool boardUsbPresent(bool *readOk) {
  const int status = axpRead(kAxpRegStatus1);
  const bool ok = status >= 0;
  if (readOk != nullptr) {
    *readOk = ok;
  }
  return ok ? ((status & kAxpVbusGood) != 0) : true;
}

bool boardHapticPower(bool enabled) {
  if (enabled) {
    return axpWrite(kAxpRegBldo2Voltage, kAxpBldo2_3v3) &&
           axpUpdate(kAxpRegLdoOnOff0, kAxpBldo2Enable, kAxpBldo2Enable);
  }
  return axpUpdate(kAxpRegLdoOnOff0, kAxpBldo2Enable, 0);
}

bool boardHapticWrite(uint8_t reg, uint8_t value) {
  return writeReg(kDrv2605, reg, value) == ESP_OK;
}

bool boardRtcUnixSeconds(uint32_t *unixSeconds) {
  if (unixSeconds == nullptr) {
    return false;
  }

  const int rawSecond = readReg(kPcf8563, kPcfRegSeconds);
  const int rawMinute = readReg(kPcf8563, kPcfRegMinutes);
  const int rawHour = readReg(kPcf8563, kPcfRegHours);
  const int rawDay = readReg(kPcf8563, kPcfRegDays);
  const int rawMonth = readReg(kPcf8563, kPcfRegMonths);
  const int rawYear = readReg(kPcf8563, kPcfRegYears);
  if (rawSecond < 0 || rawMinute < 0 || rawHour < 0 || rawDay < 0 || rawMonth < 0 ||
      rawYear < 0 || (rawSecond & kPcfVoltageLow) != 0) {
    return false;
  }

  const uint8_t second = bcdToBinary(static_cast<uint8_t>(rawSecond) & 0x7F);
  const uint8_t minute = bcdToBinary(static_cast<uint8_t>(rawMinute) & 0x7F);
  const uint8_t hour = bcdToBinary(static_cast<uint8_t>(rawHour) & 0x3F);
  const uint8_t day = bcdToBinary(static_cast<uint8_t>(rawDay) & 0x3F);
  const uint8_t month = bcdToBinary(static_cast<uint8_t>(rawMonth) & 0x1F);
  const int year = 2000 + bcdToBinary(static_cast<uint8_t>(rawYear));
  if (!validDateTime(year, month, day, hour, minute, second)) {
    return false;
  }

  const int64_t days = daysFromCivil(year, month, day);
  const int64_t value =
      days * 86400 + static_cast<int64_t>(hour) * 3600 + minute * 60 + second;
  if (value < 0 || value > UINT32_MAX) {
    return false;
  }
  *unixSeconds = static_cast<uint32_t>(value);
  return true;
}

bool boardRtcSetUnixSeconds(uint32_t unixSeconds) {
  const time_t seconds = static_cast<time_t>(unixSeconds);
  tm utc = {};
  if (gmtime_r(&seconds, &utc) == nullptr) {
    return false;
  }
  const int year = utc.tm_year + 1900;
  const uint8_t month = static_cast<uint8_t>(utc.tm_mon + 1);
  const uint8_t day = static_cast<uint8_t>(utc.tm_mday);
  const uint8_t hour = static_cast<uint8_t>(utc.tm_hour);
  const uint8_t minute = static_cast<uint8_t>(utc.tm_min);
  const uint8_t second = static_cast<uint8_t>(utc.tm_sec);
  if (!validDateTime(year, month, day, hour, minute, second)) {
    return false;
  }

  return writeReg(kPcf8563, kPcfRegSeconds, binaryToBcd(second)) == ESP_OK &&
         writeReg(kPcf8563, kPcfRegMinutes, binaryToBcd(minute)) == ESP_OK &&
         writeReg(kPcf8563, kPcfRegHours, binaryToBcd(hour)) == ESP_OK &&
         writeReg(kPcf8563, kPcfRegDays, binaryToBcd(day)) == ESP_OK &&
         writeReg(kPcf8563, kPcfRegWeekdays, binaryToBcd(weekdayFromUnix(unixSeconds))) ==
             ESP_OK &&
         writeReg(kPcf8563, kPcfRegMonths, binaryToBcd(month)) == ESP_OK &&
         writeReg(kPcf8563, kPcfRegYears, binaryToBcd(static_cast<uint8_t>(year - 2000))) ==
             ESP_OK;
}

}  // namespace app
