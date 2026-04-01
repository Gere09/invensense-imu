#include "icm20602.h"
#if defined(ARDUINO)
#include <Arduino.h>
#include "Wire.h"
#include "SPI.h"
#endif

namespace bfs {

bool Icm20602::Begin() {
  imu_.Begin();
  spi_clock_ = SPI_CFG_CLOCK_;

  // Wake up, clock source auto
  if (!WriteRegister(PWR_MGMNT_1_, CLKSEL_PLL_)) return false;
  delay(10);

  // Verifica WHO_AM_I
  if (!ReadRegisters(WHOAMI_, sizeof(who_am_i_), &who_am_i_)) return false;
  if (who_am_i_ != WHOAMI_ICM20602_) return false;

  if (!ConfigAccelRange(ACCEL_RANGE_16G))        return false;
  if (!ConfigGyroRange(GYRO_RANGE_2000DPS))      return false;
  if (!ConfigDlpfBandwidth(DLPF_BANDWIDTH_20HZ)) return false;
  if (!ConfigSrd(0))                             return false;
  return true;
}

bool Icm20602::ConfigAccelRange(const AccelRange range) {
  spi_clock_ = SPI_CFG_CLOCK_;
  switch (range) {
    case ACCEL_RANGE_2G:  requested_accel_scale_ = 2.0f  / 32767.5f; break;
    case ACCEL_RANGE_4G:  requested_accel_scale_ = 4.0f  / 32767.5f; break;
    case ACCEL_RANGE_8G:  requested_accel_scale_ = 8.0f  / 32767.5f; break;
    case ACCEL_RANGE_16G: requested_accel_scale_ = 16.0f / 32767.5f; break;
    default: return false;
  }
  requested_accel_range_ = range;
  if (!WriteRegister(ACCEL_CONFIG_, requested_accel_range_)) return false;
  accel_range_ = requested_accel_range_;
  accel_scale_ = requested_accel_scale_;
  return true;
}

bool Icm20602::ConfigGyroRange(const GyroRange range) {
  spi_clock_ = SPI_CFG_CLOCK_;
  switch (range) {
    case GYRO_RANGE_250DPS:  requested_gyro_scale_ = 250.0f  / 32767.5f; break;
    case GYRO_RANGE_500DPS:  requested_gyro_scale_ = 500.0f  / 32767.5f; break;
    case GYRO_RANGE_1000DPS: requested_gyro_scale_ = 1000.0f / 32767.5f; break;
    case GYRO_RANGE_2000DPS: requested_gyro_scale_ = 2000.0f / 32767.5f; break;
    default: return false;
  }
  requested_gyro_range_ = range;
  if (!WriteRegister(GYRO_CONFIG_, requested_gyro_range_)) return false;
  gyro_range_ = requested_gyro_range_;
  gyro_scale_ = requested_gyro_scale_;
  return true;
}

bool Icm20602::ConfigSrd(const uint8_t srd) {
  spi_clock_ = SPI_CFG_CLOCK_;
  if (!WriteRegister(SMPLRT_DIV_, srd)) return false;
  srd_ = srd;
  return true;
}

bool Icm20602::ConfigDlpfBandwidth(const DlpfBandwidth dlpf) {
  spi_clock_ = SPI_CFG_CLOCK_;
  requested_dlpf_ = dlpf;
  if (!WriteRegister(ACCEL_CONFIG2_, requested_dlpf_)) return false;
  if (!WriteRegister(CONFIG_,        requested_dlpf_)) return false;
  dlpf_bandwidth_ = requested_dlpf_;
  return true;
}

bool Icm20602::Read() {
  spi_clock_ = SPI_READ_CLOCK_;
  new_imu_data_ = false;
  if (!ReadRegisters(INT_STATUS_, sizeof(data_buf_), data_buf_)) return false;
  new_imu_data_ = (data_buf_[0] & RAW_DATA_RDY_INT_);
  if (!new_imu_data_) return false;

  accel_cnts_[0] = static_cast<int16_t>(data_buf_[1])  << 8 | data_buf_[2];
  accel_cnts_[1] = static_cast<int16_t>(data_buf_[3])  << 8 | data_buf_[4];
  accel_cnts_[2] = static_cast<int16_t>(data_buf_[5])  << 8 | data_buf_[6];
  temp_cnts_     = static_cast<int16_t>(data_buf_[7])  << 8 | data_buf_[8];
  gyro_cnts_[0]  = static_cast<int16_t>(data_buf_[9])  << 8 | data_buf_[10];
  gyro_cnts_[1]  = static_cast<int16_t>(data_buf_[11]) << 8 | data_buf_[12];
  gyro_cnts_[2]  = static_cast<int16_t>(data_buf_[13]) << 8 | data_buf_[14];

  accel_[0] = static_cast<float>(accel_cnts_[1]) * accel_scale_ * G_MPS2_;
  accel_[1] = static_cast<float>(accel_cnts_[0]) * accel_scale_ * G_MPS2_;
  accel_[2] = static_cast<float>(accel_cnts_[2]) * accel_scale_ * -1.0f * G_MPS2_;
  temp_     = static_cast<float>(temp_cnts_) / TEMP_SCALE_ + TEMP_OFFSET_;
  gyro_[0]  = static_cast<float>(gyro_cnts_[1]) * gyro_scale_ * DEG2RAD_;
  gyro_[1]  = static_cast<float>(gyro_cnts_[0]) * gyro_scale_ * DEG2RAD_;
  gyro_[2]  = static_cast<float>(gyro_cnts_[2]) * gyro_scale_ * -1.0f * DEG2RAD_;
  return true;
}

bool Icm20602::WriteRegister(const uint8_t reg, const uint8_t data) {
  return imu_.WriteRegister(reg, data, spi_clock_);
}
bool Icm20602::ReadRegisters(const uint8_t reg, const uint8_t count,
                             uint8_t * const data) {
  return imu_.ReadRegisters(reg, count, spi_clock_, data);
}

}  // namespace bfs