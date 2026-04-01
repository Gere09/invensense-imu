#ifndef INVENSENSE_IMU_SRC_ICM20602_H_
#define INVENSENSE_IMU_SRC_ICM20602_H_

#if defined(ARDUINO)
#include <Arduino.h>
#include "Wire.h"
#include "SPI.h"
#else
#include <cstddef>
#include <cstdint>
#include "core/core.h"
#endif
#include "invensense_imu.h"

namespace bfs {

class Icm20602 {
 public:
  enum I2cAddr : uint8_t {
    I2C_ADDR_PRIM = 0x68,
    I2C_ADDR_SEC  = 0x69
  };
  enum DlpfBandwidth : int8_t {
    DLPF_BANDWIDTH_218HZ = 0x00,
    DLPF_BANDWIDTH_99HZ  = 0x02,
    DLPF_BANDWIDTH_45HZ  = 0x03,
    DLPF_BANDWIDTH_20HZ  = 0x04,
    DLPF_BANDWIDTH_10HZ  = 0x05,
    DLPF_BANDWIDTH_5HZ   = 0x06
  };
  enum AccelRange : int8_t {
    ACCEL_RANGE_2G  = 0x00,
    ACCEL_RANGE_4G  = 0x08,
    ACCEL_RANGE_8G  = 0x10,
    ACCEL_RANGE_16G = 0x18
  };
  enum GyroRange : int8_t {
    GYRO_RANGE_250DPS  = 0x00,
    GYRO_RANGE_500DPS  = 0x08,
    GYRO_RANGE_1000DPS = 0x10,
    GYRO_RANGE_2000DPS = 0x18
  };

  Icm20602() {}
  Icm20602(TwoWire *i2c, const I2cAddr addr) :
           imu_(i2c, static_cast<uint8_t>(addr)) {}
  Icm20602(SPIClass *spi, const uint8_t cs) :
           imu_(spi, cs) {}

  bool Begin();
  bool ConfigAccelRange(const AccelRange range);
  bool ConfigGyroRange(const GyroRange range);
  bool ConfigSrd(const uint8_t srd);
  bool ConfigDlpfBandwidth(const DlpfBandwidth dlpf);
  bool Read();

  inline float accel_x_mps2() const { return accel_[0]; }
  inline float accel_y_mps2() const { return accel_[1]; }
  inline float accel_z_mps2() const { return accel_[2]; }
  inline float gyro_x_radps() const { return gyro_[0]; }
  inline float gyro_y_radps() const { return gyro_[1]; }
  inline float gyro_z_radps() const { return gyro_[2]; }
  inline float die_temp_c()   const { return temp_; }
  inline bool  new_imu_data() const { return new_imu_data_; }

 private:
  InvensenseImu imu_;
  int32_t spi_clock_;
  static constexpr int32_t SPI_CFG_CLOCK_  = 1000000;
  static constexpr int32_t SPI_READ_CLOCK_ = 15000000;

  AccelRange    accel_range_, requested_accel_range_;
  GyroRange     gyro_range_,  requested_gyro_range_;
  DlpfBandwidth dlpf_bandwidth_, requested_dlpf_;
  float accel_scale_, requested_accel_scale_;
  float gyro_scale_,  requested_gyro_scale_;
  uint8_t srd_;

  static constexpr float TEMP_SCALE_ = 326.8f;
  static constexpr float TEMP_OFFSET_ = 25.0f;
  uint8_t who_am_i_;
  static constexpr uint8_t WHOAMI_ICM20602_ = 0x12;

  static constexpr float G_MPS2_  = 9.80665f;
  static constexpr float DEG2RAD_ = 3.14159265358979323846f / 180.0f;

  bool new_imu_data_;
  uint8_t data_buf_[15];
  int16_t accel_cnts_[3], gyro_cnts_[3];
  int16_t temp_cnts_;
  float accel_[3], gyro_[3], temp_;

  // Registri (identici a MPU6500)
  static constexpr uint8_t PWR_MGMNT_1_    = 0x6B;
  static constexpr uint8_t CLKSEL_PLL_     = 0x01;
  static constexpr uint8_t WHOAMI_         = 0x75;
  static constexpr uint8_t ACCEL_CONFIG_   = 0x1C;
  static constexpr uint8_t GYRO_CONFIG_    = 0x1B;
  static constexpr uint8_t ACCEL_CONFIG2_  = 0x1D;
  static constexpr uint8_t CONFIG_         = 0x1A;
  static constexpr uint8_t SMPLRT_DIV_     = 0x19;
  static constexpr uint8_t INT_STATUS_     = 0x3A;
  static constexpr uint8_t RAW_DATA_RDY_INT_ = 0x01;

  bool WriteRegister(const uint8_t reg, const uint8_t data);
  bool ReadRegisters(const uint8_t reg, const uint8_t count, uint8_t * const data);
};

}  // namespace bfs
#endif