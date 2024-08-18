#pragma once

#include "ImuHal.h"
#include "SparkFun_LSM6DSV16X.h"

class LSM6DSV16X : public ImuHal {
  public:
    explicit LSM6DSV16X(TwoWire& wire) : _wire(&wire) {}

    LSM6DSV16X(const LSM6DSV16X&)            = delete;
    LSM6DSV16X& operator=(const LSM6DSV16X&) = delete;

    bool begin();

    std::optional<AccData> read_acc() override;
    std::optional<GyroData> read_gyro() override;

  private:
    SparkFun_LSM6DSV16X _lsm;
    TwoWire* _wire;
};
