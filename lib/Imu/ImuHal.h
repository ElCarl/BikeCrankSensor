#pragma once

#include <optional>

#include "ImuTypes.h"

class ImuHal {
  public:
    virtual std::optional<AccData> read_acc()   = 0;
    virtual std::optional<GyroData> read_gyro() = 0;
};
