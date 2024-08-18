#include "LSM6DSV16X.h"

namespace {
const char* const TAG{"LSM6DSV16X"};
}

bool LSM6DSV16X::begin() {
    // _wire set by reference in ctor so can't be null
    if (!_lsm.begin(*_wire)) {
        ESP_LOGE(TAG, "Could not start accelerometer");
        return false;
    }

    // Reset the device to default settings
    if (!_lsm.deviceReset()) {
        ESP_LOGE(TAG, "Could not reset LSM device");
        return false;
    }

    // Wait for it to finish resetting
    while (!_lsm.getDeviceReset()) {
        vTaskDelay(1);
    }

    ESP_LOGD(TAG, "Board reset, applying settings.");

    // Accelerometer and Gyroscope registers will not be updated
    // until read.
    _lsm.enableBlockDataUpdate();

    // Set output data rates
    _lsm.setAccelDataRate(LSM6DSV16X_ODR_AT_960Hz);
    _lsm.setGyroDataRate(LSM6DSV16X_ODR_AT_960Hz);

    // Set precisions
    _lsm.setGyroFullScale(LSM6DSV16X_2000dps);  // 333 RPM - plenty for cadence!
    _lsm.setAccelFullScale(LSM6DSV16X_16g);

    // Enable filter settling.
    _lsm.enableFilterSettling();

    // Turn on the accelerometer's filter and apply settings.
    _lsm.enableAccelLP2Filter();
    _lsm.setAccelLP2Bandwidth(LSM6DSV16X_XL_STRONG);

    // Turn on the gyroscope's filter and apply settings.
    _lsm.enableGyroLP1Filter();
    _lsm.setGyroLP1Bandwidth(LSM6DSV16X_GY_MEDIUM);

    return true;
}

std::optional<AccData> LSM6DSV16X::read_acc() {
    sfe_lsm_data_t acc_data;

    if (!_lsm.getAccel(&acc_data)) {
        return std::nullopt;
    }

    return AccData(acc_data.xData, acc_data.yData, acc_data.zData);
}

std::optional<GyroData> LSM6DSV16X::read_gyro() {
    sfe_lsm_data_t gyr_data;

    if (!_lsm.getGyro(&gyr_data)) {
        return std::nullopt;
    }

    return GyroData(gyr_data.xData, gyr_data.yData, gyr_data.zData);
}
