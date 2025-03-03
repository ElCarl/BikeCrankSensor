#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include <optional>

#include "Adafruit_ADS1X15.h"
#include "ImuHal.h"
#include "ImuTypes.h"
#include "LSM6DSV16X.h"
#include "MicrosecondTimer.h"
#include "Vector3D.h"

LSM6DSV16X imu(Wire);
Adafruit_ADS1115 ads1115;

const char* const TAG{"Main"};

// Bluetooth assigned IDs
// https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Assigned_Numbers/out/en/Assigned_Numbers.pdf?v=1723972973095
// Service UUIDs
constexpr char power_service_uuid[] = "1818";

// Characteristic UUIDs
constexpr char power_sensor_uuid[]                = "0484";
constexpr char cycling_power_control_point_uuid[] = "2a66";  // Probably won't implement yet
constexpr char cycling_power_feature_uuid[]       = "2a65";
constexpr char cycling_power_measurement_uuid[]   = "2a63";
constexpr char cycling_power_vector_uuid[]        = "2a64";

constexpr char service_uuid[] = "b0c1e97f-9b79-4bf8-bfd2-942ec4bd943f";

// Characteristics
constexpr char period_uuid[]     = "df0a6fb4-701d-4de6-87aa-0a0bb7cf27d1";
constexpr char timestamp_uuid[]  = "df0a6fb5-701d-4de6-87aa-0a0bb7cf27d1";
constexpr char acc_x_uuid[]      = "df0a6fb6-701d-4de6-87aa-0a0bb7cf27d1";
constexpr char acc_y_uuid[]      = "df0a6fb7-701d-4de6-87aa-0a0bb7cf27d1";
constexpr char acc_z_uuid[]      = "df0a6fb8-701d-4de6-87aa-0a0bb7cf27d1";
constexpr char gyr_x_uuid[]      = "df0a6fb9-701d-4de6-87aa-0a0bb7cf27d1";
constexpr char gyr_y_uuid[]      = "df0a6fba-701d-4de6-87aa-0a0bb7cf27d1";
constexpr char gyr_z_uuid[]      = "df0a6fbb-701d-4de6-87aa-0a0bb7cf27d1";
constexpr char amp_strain_uuid[] = "df0a6fbc-701d-4de6-87aa-0a0bb7cf27d1";
constexpr char raw_strain_uuid[] = "df0a6fbd-701d-4de6-87aa-0a0bb7cf27d1";

// RW characteristics
BLECharacteristic* period_ms{nullptr};

// Direct-access characteristics for debug/dev stuff
BLECharacteristic* timestamp{nullptr};
BLECharacteristic* acc_x{nullptr};
BLECharacteristic* acc_y{nullptr};
BLECharacteristic* acc_z{nullptr};
BLECharacteristic* gyr_x{nullptr};
BLECharacteristic* gyr_y{nullptr};
BLECharacteristic* gyr_z{nullptr};
BLECharacteristic* amp_strain{nullptr};
BLECharacteristic* raw_strain{nullptr};

BLEDescriptor period_desc("2901", 32);
BLEDescriptor timestamp_desc("2901", 32);
BLEDescriptor acc_x_desc("2901", 32);
BLEDescriptor acc_y_desc("2901", 32);
BLEDescriptor acc_z_desc("2901", 32);
BLEDescriptor gyr_x_desc("2901", 32);
BLEDescriptor gyr_y_desc("2901", 32);
BLEDescriptor gyr_z_desc("2901", 32);
BLEDescriptor amp_strain_desc("2901", 32);
BLEDescriptor raw_strain_desc("2901", 32);

uint32_t sample_period_ms{500};

// This is useful for initial testing but will be biased by other rotations. Probably shouldn't
// affect accuracy too much though - roll and yaw rates won't be *that* high!
constexpr float mdps_to_rpm(float mdps) {
    // mdps -> dps -> rps -> rpm
    // dps = mdps / 1000
    // rps = dps / 360
    // rpm = rps * 60
    // rpm = (dps / 360) * 60
    // rpm = ((mdps / 1000) / 360) * 60
    // rpm = ((mdps / 1000) / 6)
    // rpm = (mdps / 6000)
    return mdps / 6000.0F;
}

class U32SetCallback : public BLECharacteristicCallbacks {
  public:
    U32SetCallback(uint32_t& u) : _u(&u) {}

  private:
    void onWrite(BLECharacteristic* pCharacteristic, esp_ble_gatts_cb_param_t* param) override {
        uint32_t new_val{0};

        // Big-endian
        for (int i = 0; i < param->write.len; i++) {
            ESP_LOGD(TAG, "%i: %u", i, param->write.value[i]);
            new_val <<= 8;
            new_val += param->write.value[i];
        }
        ESP_LOGD(TAG, "Changing u32 (%p) to %u", _u, new_val);
        *_u = new_val;
    }

    uint32_t* _u;
};

U32SetCallback sample_set_callback(sample_period_ms);

void init_ble() {
    ESP_LOGI(TAG, "Starting BLE work!");

    BLEDevice::init("CrankSensor");
    BLEServer* server           = BLEDevice::createServer();
    BLEService* general_service = server->createService(service_uuid);
    BLEService* power_service   = server->createService(power_service_uuid);

    period_ms = general_service->createCharacteristic(
        period_uuid,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    timestamp =
        general_service->createCharacteristic(timestamp_uuid, BLECharacteristic::PROPERTY_READ);
    acc_x = general_service->createCharacteristic(acc_x_uuid, BLECharacteristic::PROPERTY_READ);
    acc_y = general_service->createCharacteristic(acc_y_uuid, BLECharacteristic::PROPERTY_READ);
    acc_z = general_service->createCharacteristic(acc_z_uuid, BLECharacteristic::PROPERTY_READ);
    gyr_x = general_service->createCharacteristic(gyr_x_uuid, BLECharacteristic::PROPERTY_READ);
    gyr_y = general_service->createCharacteristic(gyr_y_uuid, BLECharacteristic::PROPERTY_READ);
    gyr_z = general_service->createCharacteristic(gyr_z_uuid, BLECharacteristic::PROPERTY_READ);
    amp_strain =
        general_service->createCharacteristic(amp_strain_uuid, BLECharacteristic::PROPERTY_READ);
    raw_strain =
        general_service->createCharacteristic(raw_strain_uuid, BLECharacteristic::PROPERTY_READ);

    // Descriptors
    period_desc.setValue("Update period (ms)");
    timestamp_desc.setValue("Timestamp (ms)");
    acc_x_desc.setValue("Acc x (mg)");
    acc_y_desc.setValue("Acc y (mg)");
    acc_z_desc.setValue("Acc z (mg)");
    gyr_x_desc.setValue("Gyr x (mdps)");
    gyr_y_desc.setValue("Gyr y (mdps)");
    gyr_z_desc.setValue("Gyr z (mdps)");
    amp_strain_desc.setValue("Strain amp (counts)");
    raw_strain_desc.setValue("Strain raw (counts)");

    period_ms->addDescriptor(&period_desc);
    timestamp->addDescriptor(&timestamp_desc);
    acc_x->addDescriptor(&acc_x_desc);
    acc_y->addDescriptor(&acc_y_desc);
    acc_z->addDescriptor(&acc_z_desc);
    gyr_x->addDescriptor(&gyr_x_desc);
    gyr_y->addDescriptor(&gyr_y_desc);
    gyr_z->addDescriptor(&gyr_z_desc);
    amp_strain->addDescriptor(&amp_strain_desc);
    raw_strain->addDescriptor(&raw_strain_desc);

    period_ms->setValue(sample_period_ms);
    BLECharacteristicCallbacks cbs;

    period_ms->setCallbacks(&sample_set_callback);

    uint32_t time{xTaskGetTickCount()};
    timestamp->setValue(time);

    // Reference-only arguments are surely terrible!
    uint32_t zero{0};
    acc_x->setValue(zero);
    acc_y->setValue(zero);
    acc_z->setValue(zero);
    gyr_x->setValue(zero);
    gyr_y->setValue(zero);
    gyr_z->setValue(zero);
    amp_strain->setValue(zero);
    raw_strain->setValue(zero);

    general_service->start();

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(service_uuid);
    pAdvertising->setScanResponse(true);

    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);

    BLEDevice::startAdvertising();
    ESP_LOGI(TAG, "Characteristics defined");
}

void init_imu() {
    if (!imu.begin()) {
        ESP_LOGE(TAG, "IMU failed to start");
    }
}

void init_adc() {
    ads1115.begin(0x48, &Wire);
    ads1115.setDataRate(RATE_ADS1115_860SPS);
    ads1115.setGain(GAIN_FOUR);
}

void setup() {
    Serial.begin(115200);
    Wire.begin(6, 7, 400000);
    delay(500);
    while (!Serial);

    pinMode(23, OUTPUT);

    MicrosecondTimer timer("Setup");
    init_imu();
    timer.print_time("imu");
    init_adc();
    timer.print_time("adc");
    init_ble();
    timer.print_time("ble");

    Serial.println("Setup complete");
}

void set_acc_characteristics(AccData data) {
    float x{data.x()};
    float y{data.y()};
    float z{data.z()};

    acc_x->setValue(x);
    acc_y->setValue(y);
    acc_z->setValue(z);
}

void set_gyro_characteristics(GyroData data) {
    float x{data.x()};
    float y{data.y()};
    float z{data.z()};

    gyr_x->setValue(x);
    gyr_y->setValue(y);
    gyr_z->setValue(z);
}

void loop() {
    static TickType_t last_update{xTaskGetTickCount()};

    MicrosecondTimer loop_timer("loop");

    // Timestamp
    uint32_t t{xTaskGetTickCount()};
    timestamp->setValue(t);

    loop_timer.print_time("timestamp");

    // IMU

    uint32_t start_us{micros()};
    auto acc_data{imu.read_acc()};
    auto gyro_data{imu.read_gyro()};
    uint32_t duration_us{micros() - start_us};

    loop_timer.print_time("imu-read");

    if (acc_data.has_value()) {
        set_acc_characteristics(*acc_data);
    }
    if (gyro_data.has_value()) {
        set_gyro_characteristics(*gyro_data);
    }

    loop_timer.print_time("imu-char-set");

    // ADC
    // TODO interleave this with ADC or other work to reduce time waiting for
    // conversion
    int ampd{ads1115.readADC_Differential_0_1()};
    int raw{ads1115.readADC_Differential_2_3()};

    loop_timer.print_time("adc-read");

    amp_strain->setValue(ampd);
    raw_strain->setValue(raw);

    loop_timer.print_time("adc-char-set");

    if (acc_data.has_value() && gyro_data.has_value()) {
        float acc_mag_mg{acc_data->magnitude()};
        float gyr_mag_mdps{gyro_data->magnitude()};

        loop_timer.print_time("imu-mag");

        ESP_LOGI(TAG,
                 "IMU magnitudes: Acc: %.2f m/s^2, Gyr: %.2f RPM",
                 acc_mag_mg / 100.0F,
                 mdps_to_rpm(gyr_mag_mdps));

        loop_timer.print_time("imu-mag-print");

        ESP_LOGI(TAG,
                 "IMU: %lu us, Acc: %+8.1f %+8.1f %+8.1f, Gyro: %+8.0f %+8.0f %+8.0f",
                 duration_us,
                 acc_data->x(),
                 acc_data->y(),
                 acc_data->z(),
                 gyro_data->x(),
                 gyro_data->y(),
                 gyro_data->z());
        loop_timer.print_time("imu-print");
    } else {
        ESP_LOGE(TAG, "One or both of acc and gyro data read failed");
    }

    ESP_LOGI(TAG, "ADC: Raw: %i, Amp: %i", raw, ampd);

    loop_timer.print_time("adc-print");
    digitalWrite(23, LOW);
    xTaskDelayUntil(&last_update, pdMS_TO_TICKS(sample_period_ms));
    loop_timer.print_time("delay");
    digitalWrite(23, HIGH);
}
