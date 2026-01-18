#include "BLEModule.h"
#include "logger.h"

static const char* TAG = "BLE";

BLEModule::BLEModule()
    : initialized_(false)
    , server_(nullptr)
    , deviceInfoService_(nullptr)
    , beaconService_(nullptr)
    , beaconDataChar_(nullptr)
    , beaconConfigChar_(nullptr) {
}

BLEModule::~BLEModule() {
    if (initialized_) {
        NimBLEDevice::deinit(true);
    }
}

bool BLEModule::init() {
    if (initialized_) {
        LOGW(TAG, "Already initialized");
        return true;
    }

    LOGI(TAG, "Initializing NimBLE device: %s", BLE_DEVICE_NAME);
    NimBLEDevice::init(BLE_DEVICE_NAME);

    server_ = NimBLEDevice::createServer();
    if (!server_) {
        LOGE(TAG, "Failed to create BLE server");
        return false;
    }
    server_->setCallbacks(&serverCallbacks_);

    if (!createDeviceInfoService()) {
        LOGE(TAG, "Failed to create Device Info service");
        return false;
    }

    if (!createBeaconService()) {
        LOGE(TAG, "Failed to create Beacon service");
        return false;
    }

    deviceInfoService_->start();
    beaconService_->start();

    NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
    advertising->setName(BLE_DEVICE_NAME);
    advertising->addServiceUUID(deviceInfoService_->getUUID());
    advertising->addServiceUUID(beaconService_->getUUID());
    advertising->enableScanResponse(true);

    initialized_ = true;
    LOGI(TAG, "BLE initialized successfully");
    return true;
}

bool BLEModule::createDeviceInfoService() {
    deviceInfoService_ = server_->createService(SERVICE_UUID_CONFIG);
    if (!deviceInfoService_) {
        return false;
    }

    NimBLECharacteristic* nameChar = deviceInfoService_->createCharacteristic(
        CHAR_UUID_DEVICE_NAME,
        NIMBLE_PROPERTY::READ
    );
    nameChar->setValue(BLE_DEVICE_NAME);

    NimBLECharacteristic* fwChar = deviceInfoService_->createCharacteristic(
        CHAR_UUID_FIRMWARE_REV,
        NIMBLE_PROPERTY::READ
    );
    fwChar->setValue("1.0.0");

    return true;
}

bool BLEModule::createBeaconService() {
    beaconService_ = server_->createService(SERVICE_UUID_BEACON);
    if (!beaconService_) {
        return false;
    }

    beaconDataChar_ = beaconService_->createCharacteristic(
        CHAR_UUID_BEACON_DATA,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    beaconDataChar_->setCallbacks(&charCallbacks_);

    beaconConfigChar_ = beaconService_->createCharacteristic(
        CHAR_UUID_BEACON_CONFIG,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE
    );
    beaconConfigChar_->setValue("default");
    beaconConfigChar_->setCallbacks(&charCallbacks_);

    NimBLE2904* formatDesc = beaconConfigChar_->create2904();
    formatDesc->setFormat(NimBLE2904::FORMAT_UTF8);
    formatDesc->setCallbacks(&descCallbacks_);

    return true;
}

void BLEModule::startAdvertising() {
    if (!initialized_) {
        LOGW(TAG, "Cannot advertise: not initialized");
        return;
    }
    NimBLEDevice::startAdvertising();
    LOGI(TAG, "Advertising started");
}

void BLEModule::stopAdvertising() {
    if (!initialized_) {
        return;
    }
    NimBLEDevice::stopAdvertising();
    LOGI(TAG, "Advertising stopped");
}

bool BLEModule::isAdvertising() const {
    if (!initialized_) {
        return false;
    }
    return NimBLEDevice::getAdvertising()->isAdvertising();
}

uint16_t BLEModule::getConnectedCount() const {
    if (!initialized_ || !server_) {
        return 0;
    }
    return server_->getConnectedCount();
}

bool BLEModule::notifyBeaconData(const uint8_t* data, size_t length) {
    if (!initialized_ || !beaconDataChar_) {
        return false;
    }
    beaconDataChar_->setValue(data, length);
    beaconDataChar_->notify();
    return true;
}

bool BLEModule::setBeaconConfig(const char* config) {
    if (!initialized_ || !beaconConfigChar_) {
        return false;
    }
    beaconConfigChar_->setValue(config);
    return true;
}

std::string BLEModule::getBeaconConfig() const {
    if (!initialized_ || !beaconConfigChar_) {
        return "";
    }
    return beaconConfigChar_->getValue();
}
