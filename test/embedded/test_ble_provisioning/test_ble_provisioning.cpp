#include <Arduino.h>
#include <unity.h>
#include "BLEModule.h"

static BLEModule* ble = nullptr;

void setUp(void) {}
void tearDown(void) {}

// --- Provisioning lifecycle ---

void test_init_succeeds_when_enabled(void) {
    ble = new BLEModule(true);
    TEST_ASSERT_TRUE(ble->init());
    TEST_ASSERT_TRUE(ble->isInitialized());
}

void test_advertising_starts_after_init(void) {
    ble->startAdvertising();
    TEST_ASSERT_TRUE(ble->isAdvertising());
}

void test_stop_clears_initialized_flag(void) {
    ble->stop();
    TEST_ASSERT_FALSE(ble->isInitialized());
}

void test_stop_clears_advertising(void) {
    // After stop(), isAdvertising() must return false (not initialized guard)
    TEST_ASSERT_FALSE(ble->isAdvertising());
}

void test_connected_count_is_zero_after_stop(void) {
    TEST_ASSERT_EQUAL_UINT16(0, ble->getConnectedCount());
}

void test_no_pending_credentials_after_stop(void) {
    TEST_ASSERT_FALSE(ble->hasPendingWiFiCredentials());
}

// --- Crash / double-stop resilience ---

void test_stop_on_uninitialized_module_does_not_crash(void) {
    // Simulate a device that crashed before init completed, then rebooted
    // and calls stop() defensively.
    BLEModule uninitialized(true);
    uninitialized.stop(); // must not crash or assert
    TEST_PASS();
}

void test_double_stop_does_not_crash(void) {
    // A second stop() on an already-stopped module must be a no-op.
    ble->stop();
    TEST_PASS();
}

// --- Disabled module ---

void test_init_returns_false_when_disabled(void) {
    delete ble;
    ble = new BLEModule(false);
    TEST_ASSERT_FALSE(ble->init());
    TEST_ASSERT_FALSE(ble->isInitialized());
}

void test_stop_on_disabled_uninitialised_does_not_crash(void) {
    ble->stop();
    TEST_PASS();
}

void setup() {
    delay(2000);
    UNITY_BEGIN();

    // Provisioning lifecycle
    RUN_TEST(test_init_succeeds_when_enabled);
    RUN_TEST(test_advertising_starts_after_init);
    RUN_TEST(test_stop_clears_initialized_flag);
    RUN_TEST(test_stop_clears_advertising);
    RUN_TEST(test_connected_count_is_zero_after_stop);
    RUN_TEST(test_no_pending_credentials_after_stop);

    // Crash / double-stop resilience
    RUN_TEST(test_stop_on_uninitialized_module_does_not_crash);
    RUN_TEST(test_double_stop_does_not_crash);

    // Disabled module
    RUN_TEST(test_init_returns_false_when_disabled);
    RUN_TEST(test_stop_on_disabled_uninitialised_does_not_crash);

    delete ble;
    ble = nullptr;

    UNITY_END();
}

void loop() {}
