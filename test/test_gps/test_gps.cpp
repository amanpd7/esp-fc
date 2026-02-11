// test/test_gps/test_gps.cpp

#include <unity.h>
#include "Sensor/GpsSensor.hpp"
#include "Model.h"

using namespace Espfc;
using namespace Espfc::Sensor;

// Helper function to set up GPS with valid data
void setupGpsWithFix(Model& model) {
    model.state.gps.fix = true;
    model.state.gps.fixType = 3;  // 3D fix
    model.state.gps.numSats = 10;
    model.state.gps.homeSet = true;
}

void test_gps_distance_calculation() {
    Model model;
    GpsSensor gps(model);

    // Set home position (0, 0)
    model.state.gps.home.raw.lat = 0;
    model.state.gps.home.raw.lon = 0;
    
    // Set current position ~1° north (≈ 111.2 km)
    model.state.gps.location.raw.lat = 10000000;   // 1 degree = 10^7 scaling
    model.state.gps.location.raw.lon = 0;

    setupGpsWithFix(model);

    // Call public update method which internally calls calculateHomeVector
    gps.update();

    // Expected: ~111195–111320 meters depending on exact ellipsoid formula
    // Using ±2000 m tolerance for floating-point / implementation differences
    TEST_ASSERT_INT_WITHIN(2000, 111200, model.state.gps.distanceToHome);

    // Should be pointing north (0°)
    TEST_ASSERT_INT_WITHIN(5, 0, model.state.gps.directionToHome);
}

void test_gps_bearing_calculation() {
    Model model;
    GpsSensor gps(model);

    // Home at (0, 0)
    model.state.gps.home.raw.lat = 0;
    model.state.gps.home.raw.lon = 0;

    // Current position 1° east
    model.state.gps.location.raw.lat = 0;
    model.state.gps.location.raw.lon = 10000000;

    setupGpsWithFix(model);

    // Call public update method
    gps.update();

    // Should be ~90° (east)
    TEST_ASSERT_INT_WITHIN(5, 90, model.state.gps.directionToHome);
}

void test_gps_bearing_northeast() {
    Model model;
    GpsSensor gps(model);

    // Home at (0, 0)
    model.state.gps.home.raw.lat = 0;
    model.state.gps.home.raw.lon = 0;

    // Current position 1° north, 1° east (northeast)
    model.state.gps.location.raw.lat = 10000000;
    model.state.gps.location.raw.lon = 10000000;

    setupGpsWithFix(model);

    gps.update();

    // Should be ~45° (northeast)
    TEST_ASSERT_INT_WITHIN(5, 45, model.state.gps.directionToHome);
}

void test_gps_bearing_south() {
    Model model;
    GpsSensor gps(model);

    // Home at equator
    model.state.gps.home.raw.lat = 0;
    model.state.gps.home.raw.lon = 0;

    // Current position 1° south
    model.state.gps.location.raw.lat = -10000000;
    model.state.gps.location.raw.lon = 0;

    setupGpsWithFix(model);

    gps.update();

    // Should be 180° (south) or -180°
    int16_t direction = model.state.gps.directionToHome;
    TEST_ASSERT_TRUE(direction == 180 || direction == -180);
}

void test_gps_auto_home_on_arm() {
    Model model;
    GpsSensor gps(model);

    // Configure auto home
    model.config.gps.autoSetHome = 1;
    model.config.gps.minSats = 8;

    // GPS has good fix
    model.state.gps.fix = true;
    model.state.gps.fixType = 3;
    model.state.gps.numSats = 10;
    model.state.gps.location.raw.lat = 377490000;  // San Francisco
    model.state.gps.location.raw.lon = -1224194000;

    // Not armed, home not set
    model.state.gps.homeSet = false;
    model.state.mode.mask &= ~(1 << MODE_ARMED);

    // Update - should not set home (not armed)
    gps.update();
    TEST_ASSERT_FALSE(model.state.gps.homeSet);

    // Now arm
    model.state.mode.mask &= ~(1 << MODE_ARMED);

    // Update - should set home
    gps.update();
    TEST_ASSERT_TRUE(model.state.gps.homeSet);
    TEST_ASSERT_EQUAL_INT32(377490000, model.state.gps.home.raw.lat);
    TEST_ASSERT_EQUAL_INT32(-1224194000, model.state.gps.home.raw.lon);
}

void test_gps_min_sats_requirement() {
    Model model;
    GpsSensor gps(model);

    model.config.gps.minSats = 8;
    model.config.gps.autoSetHome = 1;

    // GPS has fix but not enough satellites
    model.state.gps.fix = true;
    model.state.gps.fixType = 3;
    model.state.gps.numSats = 6;  // Less than minimum
    model.state.gps.location.raw.lat = 377490000;
    model.state.gps.location.raw.lon = -1224194000;
    model.state.gps.homeSet = false;
    model.state.mode.mask &= ~(1 << MODE_ARMED);

    // Update - should NOT set home (not enough sats)
    gps.update();
    TEST_ASSERT_FALSE(model.state.gps.homeSet);

    // Increase satellite count
    model.state.gps.numSats = 10;

    // Update - should now set home
    gps.update();
    TEST_ASSERT_TRUE(model.state.gps.homeSet);
}

void test_gps_distance_zero_at_home() {
    Model model;
    GpsSensor gps(model);

    // Set home and current location to same position
    model.state.gps.home.raw.lat = 377490000;
    model.state.gps.home.raw.lon = -1224194000;
    model.state.gps.location.raw.lat = 377490000;
    model.state.gps.location.raw.lon = -1224194000;

    setupGpsWithFix(model);

    gps.update();

    // Distance should be 0 or very close
    TEST_ASSERT_INT_WITHIN(1, 0, model.state.gps.distanceToHome);
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_gps_distance_calculation);
    RUN_TEST(test_gps_bearing_calculation);
    RUN_TEST(test_gps_bearing_northeast);
    RUN_TEST(test_gps_bearing_south);
    RUN_TEST(test_gps_auto_home_on_arm);
    RUN_TEST(test_gps_min_sats_requirement);
    RUN_TEST(test_gps_distance_zero_at_home);

    return UNITY_END();
}