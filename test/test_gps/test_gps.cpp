#include <unity.h>
#include <cmath>
#include <cstdint>

// Haversine distance calculation (same as in GpsSensor.cpp)
float haversineDistance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2)
{
  const double R = 6371000.0; // Earth radius in meters
  
  double phi1 = lat1 * 1e-7 * M_PI / 180.0;
  double phi2 = lat2 * 1e-7 * M_PI / 180.0;
  double deltaPhi = (lat2 - lat1) * 1e-7 * M_PI / 180.0;
  double deltaLambda = (lon2 - lon1) * 1e-7 * M_PI / 180.0;
  
  double a = sin(deltaPhi / 2.0) * sin(deltaPhi / 2.0) +
             cos(phi1) * cos(phi2) *
             sin(deltaLambda / 2.0) * sin(deltaLambda / 2.0);
  
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  
  return R * c;
}

// Bearing calculation (same as in GpsSensor.cpp)
int16_t calculateBearing(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2)
{
  double phi1 = lat1 * 1e-7 * M_PI / 180.0;
  double phi2 = lat2 * 1e-7 * M_PI / 180.0;
  double deltaLambda = (lon2 - lon1) * 1e-7 * M_PI / 180.0;
  
  double y = sin(deltaLambda) * cos(phi2);
  double x = cos(phi1) * sin(phi2) -
             sin(phi1) * cos(phi2) * cos(deltaLambda);
  
  double theta = atan2(y, x);
  double bearing = fmod((theta * 180.0 / M_PI + 360.0), 360.0);
  
  return (int16_t)bearing;
}

// ============================================================================
// DISTANCE TESTS
// ============================================================================

void test_gps_distance_1_degree_north() {
    // 1 degree latitude ≈ 111.195 km (constant everywhere)
    int32_t lat1 = 0;
    int32_t lon1 = 0;
    int32_t lat2 = 10000000;  // 1 degree = 10^7
    int32_t lon2 = 0;
    
    float distance = haversineDistance(lat1, lon1, lat2, lon2);
    
    TEST_ASSERT_FLOAT_WITHIN(1000.0f, 111195.0f, distance);
}

void test_gps_distance_1_degree_east() {
    // At equator, 1 degree longitude ≈ 111.195 km
    int32_t lat1 = 0;
    int32_t lon1 = 0;
    int32_t lat2 = 0;
    int32_t lon2 = 10000000;
    
    float distance = haversineDistance(lat1, lon1, lat2, lon2);
    
    TEST_ASSERT_FLOAT_WITHIN(1000.0f, 111195.0f, distance);
}

void test_gps_distance_short() {
    // 0.0009 degrees ≈ 100m at equator
    int32_t lat1 = 0;
    int32_t lon1 = 0;
    int32_t lat2 = 9000;
    int32_t lon2 = 0;
    
    float distance = haversineDistance(lat1, lon1, lat2, lon2);
    
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 100.0f, distance);
}

void test_gps_distance_diagonal() {
    // Pythagorean: sqrt(111195^2 + 111195^2) ≈ 157km
    int32_t lat1 = 0;
    int32_t lon1 = 0;
    int32_t lat2 = 10000000;  // 1° north
    int32_t lon2 = 10000000;  // 1° east
    
    float distance = haversineDistance(lat1, lon1, lat2, lon2);
    
    TEST_ASSERT_FLOAT_WITHIN(1000.0f, 157249.0f, distance);
}

void test_gps_distance_zero() {
    // Same position should return 0
    int32_t lat = 401234567;
    int32_t lon = -740987654;
    
    float distance = haversineDistance(lat, lon, lat, lon);
    
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, distance);
}

void test_gps_distance_nyc_to_central_park() {
    // Times Square to Central Park ≈ 4.5 km
    int32_t lat1 = 407420000;  // 40.7420° N
    int32_t lon1 = -739930000; // -73.9930° W
    
    int32_t lat2 = 407820000;  // 40.7820° N (Central Park)
    int32_t lon2 = -739650000; // -73.9650° W
    
    float distance = haversineDistance(lat1, lon1, lat2, lon2);
    
    TEST_ASSERT_FLOAT_WITHIN(500.0f, 4800.0f, distance);
}

void test_gps_distance_across_equator() {
    // Test crossing the equator
    int32_t lat1 = 10000000;   // 1° N
    int32_t lon1 = 0;
    int32_t lat2 = -10000000;  // 1° S
    int32_t lon2 = 0;
    
    float distance = haversineDistance(lat1, lon1, lat2, lon2);
    
    TEST_ASSERT_FLOAT_WITHIN(2000.0f, 222390.0f, distance); // 2 degrees
}

void test_gps_distance_across_prime_meridian() {
    // Test crossing the prime meridian
    int32_t lat1 = 0;
    int32_t lon1 = 10000000;   // 1° E
    int32_t lat2 = 0;
    int32_t lon2 = -10000000;  // 1° W
    
    float distance = haversineDistance(lat1, lon1, lat2, lon2);
    
    TEST_ASSERT_FLOAT_WITHIN(2000.0f, 222390.0f, distance); // 2 degrees
}

// ============================================================================
// BEARING TESTS
// ============================================================================

void test_gps_bearing_north() {
    int32_t lat1 = 0;
    int32_t lon1 = 0;
    int32_t lat2 = 10000000;
    int32_t lon2 = 0;
    
    int16_t bearing = calculateBearing(lat1, lon1, lat2, lon2);
    
    TEST_ASSERT_INT16_WITHIN(1, 0, bearing);
}

void test_gps_bearing_east() {
    int32_t lat1 = 0;
    int32_t lon1 = 0;
    int32_t lat2 = 0;
    int32_t lon2 = 10000000;
    
    int16_t bearing = calculateBearing(lat1, lon1, lat2, lon2);
    
    TEST_ASSERT_INT16_WITHIN(1, 90, bearing);
}

void test_gps_bearing_south() {
    int32_t lat1 = 0;
    int32_t lon1 = 0;
    int32_t lat2 = -10000000;
    int32_t lon2 = 0;
    
    int16_t bearing = calculateBearing(lat1, lon1, lat2, lon2);
    
    TEST_ASSERT_INT16_WITHIN(1, 180, bearing);
}

void test_gps_bearing_west() {
    int32_t lat1 = 0;
    int32_t lon1 = 0;
    int32_t lat2 = 0;
    int32_t lon2 = -10000000;
    
    int16_t bearing = calculateBearing(lat1, lon1, lat2, lon2);
    
    TEST_ASSERT_INT16_WITHIN(1, 270, bearing);
}

void test_gps_bearing_northeast() {
    int32_t lat1 = 0;
    int32_t lon1 = 0;
    int32_t lat2 = 10000000;
    int32_t lon2 = 10000000;
    
    int16_t bearing = calculateBearing(lat1, lon1, lat2, lon2);
    
    TEST_ASSERT_INT16_WITHIN(2, 45, bearing);
}

void test_gps_bearing_southwest() {
    int32_t lat1 = 0;
    int32_t lon1 = 0;
    int32_t lat2 = -10000000;
    int32_t lon2 = -10000000;
    
    int16_t bearing = calculateBearing(lat1, lon1, lat2, lon2);
    
    TEST_ASSERT_INT16_WITHIN(2, 225, bearing);
}

void test_gps_bearing_nyc_to_boston() {
    // NYC to Boston is roughly NE, bearing ≈ 40-50°
    int32_t lat1 = 407420000;  // NYC
    int32_t lon1 = -739930000;
    
    int32_t lat2 = 423610000;  // Boston
    int32_t lon2 = -710590000;
    
    int16_t bearing = calculateBearing(lat1, lon1, lat2, lon2);
    
    TEST_ASSERT_GREATER_OR_EQUAL(35, bearing);
    TEST_ASSERT_LESS_OR_EQUAL(65, bearing);
}

void test_gps_bearing_reverse() {
    // Bearing from A to B vs B to A should differ by ≈180°
    int32_t lat1 = 0;
    int32_t lon1 = 0;
    int32_t lat2 = 10000000;
    int32_t lon2 = 10000000;
    
    int16_t bearing_forward = calculateBearing(lat1, lon1, lat2, lon2);
    int16_t bearing_reverse = calculateBearing(lat2, lon2, lat1, lon1);
    
    int16_t diff = abs(bearing_forward - bearing_reverse);
    // Should be close to 180° (allow some error for non-straight paths)
    TEST_ASSERT_INT16_WITHIN(10, 180, diff);
}

// ============================================================================
// COORDINATE SCALING TESTS
// ============================================================================

void test_gps_coordinate_scaling() {
    // Verify 1 degree = 10,000,000 in scaled format
    int32_t one_degree = 10000000;
    float converted = one_degree * 1e-7f;
    
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, converted);
}

void test_gps_negative_coordinates() {
    // Test negative coordinates (Southern/Western hemispheres)
    int32_t neg_lat = -401234567;
    int32_t neg_lon = -740987654;
    
    float lat_degrees = neg_lat * 1e-7f;
    float lon_degrees = neg_lon * 1e-7f;
    
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -40.1234567f, lat_degrees);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -74.0987654f, lon_degrees);
}

void test_gps_max_coordinates() {
    // Test maximum valid coordinates (90°, 180°)
    int32_t max_lat = 900000000;   // 90° N
    int32_t max_lon = 1800000000;  // 180° E
    
    float lat_degrees = max_lat * 1e-7f;
    float lon_degrees = max_lon * 1e-7f;
    
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 90.0f, lat_degrees);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 180.0f, lon_degrees);
}

// ============================================================================
// EDGE CASE TESTS
// ============================================================================

void test_gps_same_position() {
    int32_t pos = 401234567;
    
    float distance = haversineDistance(pos, pos, pos, pos);
    
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, distance);
    
    // Note: Bearing is undefined for same position, so we don't test it
    // to avoid potential divide-by-zero or NaN issues
}

void test_gps_very_short_distance() {
    // 0.00001 degrees ≈ 1.1 meters (more realistic minimum)
    int32_t lat1 = 0;
    int32_t lon1 = 0;
    int32_t lat2 = 100;  // Increased from 1 to 100
    int32_t lon2 = 0;
    
    float distance = haversineDistance(lat1, lon1, lat2, lon2);
    
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1.1f, distance);
}

void test_gps_bearing_range() {
    // Test bearing is always in [0, 360) range
    int32_t lat1 = 401234567;
    int32_t lon1 = -740987654;
    
    for (int i = 0; i < 8; i++) {
        int32_t lat2 = lat1 + (i % 3 - 1) * 10000000;
        int32_t lon2 = lon1 + (i / 3 - 1) * 10000000;
        
        if (lat2 == lat1 && lon2 == lon1) continue;
        
        int16_t bearing = calculateBearing(lat1, lon1, lat2, lon2);
        
        TEST_ASSERT_GREATER_OR_EQUAL(0, bearing);
        TEST_ASSERT_LESS_THAN(360, bearing);
    }
}

void test_gps_high_latitude() {
    // Test near poles where longitude convergence matters
    int32_t lat1 = 850000000;  // 85° N
    int32_t lon1 = 0;
    int32_t lat2 = 850000000;
    int32_t lon2 = 10000000;   // 1° longitude difference
    
    float distance = haversineDistance(lat1, lon1, lat2, lon2);
    
    // At 85° latitude, 1° longitude ≈ 9.7 km (much less than at equator)
    TEST_ASSERT_LESS_THAN(20000.0f, distance);
}

// ============================================================================
// PRECISION TESTS
// ============================================================================

void test_gps_precision_1_meter() {
    // 0.000009 degrees ≈ 1 meter
    int32_t lat1 = 0;
    int32_t lon1 = 0;
    int32_t lat2 = 90;
    int32_t lon2 = 0;
    
    float distance = haversineDistance(lat1, lon1, lat2, lon2);
    
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 1.0f, distance);
}

void test_gps_precision_10_meters() {
    // 0.00009 degrees ≈ 10 meters
    int32_t lat1 = 0;
    int32_t lon1 = 0;
    int32_t lat2 = 900;
    int32_t lon2 = 0;
    
    float distance = haversineDistance(lat1, lon1, lat2, lon2);
    
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 10.0f, distance);
}

void test_gps_precision_100_meters() {
    // Typical drone flight distance
    int32_t lat1 = 266960000;  // Siliguri area
    int32_t lon1 = 884487000;
    int32_t lat2 = 266969000;  // ~100m north
    int32_t lon2 = 884487000;
    
    float distance = haversineDistance(lat1, lon1, lat2, lon2);
    
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 100.0f, distance);
}

// ============================================================================
// MAIN
// ============================================================================

void setUp(void) {}
void tearDown(void) {}

int main() {
    UNITY_BEGIN();
    
    // Distance tests (8)
    RUN_TEST(test_gps_distance_1_degree_north);
    RUN_TEST(test_gps_distance_1_degree_east);
    RUN_TEST(test_gps_distance_short);
    RUN_TEST(test_gps_distance_diagonal);
    RUN_TEST(test_gps_distance_zero);
    RUN_TEST(test_gps_distance_nyc_to_central_park);
    RUN_TEST(test_gps_distance_across_equator);
    RUN_TEST(test_gps_distance_across_prime_meridian);
    
    // Bearing tests (8)
    RUN_TEST(test_gps_bearing_north);
    RUN_TEST(test_gps_bearing_east);
    RUN_TEST(test_gps_bearing_south);
    RUN_TEST(test_gps_bearing_west);
    RUN_TEST(test_gps_bearing_northeast);
    RUN_TEST(test_gps_bearing_southwest);
    RUN_TEST(test_gps_bearing_nyc_to_boston);
    RUN_TEST(test_gps_bearing_reverse);
    
    // Coordinate scaling tests (3)
    RUN_TEST(test_gps_coordinate_scaling);
    RUN_TEST(test_gps_negative_coordinates);
    RUN_TEST(test_gps_max_coordinates);
    
    // Edge case tests (4)
    RUN_TEST(test_gps_same_position);
    RUN_TEST(test_gps_very_short_distance);
    RUN_TEST(test_gps_bearing_range);
    RUN_TEST(test_gps_high_latitude);
    
    // Precision tests (3)
    RUN_TEST(test_gps_precision_1_meter);
    RUN_TEST(test_gps_precision_10_meters);
    RUN_TEST(test_gps_precision_100_meters);
    
    return UNITY_END();
}