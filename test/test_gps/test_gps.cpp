// test/test_gps/test_gps.cpp
// Tests for GPS distance and bearing math used in GpsSensor::calculateHomeVector()

#include <unity.h>
#include <cmath>
#include <cstdint>

// Equirectangular approximation — matches GpsSensor::calculateHomeVector()
// Inputs: lat/lon in deg * 1e7 (raw GPS coordinate units)
static uint16_t gpsDistance(int32_t homeLat, int32_t homeLon,
                             int32_t curLat,  int32_t curLon)
{
  const float LAT_TO_M = 1.113e-2f; // deg*1e-7 → meters (111300 m/deg / 1e7)
  const float dlat = (curLat - homeLat) * LAT_TO_M;
  const float dlon = (curLon - homeLon) * LAT_TO_M
                     * cosf(homeLat * 1e-7f * (float)M_PI / 180.0f);
  return (uint16_t)sqrtf(dlat * dlat + dlon * dlon);
}

static int16_t gpsBearing(int32_t homeLat, int32_t homeLon,
                           int32_t curLat,  int32_t curLon)
{
  const float LAT_TO_M = 1.113e-2f;
  const float dlat = (curLat - homeLat) * LAT_TO_M;
  const float dlon = (curLon - homeLon) * LAT_TO_M
                     * cosf(homeLat * 1e-7f * (float)M_PI / 180.0f);
  float bearing = atan2f(dlon, dlat) * (180.0f / (float)M_PI);
  if (bearing < 0.0f) bearing += 360.0f;
  return (int16_t)bearing;
}

// ---------------------------------------------------------------------------

void test_distance_north()
{
  // 0.1 degree north from equator ≈ 11130 m (within uint16_t range)
  uint16_t d = gpsDistance(0, 0, 1000000, 0);
  TEST_ASSERT_INT_WITHIN(200, 11130, (int)d);
}

void test_distance_zero_at_same_position()
{
  uint16_t d = gpsDistance(377490000, -1224194000, 377490000, -1224194000);
  TEST_ASSERT_EQUAL_UINT16(0, d);
}

void test_bearing_north()
{
  int16_t b = gpsBearing(0, 0, 10000000, 0);
  TEST_ASSERT_INT_WITHIN(2, 0, (int)b);
}

void test_bearing_east()
{
  int16_t b = gpsBearing(0, 0, 0, 10000000);
  TEST_ASSERT_INT_WITHIN(2, 90, (int)b);
}

void test_bearing_south()
{
  int16_t b = gpsBearing(0, 0, -10000000, 0);
  TEST_ASSERT_TRUE(b == 180 || b == -180);
}

void test_bearing_west()
{
  int16_t b = gpsBearing(0, 0, 0, -10000000);
  TEST_ASSERT_INT_WITHIN(2, 270, (int)b);
}

void test_bearing_northeast()
{
  int16_t b = gpsBearing(0, 0, 10000000, 10000000);
  TEST_ASSERT_INT_WITHIN(5, 45, (int)b);
}

void test_bearing_wraps_360()
{
  // West is 270°, not -90°
  int16_t b = gpsBearing(0, 0, 0, -5000000);
  TEST_ASSERT_TRUE(b >= 0);
}

// ---------------------------------------------------------------------------

int main()
{
  UNITY_BEGIN();

  RUN_TEST(test_distance_north);
  RUN_TEST(test_distance_zero_at_same_position);
  RUN_TEST(test_bearing_north);
  RUN_TEST(test_bearing_east);
  RUN_TEST(test_bearing_south);
  RUN_TEST(test_bearing_west);
  RUN_TEST(test_bearing_northeast);
  RUN_TEST(test_bearing_wraps_360);

  return UNITY_END();
}
