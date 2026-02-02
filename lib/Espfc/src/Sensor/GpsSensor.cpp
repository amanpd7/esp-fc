#include "Sensor/GpsSensor.hpp"
#include <GpsProtocol.hpp>
#include <Arduino.h>
#include <cmath>
#include <tuple>

namespace Espfc::Sensor
{

static constexpr std::array<int, 6> BAUDS{
  9600, 115200, 230400, 57600, 38400, 19200,
};

static constexpr std::array<uint16_t, 6> NMEA_MSG_OFF{
  Gps::NMEA_MSG_GGA, Gps::NMEA_MSG_GLL, Gps::NMEA_MSG_GSA,
  Gps::NMEA_MSG_GSV, Gps::NMEA_MSG_RMC, Gps::NMEA_MSG_VTG,
};

// M8/M9 messages (NAV-PVT supported)
static constexpr std::array<std::tuple<uint16_t, uint8_t>, 2> UBX_MSG_M8{
  std::make_tuple(Gps::UBX_NAV_PVT,  1u),
  std::make_tuple(Gps::UBX_NAV_SAT, 10u),
};

// M6/M7 messages (legacy, no NAV-PVT)
static constexpr std::array<std::tuple<uint16_t, uint8_t>, 3> UBX_MSG_M6{
  std::make_tuple(0x0102, 1u),  // NAV-POSLLH
  std::make_tuple(0x0112, 1u),  // NAV-VELNED
  std::make_tuple(0x0135, 10u), // NAV-SAT
};

GpsSensor::GpsSensor(Model& model): _model(model), _useLegacyMessages(false) {}

int GpsSensor::begin(Device::SerialDevice* port, int baud)
{
  _port = port;
  _targetBaud = _currentBaud = baud;
  _timer.setRate(50);

  _state = DETECT_BAUD;
  _timeout = micros() + DETECT_TIMEOUT;
  _counter = 0;
  setBaud(BAUDS[_counter]);

  return 1;
}

int GpsSensor::update()
{
  if(!_port) return 0;

  if(!_timer.check()) return 0;

  Utils::Stats::Measure measure(_model.state.stats, COUNTER_GPS_READ);

  bool updated = false;
  uint8_t buff[32];
  size_t read = 0;
  while ((read = _port->readMany(buff, sizeof(buff))))
  {
    for (size_t i = 0; i < read; i++)
    {
      updated |= processUbx(buff[i]);
      processNmea(buff[i]);
    }
  }

  if (!updated) handle();

  return 1;
}

bool GpsSensor::processUbx(uint8_t c)
{
  _ubxParser.parse(c, _ubxMsg);
  if (!_ubxMsg.isReady()) return false;

  onMessage();

  handle();
  _ubxMsg = Gps::UbxMessage();

  return true;
}

void GpsSensor::processNmea(uint8_t c)
{
  _nmeaParser.parse(c, _nmeaMsg);
  if (!_nmeaMsg.isReady()) return;

  //$GNTXT,01,01,01,More than 100 frame errors, UART RX was disabled*70
  static const char * msg = "GNTXT,01,01,01,More than 100 frame errors";

  if(!_model.state.gps.frameError && std::strncmp(_nmeaMsg.payload, msg, std::strlen(msg)) == 0)
  {
    _model.state.gps.frameError = true;
    if(!_model.isModeActive(MODE_ARMED)) _model.logger.err().logln("GPS RX Frame Err");
  }

  // Parse GGA sentences for position/satellites
  if (std::strncmp(_nmeaMsg.payload, "GPGGA,", 6) == 0 || 
      std::strncmp(_nmeaMsg.payload, "GNGGA,", 6) == 0)
  {
    handleNmeaGGA();
  }
  
  // Parse RMC sentences for speed/heading/time
  if (std::strncmp(_nmeaMsg.payload, "GPRMC,", 6) == 0 || 
      std::strncmp(_nmeaMsg.payload, "GNRMC,", 6) == 0)
  {
    handleNmeaRMC();
  }

  onMessage();

  _nmeaMsg = Gps::NmeaMessage();
}

void GpsSensor::onMessage()
{
  if(_state == DETECT_BAUD)
  {
    _state = SET_BAUD;
    _model.logger.info().log("GPS DET").logln(_currentBaud);
  }
}

void GpsSensor::handle()
{
  switch (_state)
  {
    case DETECT_BAUD:
      if(micros() > _timeout)
      {
        // on timeout check next baud
        _counter++;
        if(_counter < BAUDS.size())
        {
          setBaud(BAUDS[_counter]);
        }
        else
        {
          _state = SET_BAUD;
          _counter = 0;
        }
        _timeout = micros() + DETECT_TIMEOUT;
      }
      break;

    case SET_BAUD:
      send(Gps::UbxCfgPrt20{
        .portId = 1,
        .resered1 = 0,
        .txReady = 0,
        .mode = 0x08c0,     // 8N1
        .baudRate = (uint32_t)_targetBaud, // baud
        .inProtoMask = 0x03,  // UBX + NMEA (0x01 | 0x02)
        .outProtoMask = 0x03, // UBX + NMEA (0x01 | 0x02)
        .flags = 0,
        .resered2 = 0,
      }, GET_VERSION, GET_VERSION);  // Skip NMEA disable, go straight to version
      delay(30); // wait until transmission complete at 9600bps
      setBaud(_targetBaud);
      delay(5);
      _model.logger.info().logln(F("GPS UBX+NMEA MODE"));
      break;

    case DISABLE_NMEA:
      // Skip disabling NMEA - we want to keep it for parsing
      _counter = 0;
      _state = GET_VERSION;
      _model.logger.info().logln(F("GPS NMEA KEEP"));
      break;

    case GET_VERSION:
      send(Gps::UbxMonVer{}, ENABLE_UBX);
      _timeout = micros() + 3 * TIMEOUT;
      break;

    case ENABLE_UBX:
    {
      // Determine which message set to use based on GPS version
      const bool useM6 = _useLegacyMessages;
      const size_t msgCount = useM6 ? UBX_MSG_M6.size() : UBX_MSG_M8.size();
      
      const Gps::UbxCfgMsg3 m{
        .msgId = useM6 ? std::get<0>(UBX_MSG_M6[_counter]) : std::get<0>(UBX_MSG_M8[_counter]),
        .rate = useM6 ? std::get<1>(UBX_MSG_M6[_counter]) : std::get<1>(UBX_MSG_M8[_counter]),
      };
      
      _counter++;
      if (_counter < msgCount)
      {
        send(m, _state, _state);
      }
      else
      {
        send(m, ENABLE_NAV5, ENABLE_NAV5);
        _counter = 0;
        _timeout = micros() + 10 * TIMEOUT;
        _model.logger.info().logln(F("GPS UBX"));
      }
    }
    break;

    case ENABLE_NAV5:
      send(Gps::UbxCfgNav5{
        .mask = { .value = 0xffff }, // all
        .dynModel = 8, // airborne
        .fixMode = 3,
        .fixedAlt = 0,
        .fixedAltVar = 10000,
        .minElev = 5,
        .drLimit = 0,
        .pDOP = 250,
        .tDOP = 250,
        .pAcc = 100,
        .tAcc = 300,
        .staticHoldThresh = 0,
        .dgnssTimeout = 60,
        .cnoThreshNumSVs = 0,
        .cnoThresh = 0,
        .reserved0 = {0, 0},
        .staticHoldMaxDist = 200,
        .utcStandard = 0,
        .reserved1 = {0, 0, 0, 0, 0},
      }, ENABLE_SBAS, ENABLE_SBAS);
      _model.logger.info().logln(F("GPS NAV5"));
      break;

    case ENABLE_SBAS:
      if (_model.state.gps.support.sbas)
      {
        send(Gps::UbxCfgSbas8{
          .mode = 1,
          .usage = 1,
          .maxSbas = 3,
          .scanmode2 = 0,
          .scanmode1 = 0,
        }, SET_RATE, SET_RATE);
      }
      else
      {
        setState(SET_RATE);
      }
      _model.logger.info().log(F("GPS SBAS")).logln(_model.state.gps.support.sbas);
      break;

    case SET_RATE:
      // Skip rate configuration, go straight to receiving
      _state = RECEIVE;
      _model.state.gps.present = true;
      _model.logger.info().logln(F("GPS START RX"));
      break;

    case ERROR:
      if (_counter == 0)
      {
        _model.logger.err().logln(F("GPS ERROR"));
        _counter++;
      }
      handleError();
      break;

    case RECEIVE:
      _model.state.gps.present = true;
    case WAIT:
    default:
      if (_ubxMsg.isReady())
      {
        if (_ubxMsg.isAck())
        {
          _state = _ackState;
        }
        else if (_ubxMsg.isNak())
        {
          _state = _timeoutState;
        }
        else if (_ubxMsg.isResponse(Gps::UbxMonVer::ID))
        {
          handleVersion();
          _state = _ackState;
          _counter = 0;
        }
        else if (_ubxMsg.isResponse(Gps::UbxNavPvt92::ID))
        {
          handleNavPvt();
        }
        else if (_ubxMsg.isResponse(static_cast<Gps::MsgId>(0x0102))) // NAV-POSLLH (M6/M7)
        {
          handleNavPosllh();
        }
        else if (_ubxMsg.isResponse(static_cast<Gps::MsgId>(0x0112))) // NAV-VELNED (M6/M7)
        {
          handleNavVelned();
        }
        else if (_ubxMsg.isResponse(Gps::UbxNavSat::ID))
        {
          handleNavSat();
        }
        else
        {
        }
      }
      else if (_state == WAIT && micros() > _timeout)
      {
        // timeout
        _state = _timeoutState;
        _model.state.gps.present = false;
      }
      break;
  }
}

void GpsSensor::setBaud(int baud)
{
  if(baud != _currentBaud)
  {
    _port->updateBaudRate(baud);
    _currentBaud = baud;
    _model.logger.info().log(F("GPS BAUD")).logln(baud);
  }
}

void GpsSensor::setState(State state, State ackState, State timeoutState)
{
  setState(state);
  _ackState = ackState;
  _timeoutState = timeoutState;
}

void GpsSensor::setState(State state)
{
  _state = state;
  _timeout = micros() + TIMEOUT;
}

void GpsSensor::handleError() const
{
  _model.state.gps.present = false;
}

void GpsSensor::calculateHomeVector() const
{
  if (!_model.state.gps.homeSet || !_model.state.gps.fix) {
    _model.state.gps.distanceToHome = 0;
    _model.state.gps.directionToHome = 0;
    return;
  }
  
  // Calculate distance using Haversine formula
  _model.state.gps.distanceToHome = haversineDistance(
    _model.state.gps.home.raw.lat,
    _model.state.gps.home.raw.lon,
    _model.state.gps.location.raw.lat,
    _model.state.gps.location.raw.lon
  );
  
  // Calculate bearing
  _model.state.gps.directionToHome = calculateBearing(
    _model.state.gps.location.raw.lat,
    _model.state.gps.location.raw.lon,
    _model.state.gps.home.raw.lat,
    _model.state.gps.home.raw.lon
  );
}

float GpsSensor::haversineDistance(int32_t lat1, int32_t lon1, 
                                    int32_t lat2, int32_t lon2)
{
  // Convert to radians (coordinates are in degrees * 1e7)
  const double R = 6371000.0; // Earth radius in meters
  
  double phi1 = lat1 * 1e-7 * M_PI / 180.0;
  double phi2 = lat2 * 1e-7 * M_PI / 180.0;
  double deltaPhi = (lat2 - lat1) * 1e-7 * M_PI / 180.0;
  double deltaLambda = (lon2 - lon1) * 1e-7 * M_PI / 180.0;
  
  double a = sin(deltaPhi / 2.0) * sin(deltaPhi / 2.0) +
             cos(phi1) * cos(phi2) *
             sin(deltaLambda / 2.0) * sin(deltaLambda / 2.0);
  
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  
  return R * c; // Distance in meters
}

int16_t GpsSensor::calculateBearing(int32_t lat1, int32_t lon1,
                                     int32_t lat2, int32_t lon2)
{
  // Convert to radians
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

void GpsSensor::setHomePosition() const
{
  if (!_model.state.gps.fix || _model.state.gps.fixType < 2) {
    return; // Need at least 2D fix
  }
  
  if (_model.state.gps.numSats < _model.config.gps.minSats) {
    return; // Not enough satellites
  }
  
  // Set home position
  _model.state.gps.home.raw.lat = _model.state.gps.location.raw.lat;
  _model.state.gps.home.raw.lon = _model.state.gps.location.raw.lon;
  _model.state.gps.home.raw.height = _model.state.gps.location.raw.height;
  _model.state.gps.homeSet = true;
  
  _model.logger.info()
    .log(F("GPS HOME SET: "))
    .log(_model.state.gps.home.raw.lat * 1e-7f)
    .log(F(", "))
    .logln(_model.state.gps.home.raw.lon * 1e-7f);
}

bool GpsSensor::shouldSetHome() const
{
  // Don't set if already set and setHomeOnce is enabled
  if (_model.state.gps.homeSet && _model.config.gps.setHomeOnce) {
    return false;
  }
  
  // Check if we have good GPS fix
  if (!_model.state.gps.fix || _model.state.gps.fixType < 2) {
    return false;
  }
  
  // Check satellite count
  if (_model.state.gps.numSats < _model.config.gps.minSats) {
    return false;
  }
  
  // Check horizontal accuracy (< 5 meters)
  if (_model.state.gps.accuracy.horizontal > 5000) {
    return false;
  }
  
  return true;
}

// Parse NMEA GGA sentence for position and satellites
void GpsSensor::handleNmeaGGA()
{
  // $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
  const char* p = _nmeaMsg.payload;
  
  // Skip to first comma (past GPGGA)
  while (*p && *p != ',') p++;
  if (!*p) return;
  p++; // skip comma
  
  // Skip time
  while (*p && *p != ',') p++;
  if (!*p) return;
  p++;
  
  // Parse latitude
  char latStr[12] = {0};
  int i = 0;
  while (*p && *p != ',' && i < 11) latStr[i++] = *p++;
  if (!*p) return;
  p++; // skip comma
  
  char latDir = *p;
  p += 2; // skip N/S and comma
  
  // Parse longitude  
  char lonStr[12] = {0};
  i = 0;
  while (*p && *p != ',' && i < 11) lonStr[i++] = *p++;
  if (!*p) return;
  p++;
  
  char lonDir = *p;
  p += 2; // skip E/W and comma
  
  // Parse fix quality
  int fixQuality = *p - '0';
  p += 2; // skip fix and comma
  
  // Parse satellite count
  char satStr[4] = {0};
  i = 0;
  while (*p && *p != ',' && i < 3) satStr[i++] = *p++;
  int numSats = atoi(satStr);
  if (*p) p++; // skip comma
  
  // Skip HDOP
  while (*p && *p != ',') p++;
  if (*p) p++; // skip comma
  
  // Parse altitude
  char altStr[12] = {0};
  i = 0;
  while (*p && *p != ',' && i < 11) altStr[i++] = *p++;
  float altitude = atof(altStr);
  
  // Convert lat/lon from NMEA format (DDMM.MMMM) to degrees * 1e7
  float latDeg = atof(latStr);
  int latDegInt = (int)(latDeg / 100);
  float latMin = latDeg - (latDegInt * 100);
  float lat = latDegInt + (latMin / 60.0);
  if (latDir == 'S') lat = -lat;
  
  float lonDeg = atof(lonStr);
  int lonDegInt = (int)(lonDeg / 100);
  float lonMin = lonDeg - (lonDegInt * 100);
  float lon = lonDegInt + (lonMin / 60.0);
  if (lonDir == 'W') lon = -lon;
  
  // Update GPS state
  _model.state.gps.location.raw.lat = (int32_t)(lat * 1e7);
  _model.state.gps.location.raw.lon = (int32_t)(lon * 1e7);
  _model.state.gps.location.raw.height = (int32_t)(altitude * 1000); // convert m to mm
  _model.state.gps.numSats = numSats;
  _model.state.gps.fix = (fixQuality > 0);
  _model.state.gps.fixType = (fixQuality > 0) ? 3 : 0;
  
  uint32_t now = micros();
  _model.state.gps.interval = now - _model.state.gps.lastMsgTs;
  _model.state.gps.lastMsgTs = now;
  
  // Auto-set home
  if (shouldSetHome()) {
    if (_model.isModeActive(MODE_ARMED) && _model.config.gps.autoSetHome) {
      setHomePosition();
    }
  }
  
  calculateHomeVector();
}

// Parse NMEA RMC sentence for speed, heading, and date/time
void GpsSensor::handleNmeaRMC()
{
  // $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
  const char* p = _nmeaMsg.payload;
  
  // Skip to first comma (past GPRMC)
  while (*p && *p != ',') p++;
  if (!*p) return;
  p++; // skip comma
  
  // Parse time (HHMMSS.ss)
  char timeStr[12] = {0};
  int i = 0;
  while (*p && *p != ',' && i < 11) timeStr[i++] = *p++;
  if (*p) p++; // skip comma
  
  // Skip status
  p += 2; // skip status and comma
  
  // Skip lat
  while (*p && *p != ',') p++;
  if (*p) p++; // skip comma
  p += 2; // skip N/S and comma
  
  // Skip lon
  while (*p && *p != ',') p++;
  if (*p) p++; // skip comma
  p += 2; // skip E/W and comma
  
  // Parse speed (knots)
  char speedStr[12] = {0};
  i = 0;
  while (*p && *p != ',' && i < 11) speedStr[i++] = *p++;
  float speedKnots = atof(speedStr);
  if (*p) p++; // skip comma
  
  // Parse heading (degrees)
  char headingStr[12] = {0};
  i = 0;
  while (*p && *p != ',' && i < 11) headingStr[i++] = *p++;
  float heading = atof(headingStr);
  if (*p) p++; // skip comma
  
  // Parse date (DDMMYY)
  char dateStr[8] = {0};
  i = 0;
  while (*p && *p != ',' && i < 7) dateStr[i++] = *p++;
  
  // Convert speed from knots to mm/s (1 knot = 0.514444 m/s)
  float speedMs = speedKnots * 0.514444f;
  _model.state.gps.velocity.raw.groundSpeed = (int32_t)(speedMs * 1000); // m/s to mm/s
  _model.state.gps.velocity.raw.heading = (int32_t)(heading * 1e5); // deg to deg * 1e5
  
  // Parse time
  if (strlen(timeStr) >= 6) {
    int hour = (timeStr[0] - '0') * 10 + (timeStr[1] - '0');
    int min = (timeStr[2] - '0') * 10 + (timeStr[3] - '0');
    int sec = (timeStr[4] - '0') * 10 + (timeStr[5] - '0');
    
    _model.state.gps.dateTime.hour = hour;
    _model.state.gps.dateTime.minute = min;
    _model.state.gps.dateTime.second = sec;
  }
  
  // Parse date
  if (strlen(dateStr) >= 6) {
    int day = (dateStr[0] - '0') * 10 + (dateStr[1] - '0');
    int month = (dateStr[2] - '0') * 10 + (dateStr[3] - '0');
    int year = 2000 + (dateStr[4] - '0') * 10 + (dateStr[5] - '0');
    
    _model.state.gps.dateTime.day = day;
    _model.state.gps.dateTime.month = month;
    _model.state.gps.dateTime.year = year;
  }
}

// M8/M9 NAV-PVT handler
void GpsSensor::handleNavPvt() const
{
  const auto &m = *_ubxMsg.getAs<Gps::UbxNavPvt92>();

  _model.state.gps.fix = m.fixType >= 2 && m.flags.gnssFixOk;
  _model.state.gps.fixType = m.fixType;
  _model.state.gps.numSats = m.numSV;

  _model.state.gps.accuracy.pDop = m.pDOP;
  _model.state.gps.accuracy.horizontal = m.hAcc; // mm
  _model.state.gps.accuracy.vertical = m.vAcc; // mm
  _model.state.gps.accuracy.speed = m.sAcc; // mm/s
  _model.state.gps.accuracy.heading = m.headAcc; // deg * 1e5

  _model.state.gps.location.raw.lat = m.lat;
  _model.state.gps.location.raw.lon = m.lon;
  _model.state.gps.location.raw.height = m.hSML; // mm

  _model.state.gps.velocity.raw.groundSpeed = m.gSpeed; // mm/s
  _model.state.gps.velocity.raw.heading = m.headMot; // deg * 1e5

  _model.state.gps.velocity.raw.north = m.velN; // mm/s
  _model.state.gps.velocity.raw.east  = m.velE; // mm/s
  _model.state.gps.velocity.raw.down  = m.velD; // mm/s
  _model.state.gps.velocity.raw.speed3d = lrintf(std::sqrt(
    _model.state.gps.velocity.raw.groundSpeed * _model.state.gps.velocity.raw.groundSpeed +
    _model.state.gps.velocity.raw.down * _model.state.gps.velocity.raw.down
  ));

  if(m.valid.validDate && m.valid.validTime)
  {
    _model.state.gps.dateTime.year = m.year;
    _model.state.gps.dateTime.month = m.month;
    _model.state.gps.dateTime.day = m.day;
    _model.state.gps.dateTime.hour = m.hour;
    _model.state.gps.dateTime.minute = m.min;
    _model.state.gps.dateTime.second = m.sec;
    int32_t msec = m.nano / 1000000;
    if(msec < 0) {
      msec += 1000;
    }
    _model.state.gps.dateTime.msec = msec;
  }

  uint32_t now = micros();
  _model.state.gps.interval = now - _model.state.gps.lastMsgTs;
  _model.state.gps.lastMsgTs = now;

  // Auto-set home position
  if (shouldSetHome()) {
    if (_model.isModeActive(MODE_ARMED) && _model.config.gps.autoSetHome) {
      setHomePosition();
    }
  }
  
  // Calculate distance and bearing to home
  calculateHomeVector();
}

// M6/M7 NAV-POSLLH handler
void GpsSensor::handleNavPosllh() const
{
  // NAV-POSLLH structure (28 bytes payload)
  struct __attribute__((packed)) UbxNavPosllh {
    uint32_t iTOW;
    int32_t lon;      // deg * 1e7
    int32_t lat;      // deg * 1e7
    int32_t height;   // mm
    int32_t hMSL;     // mm
    uint32_t hAcc;    // mm
    uint32_t vAcc;    // mm
  };
  
  const auto &m = *reinterpret_cast<const UbxNavPosllh*>(_ubxMsg.payload);
  
  _model.state.gps.location.raw.lat = m.lat;
  _model.state.gps.location.raw.lon = m.lon;
  _model.state.gps.location.raw.height = m.hMSL;
  
  _model.state.gps.accuracy.horizontal = m.hAcc;
  _model.state.gps.accuracy.vertical = m.vAcc;
  
  // Assume fix if we're getting position data (will be confirmed by NAV-SAT)
  _model.state.gps.fix = true;
  _model.state.gps.fixType = 3;
  
  uint32_t now = micros();
  _model.state.gps.interval = now - _model.state.gps.lastMsgTs;
  _model.state.gps.lastMsgTs = now;
  
  // Auto-set home position
  if (shouldSetHome()) {
    if (_model.isModeActive(MODE_ARMED) && _model.config.gps.autoSetHome) {
      setHomePosition();
    }
  }
  
  // Calculate distance and bearing to home
  calculateHomeVector();
}

// M6/M7 NAV-VELNED handler
void GpsSensor::handleNavVelned() const
{
  // NAV-VELNED structure (36 bytes payload)
  struct __attribute__((packed)) UbxNavVelned {
    uint32_t iTOW;
    int32_t velN;      // cm/s
    int32_t velE;      // cm/s
    int32_t velD;      // cm/s
    uint32_t speed;    // cm/s
    uint32_t gSpeed;   // cm/s
    int32_t heading;   // deg * 1e5
    uint32_t sAcc;     // cm/s
    uint32_t cAcc;     // deg * 1e5
  };
  
  const auto &m = *reinterpret_cast<const UbxNavVelned*>(_ubxMsg.payload);
  
  // Convert cm/s to mm/s (multiply by 10)
  _model.state.gps.velocity.raw.north = m.velN * 10;
  _model.state.gps.velocity.raw.east = m.velE * 10;
  _model.state.gps.velocity.raw.down = m.velD * 10;
  _model.state.gps.velocity.raw.groundSpeed = m.gSpeed * 10;
  _model.state.gps.velocity.raw.heading = m.heading;
  _model.state.gps.velocity.raw.speed3d = m.speed * 10;
  
  _model.state.gps.accuracy.speed = m.sAcc * 10;
  _model.state.gps.accuracy.heading = m.cAcc;
}

void GpsSensor::handleNavSat() const
{
  const auto &m = *_ubxMsg.getAs<Gps::UbxNavSat>();
  _model.state.gps.numCh = m.numSvs;
  _model.state.gps.numSats = m.numSvs; // Update sat count from NAV-SAT
  
  for (uint8_t i = 0; i < SAT_MAX; i++)
  {
    if(i < m.numSvs)
    {
      _model.state.gps.svinfo[i].id = m.sats[i].svId;
      _model.state.gps.svinfo[i].gnssId = m.sats[i].gnssId;
      _model.state.gps.svinfo[i].cno = m.sats[i].cno;
      _model.state.gps.svinfo[i].quality.value = m.sats[i].flags.value;
    }
    else
    {
      _model.state.gps.svinfo[i] = GpsSatelite{};
    }
  }
}

void GpsSensor::handleVersion()
{
  const char *payload = (const char *)_ubxMsg.payload;

  _model.logger.info().log(F("GPS VER")).logln(payload);
  _model.logger.info().log(F("GPS VER")).logln(payload + 30);

  // Detect GPS version and set message format
  if (std::strcmp(payload + 30, "00040007") == 0 || 
      std::strcmp(payload + 30, "00070000") == 0)
  {
    // Neo-6M (00040007) or Neo-7M (00070000) - use legacy messages
    _model.state.gps.support.version = GPS_UNKNOWN;
    _useLegacyMessages = true;
    _model.logger.info().logln(F("GPS M6/M7 MODE"));
  }
  else if (std::strcmp(payload + 30, "00080000") == 0)
  {
    _model.state.gps.support.version = GPS_M8;
    _useLegacyMessages = false;
    _model.logger.info().logln(F("GPS M8 MODE"));
  }
  else if (std::strcmp(payload + 30, "00090000") == 0)
  {
    _model.state.gps.support.version = GPS_M9;
    _useLegacyMessages = false;
    _model.logger.info().logln(F("GPS M9 MODE"));
  }
  else if (std::strcmp(payload + 30, "00190000") == 0)
  {
    _model.state.gps.support.version = GPS_F9;
    _useLegacyMessages = false;
    _model.logger.info().logln(F("GPS F9 MODE"));
  }
  else
  {
    // Unknown GPS, try M8+ protocol first
    _useLegacyMessages = false;
    _model.logger.info().logln(F("GPS UNKNOWN, TRY M8+"));
  }
  
  if (_ubxMsg.length >= 70)
  {
    checkSupport(payload + 40);
    _model.logger.info().log(F("GPS EXT")).logln(payload + 40);
  }
  if (_ubxMsg.length >= 100)
  {
    checkSupport(payload + 70);
    _model.logger.info().log(F("GPS EXT")).logln(payload + 70);
  }
  if (_ubxMsg.length >= 130)
  {
    checkSupport(payload + 100);
    _model.logger.info().log(F("GPS EXT")).logln(payload + 100);
  }
  if (_ubxMsg.length >= 160)
  {
    checkSupport(payload + 130);
    _model.logger.info().log(F("GPS EXT")).logln(payload + 130);
  }
}

void GpsSensor::checkSupport(const char *payload) const
{
  if (std::strstr(payload, "SBAS") != nullptr)
  {
    _model.state.gps.support.sbas = true;
  }
  if (std::strstr(payload, "GLO") != nullptr)
  {
    _model.state.gps.support.glonass = true;
  }
  if (std::strstr(payload, "GAL") != nullptr)
  {
    _model.state.gps.support.galileo = true;
  }
  if (std::strstr(payload, "BDS") != nullptr)
  {
    _model.state.gps.support.beidou = true;
  }
}

}