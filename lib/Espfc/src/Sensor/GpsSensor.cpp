#include "Sensor/GpsSensor.hpp"
#include <GpsProtocol.hpp>
#include <Arduino.h>
#include <cmath>
#include <tuple>

namespace Espfc::Sensor
{

static constexpr std::array<int, 6> BAUDS{
  115200, 230400, 460800, 57600, 38400, 9600,
};

static constexpr std::array<std::tuple<uint16_t, uint8_t>, 2> UBX_MSG_ON{
  std::make_tuple(Gps::UBX_NAV_PVT,  1u),
  std::make_tuple(Gps::UBX_NAV_SAT, 10u),
};

GpsSensor::GpsSensor(Model& model): _model(model) {}

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

void GpsSensor::onMessage()
{
  if(_state == DETECT_BAUD)
  {
    _state = SET_BAUD;
    _model.logger.info().log(F("GPS DET")).logln(_currentBaud);
  }
}

void GpsSensor::handle()
{
  switch (_state)
  {
    case DETECT_BAUD:
      if(micros() > _timeout)
      {
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
        .mode = 0x08c0,
        .baudRate = (uint32_t)_targetBaud,
        .inProtoMask = 0x01,
        .outProtoMask = 0x01,
        .flags = 0,
        .resered2 = 0,
      }, GET_VERSION, GET_VERSION);
      delay(30);
      setBaud(_targetBaud);
      delay(5);
      break;

    case GET_VERSION:
      send(Gps::UbxMonVer{}, CONFIGURE_GNSS);
      _timeout = micros() + 3 * TIMEOUT;
      break;

    case CONFIGURE_GNSS:
      configureGnss();
      break;

    case ENABLE_UBX:
    {
      const Gps::UbxCfgMsg3 m{
        .msgId = std::get<0>(UBX_MSG_ON[_counter]),
        .rate = std::get<1>(UBX_MSG_ON[_counter]),
      };
      _counter++;
      if (_counter < UBX_MSG_ON.size())
      {
        send(m, _state);
      }
      else
      {
        send(m, ENABLE_NAV5);
        _counter = 0;
        _timeout = micros() + 10 * TIMEOUT;
        _model.logger.info().logln(F("GPS UBX"));
      }
    }
    break;

    case ENABLE_NAV5:
      send(Gps::UbxCfgNav5{
        .mask = { .value = 0xffff },
        .dynModel = 8,
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
    {
      uint16_t mRate;
      if (_model.state.gps.support.version == GPS_M10)
      {
        mRate = _currentBaud >= 230400 ? 40 : 100;
      }
      else
      {
        mRate = _currentBaud >= 115200 ? 100 : 200;
      }
      
      const uint16_t nRate = 1;
      const Gps::UbxCfgRate6 m{
        .measRate = mRate,
        .navRate = nRate,
        .timeRef = 0,
      };
      send(m, RECEIVE);
      _model.logger.info().log(F("GPS RATE")).log(mRate).log('/').logln(nRate);
    }
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
        else if (_ubxMsg.isResponse(Gps::UbxNavSat::ID))
        {
          handleNavSat();
        }
      }
      else if (_state == WAIT && micros() > _timeout)
      {
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

void GpsSensor::configureGnss()
{
  const auto version = _model.state.gps.support.version;
  const bool useDualBand = (_model.config.gps.enableDualBand && version == GPS_M10);
  
  // Determine which constellations to enable based on gnssMode
  bool enableGPS = _model.config.gps.enableGPS;
  bool enableGLO = _model.config.gps.enableGLONASS;
  bool enableGAL = _model.config.gps.enableGalileo;
  bool enableBDS = _model.config.gps.enableBeiDou;
  bool enableQZSS = _model.config.gps.enableQZSS;
  bool enableSBAS = _model.config.gps.enableSBAS;
  
  // Apply gnssMode preset if not Auto
  switch (_model.config.gps.gnssMode)
  {
    case 1: // GPS only
      enableGPS = true;
      enableGLO = enableGAL = enableBDS = enableQZSS = false;
      break;
    case 2: // GPS + GLONASS
      enableGPS = enableGLO = true;
      enableGAL = enableBDS = enableQZSS = false;
      break;
    case 3: // GPS + Galileo
      enableGPS = enableGAL = true;
      enableGLO = enableBDS = enableQZSS = false;
      break;
    case 4: // GPS + BeiDou
      enableGPS = enableBDS = true;
      enableGLO = enableGAL = enableQZSS = false;
      break;
    case 5: // All constellations
      enableGPS = enableGLO = enableGAL = enableBDS = enableQZSS = true;
      break;
    // case 0: Auto - use individual enable flags (default)
  }
  
  if (version == GPS_M10 || version == GPS_M8 || version == GPS_M9 || version == GPS_F9)
  {
    // Build GNSS configuration based on user settings
    uint8_t gnssConfig[] = {
      0xB5, 0x62,       // Header
      0x06, 0x3E,       // CFG-GNSS
      0x3C, 0x00,       // Length: 60 bytes
      0x00,             // msgVer
      0x00,             // numTrkChHw
      0xFF,             // numTrkChUse
      0x07,             // numConfigBlocks
      // GPS: L1C/A or L1+L5
      0x00, 0x08, 0x10, 0x00,
      (uint8_t)(enableGPS ? 0x01 : 0x00), 0x00,
      (uint8_t)(useDualBand ? 0x03 : 0x01), 0x01,
      // SBAS: L1C/A
      0x01, 0x01, 0x03, 0x00,
      (uint8_t)(enableSBAS ? 0x01 : 0x00), 0x00, 0x01, 0x01,
      // Galileo: E1 or E1+E5a
      0x02, 0x04, 0x08, 0x00,
      (uint8_t)(enableGAL ? 0x01 : 0x00), 0x00, 0x01, 0x01,
      // BeiDou: B1I or B1I+B2a
      0x03, 0x08, 0x10, 0x00,
      (uint8_t)(enableBDS ? 0x01 : 0x00), 0x00,
      (uint8_t)(useDualBand ? 0x03 : 0x01), 0x01,
      // IMES: disabled
      0x04, 0x00, 0x00, 0x00, 0x05, 0x00, 0x01, 0x01,
      // QZSS: L1C/A or L1+L5
      0x05, 0x00, 0x03, 0x00,
      (uint8_t)(enableQZSS ? 0x01 : 0x00), 0x00, 0x01, 0x01,
      // GLONASS: L1
      0x06, 0x08, 0x0E, 0x00,
      (uint8_t)(enableGLO ? 0x01 : 0x00), 0x00, 0x01, 0x01,
      0x00, 0x00        // Checksum (will be calculated)
    };
    
    // Calculate checksum
    uint8_t ckA = 0, ckB = 0;
    for (size_t i = 2; i < sizeof(gnssConfig) - 2; i++)
    {
      ckA += gnssConfig[i];
      ckB += ckA;
    }
    gnssConfig[sizeof(gnssConfig) - 2] = ckA;
    gnssConfig[sizeof(gnssConfig) - 1] = ckB;
    
    _port->write(gnssConfig, sizeof(gnssConfig));
    
    // Log configuration
    _model.logger.info().log(F("GPS GNSS "));
    if (useDualBand) {
      _model.logger.info().log(F("L1+L5 "));
    }
    _model.logger.info().log(F("["));
    if (enableGPS) _model.logger.info().log(F("GPS "));
    if (enableGLO) _model.logger.info().log(F("GLO "));
    if (enableGAL) _model.logger.info().log(F("GAL "));
    if (enableBDS) _model.logger.info().log(F("BDS "));
    if (enableQZSS) _model.logger.info().log(F("QZSS "));
    if (enableSBAS) _model.logger.info().log(F("SBAS"));
    _model.logger.info().logln(F("]"));
    
    setState(WAIT, ENABLE_UBX, ENABLE_UBX);
  }
  else
  {
    setState(ENABLE_UBX);
  }
}

void GpsSensor::calculateHomeVector() const
{
  if (!_model.state.gps.homeSet || !_model.state.gps.fix) {
    _model.state.gps.distanceToHome = 0;
    _model.state.gps.directionToHome = 0;
    return;
  }
  
  _model.state.gps.distanceToHome = haversineDistance(
    _model.state.gps.home.raw.lat,
    _model.state.gps.home.raw.lon,
    _model.state.gps.location.raw.lat,
    _model.state.gps.location.raw.lon
  );
  
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
  const double R = 6371000.0;
  
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

int16_t GpsSensor::calculateBearing(int32_t lat1, int32_t lon1,
                                     int32_t lat2, int32_t lon2)
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

void GpsSensor::setHomePosition() const
{
  if (!_model.state.gps.fix || _model.state.gps.fixType < 2) {
    return;
  }
  
  if (_model.state.gps.numSats < _model.config.gps.minSats) {
    return;
  }
  
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
  if (_model.state.gps.homeSet && _model.config.gps.setHomeOnce) {
    return false;
  }
  
  if (!_model.state.gps.fix || _model.state.gps.fixType < 2) {
    return false;
  }
  
  if (_model.state.gps.numSats < _model.config.gps.minSats) {
    return false;
  }
  
  if (_model.state.gps.accuracy.horizontal > 5000) {
    return false;
  }
  
  return true;
}

void GpsSensor::handleNavPvt() const
{
  const auto &m = *_ubxMsg.getAs<Gps::UbxNavPvt92>();

  _model.state.gps.fix = m.fixType == 3 && m.flags.gnssFixOk;
  _model.state.gps.fixType = m.fixType;
  _model.state.gps.numSats = m.numSV;

  _model.state.gps.accuracy.pDop = m.pDOP;
  _model.state.gps.accuracy.horizontal = m.hAcc;
  _model.state.gps.accuracy.vertical = m.vAcc;
  _model.state.gps.accuracy.speed = m.sAcc;
  _model.state.gps.accuracy.heading = m.headAcc;

  _model.state.gps.location.raw.lat = m.lat;
  _model.state.gps.location.raw.lon = m.lon;
  _model.state.gps.location.raw.height = m.hSML;

  _model.state.gps.velocity.raw.groundSpeed = m.gSpeed;
  _model.state.gps.velocity.raw.heading = m.headMot;

  _model.state.gps.velocity.raw.north = m.velN;
  _model.state.gps.velocity.raw.east  = m.velE;
  _model.state.gps.velocity.raw.down  = m.velD;
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

  if (shouldSetHome()) {
    if (_model.isModeActive(MODE_ARMED) && _model.config.gps.autoSetHome) {
      setHomePosition();
    }
  }
  
  calculateHomeVector();
}

void GpsSensor::handleNavSat() const
{
  const auto &m = *_ubxMsg.getAs<Gps::UbxNavSat>();
  _model.state.gps.numCh = m.numSvs;
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

void GpsSensor::handleVersion() const
{
  const char *payload = (const char *)_ubxMsg.payload;

  _model.logger.info().log(F("GPS VER")).logln(payload);
  _model.logger.info().log(F("GPS VER")).logln(payload + 30);

  if (std::strcmp(payload + 30, "00080000") == 0)
  {
    _model.state.gps.support.version = GPS_M8;
  }
  else if (std::strcmp(payload + 30, "00090000") == 0)
  {
    _model.state.gps.support.version = GPS_M9;
  }
  else if (std::strcmp(payload + 30, "00190000") == 0)
  {
    _model.state.gps.support.version = GPS_F9;
  }
  else if (std::strcmp(payload + 30, "000A0000") == 0)
  {
    _model.state.gps.support.version = GPS_M10;
    _model.state.gps.support.dualBand = true;
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
  if (std::strstr(payload, "QZSS") != nullptr)
  {
    _model.state.gps.support.qzss = true;
  }
}

}