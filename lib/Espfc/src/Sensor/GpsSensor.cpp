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

  onMessage();

  _nmeaMsg = Gps::NmeaMessage();
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
        .inProtoMask = 0x07,
        .outProtoMask = 0x07,
        .flags = 0,
        .resered2 = 0,
      }, DISABLE_NMEA, DISABLE_NMEA);
      delay(30); // wait until transmission complete at 9600bps
      setBaud(_targetBaud);
      delay(5);
      break;

    case DISABLE_NMEA:
    {
      const Gps::UbxCfgMsg3 m{
        .msgId = NMEA_MSG_OFF[_counter],
        .rate = 0,
      };
      _counter++;
      if (_counter < NMEA_MSG_OFF.size())
      {
        send(m, _state);
      }
      else
      {
        _counter = 0;
        send(m, GET_VERSION);
        _model.logger.info().log(F("GPS NMEA")).logln(0);
      }
    }
    break;

    case GET_VERSION:
      send(Gps::UbxMonVer{}, ENABLE_UBX); // version handled in WAIT/RECEIVE
      _timeout = micros() + 3 * TIMEOUT;
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
        }, CONFIGURE_GNSS, CONFIGURE_GNSS);
      }
      else
      {
        setState(CONFIGURE_GNSS);
      }
      _model.logger.info().log(F("GPS SBAS")).logln(_model.state.gps.support.sbas);
      break;

    case CONFIGURE_GNSS:
      configureGnss();
      break;

    case SET_RATE:
    {
      const uint16_t mRate = _currentBaud > 100000 ? 100 : 200;
      const uint16_t nRate = 1;
      const Gps::UbxCfgRate6 m{
        .measRate = mRate,
        .navRate = nRate,
        .timeRef = 0, // utc
      };
      send(m, RECEIVE);
      _model.logger.info().log(F("GPS RATE")).log(mRate).logln(nRate);
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

void GpsSensor::configureGnss()
{
  const auto version = _model.state.gps.support.version;
  const bool useDualBand = (_model.config.gps.enableDualBand && version == GPS_M10);

  bool enableGPS  = _model.config.gps.enableGPS;
  bool enableGLO  = _model.config.gps.enableGLONASS;
  bool enableGAL  = _model.config.gps.enableGalileo;
  bool enableBDS  = _model.config.gps.enableBeiDou;
  bool enableQZSS = _model.config.gps.enableQZSS;
  bool enableSBAS = _model.config.gps.enableSBAS;

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
    // case 0: Auto - use individual enable flags
  }

  const Gps::UbxCfgGnss7 gnss{
    .msgVer = 0,
    .numTrkChHw = 0,
    .numTrkChUse = 0xFF,
    .numConfigBlocks = 7,
    .blocks = {
      // GPS: L1C/A or L1+L5
      { 0x00, 0x08, 0x10, 0x00, (uint8_t)(enableGPS  ? 0x01 : 0x00), 0x00, (uint8_t)(useDualBand ? 0x03 : 0x01), 0x01 },
      // SBAS: L1C/A
      { 0x01, 0x01, 0x03, 0x00, (uint8_t)(enableSBAS ? 0x01 : 0x00), 0x00, 0x01, 0x01 },
      // Galileo: E1 or E1+E5a
      { 0x02, 0x04, 0x08, 0x00, (uint8_t)(enableGAL  ? 0x01 : 0x00), 0x00, 0x01, 0x01 },
      // BeiDou: B1I or B1I+B2a
      { 0x03, 0x08, 0x10, 0x00, (uint8_t)(enableBDS  ? 0x01 : 0x00), 0x00, (uint8_t)(useDualBand ? 0x03 : 0x01), 0x01 },
      // IMES: disabled
      { 0x04, 0x00, 0x00, 0x00, 0x05, 0x00, 0x01, 0x01 },
      // QZSS: L1C/A or L1+L5
      { 0x05, 0x00, 0x03, 0x00, (uint8_t)(enableQZSS ? 0x01 : 0x00), 0x00, 0x01, 0x01 },
      // GLONASS: L1
      { 0x06, 0x08, 0x0E, 0x00, (uint8_t)(enableGLO  ? 0x01 : 0x00), 0x00, 0x01, 0x01 },
    },
  };

  _model.logger.info().log(F("GPS GNSS "));
  if (useDualBand) _model.logger.info().log(F("L1+L5 "));
  _model.logger.info().log(F("["));
  if (enableGPS)  _model.logger.info().log(F("GPS "));
  if (enableGLO)  _model.logger.info().log(F("GLO "));
  if (enableGAL)  _model.logger.info().log(F("GAL "));
  if (enableBDS)  _model.logger.info().log(F("BDS "));
  if (enableQZSS) _model.logger.info().log(F("QZSS "));
  if (enableSBAS) _model.logger.info().log(F("SBAS"));
  _model.logger.info().logln(F("]"));

  send(gnss, SET_RATE, SET_RATE); // on NAK (CFG-GNSS deprecated >PROTVER 23.01), skip to SET_RATE instead of ERROR
}

void GpsSensor::calculateHomeVector() const
{
  if (!_model.state.gps.homeSet || !_model.state.gps.fix) {
    _model.state.gps.distanceToHome = 0;
    _model.state.gps.directionToHome = 0;
    return;
  }

  const int32_t lat1 = _model.state.gps.location.home.lat;
  const int32_t lon1 = _model.state.gps.location.home.lon;
  const int32_t lat2 = _model.state.gps.location.raw.lat;
  const int32_t lon2 = _model.state.gps.location.raw.lon;

  // Equirectangular approximation (valid for short distances < few km)
  const float LAT_TO_M = 1.113e-2f; // deg*1e-7 to meters (111300 m/deg / 1e7)
  const float dlat = (lat2 - lat1) * LAT_TO_M;
  const float dlon = (lon2 - lon1) * LAT_TO_M * cosf(lat1 * 1e-7f * (float)M_PI / 180.0f);

  _model.state.gps.distanceToHome = (uint16_t)sqrtf(dlat * dlat + dlon * dlon);

  float bearing = atan2f(dlon, dlat) * (180.0f / (float)M_PI);
  if (bearing < 0.0f) bearing += 360.0f;
  _model.state.gps.directionToHome = (int16_t)bearing;
}

void GpsSensor::handleNavPvt() const
{
  const auto &m = *_ubxMsg.getAs<Gps::UbxNavPvt92>();

  _model.state.gps.fix = m.fixType == 3 && m.flags.gnssFixOk;
  _model.state.gps.fixType = m.fixType;
  _model.state.gps.numSats = m.numSV;

  _model.state.gps.accuracy.pDop = m.pDOP;
  _model.state.gps.accuracy.horizontal = m.hAcc; // mm
  _model.state.gps.accuracy.vertical = m.vAcc;   // mm
  _model.state.gps.accuracy.speed = m.sAcc;      // mm/s
  _model.state.gps.accuracy.heading = m.headAcc; // deg * 1e5

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
