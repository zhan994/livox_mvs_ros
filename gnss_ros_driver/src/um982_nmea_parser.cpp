#include "um982_nmea_parser.h"

#include <cmath>
#include <cstdlib>
#include <sstream>

namespace um982_nmea {
namespace {
constexpr double kPi = 3.14159265358979323846;
} // namespace

Um982NmeaParser::Um982NmeaParser(ChecksumMode checksum_mode)
    : checksum_mode_(checksum_mode) {}

ParseResult Um982NmeaParser::ParseLine(const std::string &input_line) const {
  ParseResult result;

  const std::string line = Trim(input_line);
  result.raw_sentence = line;

  if (line.empty()) {
    result.type = SentenceType::kNone;
    return result;
  }

  if (line.front() != '$') {
    result.type = SentenceType::kMalformed;
    result.error_message = "NMEA sentence does not start with '$'.";
    return result;
  }

  if (HasChecksum(line)) {
    if (!CheckNmeaChecksum(line)) {
      result.type = SentenceType::kInvalidChecksum;
      result.error_message = "Invalid NMEA checksum.";
      return result;
    }
  } else if (checksum_mode_ == ChecksumMode::kRequire) {
    result.type = SentenceType::kInvalidChecksum;
    result.error_message = "Missing NMEA checksum.";
    return result;
  }

  const std::vector<std::string> fields = SplitNmea(line);
  if (fields.empty()) {
    result.type = SentenceType::kMalformed;
    result.error_message = "Empty NMEA fields.";
    return result;
  }

  const std::string &sentence_id = fields[0];

  if (EndsWith(sentence_id, "GGA")) {
    return ParseGga(line, fields);
  }

  if (EndsWith(sentence_id, "GST")) {
    return ParseGst(line, fields);
  }

  result.type = SentenceType::kUnsupported;
  return result;
}

bool Um982NmeaParser::MakeDiagonalEnuCovarianceFromGst(
    const GST_Data &gst, CovarianceData *covariance_data) {
  if (covariance_data == nullptr) {
    return false;
  }

  covariance_data->covariance.fill(0.0);
  covariance_data->type = CovarianceType::kUnknown;

  if (!gst.has_lat_lon_alt_std) {
    return false;
  }

  // GST provides latitude, longitude and altitude standard deviations.
  // NavSatFix uses ENU covariance order:
  //
  //   [ E-E, E-N, E-U,
  //     N-E, N-N, N-U,
  //     U-E, U-N, U-U ]
  //
  // Therefore:
  //   East  <- longitude std
  //   North <- latitude std
  //   Up    <- altitude std
  covariance_data->covariance[0] = gst.lon_std_m * gst.lon_std_m;
  covariance_data->covariance[4] = gst.lat_std_m * gst.lat_std_m;
  covariance_data->covariance[8] = gst.alt_std_m * gst.alt_std_m;

  covariance_data->type = CovarianceType::kDiagonalKnown;
  return true;
}

bool Um982NmeaParser::MakeEllipseEnuCovarianceFromGst(
    const GST_Data &gst, CovarianceData *covariance_data) {
  if (covariance_data == nullptr) {
    return false;
  }

  covariance_data->covariance.fill(0.0);
  covariance_data->type = CovarianceType::kUnknown;

  if (!gst.has_error_ellipse || !gst.has_lat_lon_alt_std) {
    return false;
  }

  // GST orientation is usually the semi-major axis angle from true north.
  // Convert the horizontal error ellipse into EN covariance.
  const double theta = gst.orientation_deg * kPi / 180.0;

  const double major_var = gst.semi_major_std_m * gst.semi_major_std_m;
  const double minor_var = gst.semi_minor_std_m * gst.semi_minor_std_m;
  const double up_var = gst.alt_std_m * gst.alt_std_m;

  const double sin_theta = std::sin(theta);
  const double cos_theta = std::cos(theta);

  const double var_e =
      major_var * sin_theta * sin_theta + minor_var * cos_theta * cos_theta;

  const double var_n =
      major_var * cos_theta * cos_theta + minor_var * sin_theta * sin_theta;

  const double cov_en = (major_var - minor_var) * sin_theta * cos_theta;

  covariance_data->covariance[0] = var_e;
  covariance_data->covariance[1] = cov_en;
  covariance_data->covariance[2] = 0.0;

  covariance_data->covariance[3] = cov_en;
  covariance_data->covariance[4] = var_n;
  covariance_data->covariance[5] = 0.0;

  covariance_data->covariance[6] = 0.0;
  covariance_data->covariance[7] = 0.0;
  covariance_data->covariance[8] = up_var;

  covariance_data->type = CovarianceType::kKnown;
  return true;
}

std::string Um982NmeaParser::Trim(const std::string &input) {
  std::size_t begin = 0;
  std::size_t end = input.size();

  while (begin < end && (input[begin] == ' ' || input[begin] == '\r' ||
                         input[begin] == '\n' || input[begin] == '\t')) {
    ++begin;
  }

  while (end > begin && (input[end - 1] == ' ' || input[end - 1] == '\r' ||
                         input[end - 1] == '\n' || input[end - 1] == '\t')) {
    --end;
  }

  return input.substr(begin, end - begin);
}

bool Um982NmeaParser::EndsWith(const std::string &str,
                               const std::string &suffix) {
  if (str.size() < suffix.size()) {
    return false;
  }

  return str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool Um982NmeaParser::ParseDouble(const std::string &input, double *value) {
  if (value == nullptr || input.empty()) {
    return false;
  }

  char *end = nullptr;
  const double parsed_value = std::strtod(input.c_str(), &end);

  if (end == input.c_str() || *end != '\0') {
    return false;
  }

  *value = parsed_value;
  return true;
}

bool Um982NmeaParser::ParseInt(const std::string &input, int *value) {
  if (value == nullptr) {
    return false;
  }

  double parsed_value = 0.0;
  if (!ParseDouble(input, &parsed_value)) {
    return false;
  }

  *value = static_cast<int>(parsed_value);
  return true;
}

bool Um982NmeaParser::HasChecksum(const std::string &line) {
  return line.find('*') != std::string::npos;
}

bool Um982NmeaParser::CheckNmeaChecksum(const std::string &line) {
  if (line.size() < 4 || line.front() != '$') {
    return false;
  }

  const std::size_t star_pos = line.find('*');
  if (star_pos == std::string::npos || star_pos + 2 >= line.size()) {
    return false;
  }

  unsigned char calculated = 0;
  for (std::size_t i = 1; i < star_pos; ++i) {
    calculated ^= static_cast<unsigned char>(line[i]);
  }

  const std::string checksum_str = line.substr(star_pos + 1, 2);

  char *end = nullptr;
  const unsigned long expected = std::strtoul(checksum_str.c_str(), &end, 16);
  if (end == checksum_str.c_str()) {
    return false;
  }

  return calculated == static_cast<unsigned char>(expected);
}

std::vector<std::string> Um982NmeaParser::SplitNmea(const std::string &line) {
  std::vector<std::string> fields;

  if (line.empty() || line.front() != '$') {
    return fields;
  }

  const std::size_t star_pos = line.find('*');

  std::string body;
  if (star_pos == std::string::npos) {
    body = line.substr(1);
  } else {
    body = line.substr(1, star_pos - 1);
  }

  std::stringstream ss(body);
  std::string field;

  while (std::getline(ss, field, ',')) {
    fields.push_back(field);
  }

  return fields;
}

bool Um982NmeaParser::NmeaDegToDecimal(const std::string &value,
                                       const std::string &direction,
                                       double *decimal_deg) {
  if (decimal_deg == nullptr || direction.empty()) {
    return false;
  }

  double nmea_deg = 0.0;
  if (!ParseDouble(value, &nmea_deg)) {
    return false;
  }

  const int degree = static_cast<int>(nmea_deg / 100.0);
  const double minute = nmea_deg - static_cast<double>(degree) * 100.0;

  double decimal = static_cast<double>(degree) + minute / 60.0;

  if (direction == "S" || direction == "W") {
    decimal = -decimal;
  }

  *decimal_deg = decimal;
  return true;
}

ParseResult Um982NmeaParser::ParseGga(const std::string &raw_sentence,
                                      const std::vector<std::string> &fields) {
  ParseResult result;
  result.type = SentenceType::kGGA;
  result.raw_sentence = raw_sentence;

  // GGA:
  // fields[0]  GNGGA / GPGGA / ...
  // fields[1]  UTC
  // fields[2]  latitude
  // fields[3]  N/S
  // fields[4]  longitude
  // fields[5]  E/W
  // fields[6]  fix quality
  // 0 = 失效
  // 1 = 单点定位
  // 2 = 差分定位
  // 3 = GPS PPS 模式
  // 4 = RTK Int
  // 5 = RTK Float
  // 6 = 惯导模式
  // 7 = 手动输入模式
  // 8 = 模拟器模式

  // fields[7]  number of satellites
  // fields[8]  HDOP
  // fields[9]  altitude above MSL
  // fields[10] M
  // fields[11] geoid separation
  // fields[12] M
  // fields[13] differential age
  // fields[14] station ID
  if (fields.size() < 12) {
    result.type = SentenceType::kMalformed;
    result.error_message = "GGA sentence has insufficient fields.";
    return result;
  }

  GGA_Data gga;
  if (fields[0].size() >= 2) {
    gga.talker = fields[0].substr(0, 2);
  }
  gga.utc = fields[1];

  if (!NmeaDegToDecimal(fields[2], fields[3], &gga.latitude_deg)) {
    result.type = SentenceType::kMalformed;
    result.error_message = "Failed to parse GGA latitude.";
    return result;
  }

  if (!NmeaDegToDecimal(fields[4], fields[5], &gga.longitude_deg)) {
    result.type = SentenceType::kMalformed;
    result.error_message = "Failed to parse GGA longitude.";
    return result;
  }

  ParseInt(fields[6], &gga.fix_quality);
  ParseInt(fields[7], &gga.num_satellites);

  gga.has_hdop = ParseDouble(fields[8], &gga.hdop);
  gga.has_altitude_msl = ParseDouble(fields[9], &gga.altitude_msl_m);

  gga.has_geoid_separation =
      fields.size() > 11 && ParseDouble(fields[11], &gga.geoid_separation_m);

  if (gga.has_altitude_msl && gga.has_geoid_separation) {
    gga.altitude_ellipsoid_m = gga.altitude_msl_m + gga.geoid_separation_m;
  } else if (gga.has_altitude_msl) {
    gga.altitude_ellipsoid_m = gga.altitude_msl_m;
  }

  if (fields.size() > 13) {
    gga.has_differential_age = ParseDouble(fields[13], &gga.differential_age_s);
  }

  if (fields.size() > 14) {
    gga.station_id = fields[14];
  }

  result.gga = gga;
  return result;
}

ParseResult Um982NmeaParser::ParseGst(const std::string &raw_sentence,
                                      const std::vector<std::string> &fields) {
  ParseResult result;
  result.type = SentenceType::kGST;
  result.raw_sentence = raw_sentence;

  // GST:
  // fields[0] GNGST / GPGST / ...
  // fields[1] UTC
  // fields[2] RMS
  // fields[3] semi-major std
  // fields[4] semi-minor std
  // fields[5] orientation
  // fields[6] latitude std
  // fields[7] longitude std
  // fields[8] altitude std
  if (fields.size() < 9) {
    result.type = SentenceType::kMalformed;
    result.error_message = "GST sentence has insufficient fields.";
    return result;
  }

  GST_Data gst;
  if (fields[0].size() >= 2) {
    gst.talker = fields[0].substr(0, 2);
  }
  gst.utc = fields[1];

  gst.has_rms = ParseDouble(fields[2], &gst.rms_m);

  const bool has_semi_major = ParseDouble(fields[3], &gst.semi_major_std_m);
  const bool has_semi_minor = ParseDouble(fields[4], &gst.semi_minor_std_m);
  const bool has_orientation = ParseDouble(fields[5], &gst.orientation_deg);

  gst.has_error_ellipse = has_semi_major && has_semi_minor && has_orientation;

  const bool has_lat = ParseDouble(fields[6], &gst.lat_std_m);
  const bool has_lon = ParseDouble(fields[7], &gst.lon_std_m);
  const bool has_alt = ParseDouble(fields[8], &gst.alt_std_m);

  gst.has_lat_lon_alt_std = has_lat && has_lon && has_alt;

  result.gst = gst;
  return result;
}

} // namespace um982_nmea