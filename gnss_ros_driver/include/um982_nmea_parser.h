/**
 * @file um982_nmea_parser.h
 * @author Zhihao Zhan (zhihazhan2-c@my.cityu.edu.hk)
 * @brief Header file for the UM982 NMEA parser class.
 * @version 0.1
 * @date 2026-06-23
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef UM982_NMEA_PARSER_H
#define UM982_NMEA_PARSER_H

#include <array>
#include <string>
#include <vector>

namespace um982_nmea {

struct GGA_Data {
  std::string talker;
  std::string utc;

  double latitude_deg = 0.0;
  double longitude_deg = 0.0;

  int fix_quality = 0;
  int num_satellites = 0;

  double hdop = 0.0;

  double altitude_msl_m = 0.0;
  double geoid_separation_m = 0.0;
  double altitude_ellipsoid_m = 0.0;

  double differential_age_s = 0.0;
  std::string station_id;

  bool has_hdop = false;
  bool has_altitude_msl = false;
  bool has_geoid_separation = false;
  bool has_differential_age = false;
};

struct GST_Data {
  std::string talker;
  std::string utc;

  double rms_m = 0.0;
  double semi_major_std_m = 0.0;
  double semi_minor_std_m = 0.0;
  double orientation_deg = 0.0;

  double lat_std_m = 0.0;
  double lon_std_m = 0.0;
  double alt_std_m = 0.0;

  bool has_rms = false;
  bool has_error_ellipse = false;
  bool has_lat_lon_alt_std = false;
};

enum class SentenceType {
  kNone = 0,        // 空行， \r\n
  kGGA,             // 有效GGA
  kGST,             // 有效GST
  kUnsupported,     // 不支持的有效NMEA语句
  kInvalidChecksum, // NMEA语句，但校验和不匹配
  kMalformed,       // 格式不正确
};

struct ParseResult {
  SentenceType type = SentenceType::kNone;

  GGA_Data gga;
  GST_Data gst;

  std::string raw_sentence;
  std::string error_message;
};

enum class CovarianceType {
  kUnknown = 0,      // 完全没有可靠精度信息
  kApproximated = 1, // 没有 GST 人为设置，但用 HDOP 或 fix quality 估计
  kDiagonalKnown = 2, // 收到 GST 且使用 lat/lon/alt std
  kKnown = 3,         // 收到 GST 且使用 error ellipse
};

struct CovarianceData {
  std::array<double, 9> covariance = {};
  CovarianceType type = CovarianceType::kUnknown;
};

class Um982NmeaParser {
public:
  enum class ChecksumMode {
    kRequire = 0,
    kAllowMissing,
  };

  explicit Um982NmeaParser(ChecksumMode checksum_mode = ChecksumMode::kRequire);

  ParseResult ParseLine(const std::string &line) const;

  static bool MakeDiagonalEnuCovarianceFromGst(const GST_Data &gst,
                                               CovarianceData *covariance_data);

  static bool MakeEllipseEnuCovarianceFromGst(const GST_Data &gst,
                                              CovarianceData *covariance_data);

private:
  static std::string Trim(const std::string &input);

  static bool EndsWith(const std::string &str, const std::string &suffix);

  static bool ParseDouble(const std::string &input, double *value);

  static bool ParseInt(const std::string &input, int *value);

  static bool HasChecksum(const std::string &line);

  static bool CheckNmeaChecksum(const std::string &line);

  static std::vector<std::string> SplitNmea(const std::string &line);

  static bool NmeaDegToDecimal(const std::string &value,
                               const std::string &direction,
                               double *decimal_deg);

  static ParseResult ParseGga(const std::string &raw_sentence,
                              const std::vector<std::string> &fields);

  static ParseResult ParseGst(const std::string &raw_sentence,
                              const std::vector<std::string> &fields);

  ChecksumMode checksum_mode_;
};

} // namespace um982_nmea

#endif // UM982_NMEA_PARSER_H