#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <limits>
#include <string>

#include <boost/asio.hpp>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#include "um982_nmea_parser.h"

namespace {

constexpr double kSecondsPerDay = 86400.0;

bool ParseUtcToSecondsOfDay(const std::string &utc, double *seconds_of_day) {
  if (seconds_of_day == nullptr || utc.size() < 6) {
    return false;
  }

  char *end = nullptr;
  const double raw_utc = std::strtod(utc.c_str(), &end);
  if (end == utc.c_str() || *end != '\0') {
    return false;
  }

  const int hours = static_cast<int>(raw_utc / 10000.0);
  const int minutes = static_cast<int>(
      (raw_utc - static_cast<double>(hours) * 10000.0) / 100.0);
  const double seconds = raw_utc - static_cast<double>(hours) * 10000.0 -
                         static_cast<double>(minutes) * 100.0;

  if (hours < 0 || hours > 23 || minutes < 0 || minutes > 59 || seconds < 0.0 ||
      seconds >= 61.0) {
    return false;
  }

  *seconds_of_day = static_cast<double>(hours) * 3600.0 +
                    static_cast<double>(minutes) * 60.0 + seconds;

  return true;
}

double UtcTimeDiffSec(double lhs_sec, double rhs_sec) {
  double diff = std::fabs(lhs_sec - rhs_sec);

  // Handle UTC day rollover, for example 23:59:59.95 and 00:00:00.05.
  if (diff > kSecondsPerDay / 2.0) {
    diff = kSecondsPerDay - diff;
  }

  return diff;
}

} // namespace

class Um982NmeaNavSatFixNode {
public:
  Um982NmeaNavSatFixNode()
      : nh_(), private_nh_("~"), serial_(io_service_),
        parser_(um982_nmea::Um982NmeaParser::ChecksumMode::kRequire) {
    private_nh_.param<std::string>("port", port_, "/dev/ttyACM0");
    private_nh_.param<int>("baudrate", baudrate_, 115200);
    private_nh_.param<std::string>("frame_id", frame_id_, "gnss_ant");
    private_nh_.param<std::string>("topic", topic_, "/fix");
    private_nh_.param<std::string>("covariance_mode", covariance_mode_,
                                   "diagonal");

    // For 10 Hz NMEA, one epoch interval is about 0.1 s.
    // 0.02 s is strict enough while still tolerating formatting jitter.
    private_nh_.param<double>("max_match_time_diff_sec",
                              max_match_time_diff_sec_, 0.02);

    // Cache timeout prevents stale unmatched GGA/GST from accumulating.
    private_nh_.param<double>("cache_timeout_sec", cache_timeout_sec_, 1.0);

    // If false, unmatched GGA will be dropped after cache timeout.
    // If true, unmatched GGA will be published with UNKNOWN covariance.
    private_nh_.param<bool>("publish_unmatched_gga", publish_unmatched_gga_,
                            false);

    private_nh_.param<bool>("publish_invalid", publish_invalid_, false);

    fix_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(topic_, 10);

    OpenSerial();

    ROS_INFO_STREAM("UM982 NMEA NavSatFix node started.");
    ROS_INFO_STREAM("port: " << port_);
    ROS_INFO_STREAM("baudrate: " << baudrate_);
    ROS_INFO_STREAM("topic: " << topic_);
    ROS_INFO_STREAM("frame_id: " << frame_id_);
    ROS_INFO_STREAM("covariance_mode: " << covariance_mode_);
    ROS_INFO_STREAM("max_match_time_diff_sec: " << max_match_time_diff_sec_);
    ROS_INFO_STREAM("cache_timeout_sec: " << cache_timeout_sec_);
  }

  void Spin() {
    boost::asio::streambuf buffer;

    while (ros::ok()) {
      try {
        boost::asio::read_until(serial_, buffer, '\n');

        std::istream input_stream(&buffer);
        std::string line;
        std::getline(input_stream, line);

        HandleLine(line);

        ros::spinOnce();
      } catch (const boost::system::system_error &error) {
        ROS_ERROR_STREAM_THROTTLE(2.0, "Serial read error: " << error.what());
        Reconnect();
      }
    }
  }

private:
  struct PendingGGA {
    um982_nmea::GGA_Data gga;
    ros::Time receive_stamp;
  };

  struct PendingGST {
    um982_nmea::GST_Data gst;
    um982_nmea::CovarianceData covariance_data;
    ros::Time receive_stamp;
  };

  void OpenSerial() {
    if (serial_.is_open()) {
      serial_.close();
    }

    serial_.open(port_);

    serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
    serial_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
    serial_.set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
    serial_.set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));
  }

  void Reconnect() {
    try {
      if (serial_.is_open()) {
        serial_.close();
      }

      ros::Duration(1.0).sleep();
      OpenSerial();

      ROS_INFO_STREAM("Reconnected to " << port_);
    } catch (const std::exception &error) {
      ROS_ERROR_STREAM_THROTTLE(2.0, "Reconnect failed: " << error.what());
    }
  }

  void HandleLine(const std::string &line) {
    const um982_nmea::ParseResult result = parser_.ParseLine(line);

    switch (result.type) {
    case um982_nmea::SentenceType::kGGA:
      HandleGGA(result.gga);
      break;

    case um982_nmea::SentenceType::kGST:
      HandleGST(result.gst);
      break;

    case um982_nmea::SentenceType::kInvalidChecksum:
      ROS_WARN_STREAM_THROTTLE(5.0,
                               "Invalid checksum: " << result.raw_sentence);
      break;

    case um982_nmea::SentenceType::kMalformed:
      ROS_WARN_STREAM_THROTTLE(5.0, "Malformed NMEA: " << result.error_message);
      break;

    case um982_nmea::SentenceType::kUnsupported:
    case um982_nmea::SentenceType::kNone:
    default:
      break;
    }

    PruneExpiredCache();
  }

  void HandleGGA(const um982_nmea::GGA_Data &gga) {
    if (gga.fix_quality == 0 && !publish_invalid_) {
      return;
    }

    double gga_utc_sec = 0.0;
    if (!ParseUtcToSecondsOfDay(gga.utc, &gga_utc_sec)) {
      ROS_WARN_STREAM_THROTTLE(5.0, "Invalid GGA UTC: " << gga.utc);
      return;
    }

    PendingGGA pending_gga;
    pending_gga.gga = gga;
    pending_gga.receive_stamp = ros::Time::now();
    pending_ggas_.push_back(pending_gga);

    TryPublishMatchedFixes();
  }

  void HandleGST(const um982_nmea::GST_Data &gst) {
    double gst_utc_sec = 0.0;
    if (!ParseUtcToSecondsOfDay(gst.utc, &gst_utc_sec)) {
      ROS_WARN_STREAM_THROTTLE(5.0, "Invalid GST UTC: " << gst.utc);
      return;
    }

    um982_nmea::CovarianceData covariance_data;
    bool valid_covariance = false;

    if (covariance_mode_ == "ellipse") {
      valid_covariance =
          um982_nmea::Um982NmeaParser::MakeEllipseEnuCovarianceFromGst(
              gst, &covariance_data);
    } else {
      valid_covariance =
          um982_nmea::Um982NmeaParser::MakeDiagonalEnuCovarianceFromGst(
              gst, &covariance_data);
    }

    if (!valid_covariance) {
      return;
    }

    PendingGST pending_gst;
    pending_gst.gst = gst;
    pending_gst.covariance_data = covariance_data;
    pending_gst.receive_stamp = ros::Time::now();
    pending_gsts_.push_back(pending_gst);

    TryPublishMatchedFixes();
  }

  void TryPublishMatchedFixes() {
    while (true) {
      int best_gga_index = -1;
      int best_gst_index = -1;
      double best_time_diff_sec = max_match_time_diff_sec_;

      for (int gga_index = 0;
           gga_index < static_cast<int>(pending_ggas_.size()); ++gga_index) {
        double gga_utc_sec = 0.0;
        if (!ParseUtcToSecondsOfDay(pending_ggas_[gga_index].gga.utc,
                                    &gga_utc_sec)) {
          continue;
        }

        for (int gst_index = 0;
             gst_index < static_cast<int>(pending_gsts_.size()); ++gst_index) {
          double gst_utc_sec = 0.0;
          if (!ParseUtcToSecondsOfDay(pending_gsts_[gst_index].gst.utc,
                                      &gst_utc_sec)) {
            continue;
          }

          const double time_diff_sec = UtcTimeDiffSec(gga_utc_sec, gst_utc_sec);

          if (time_diff_sec <= max_match_time_diff_sec_ &&
              time_diff_sec <= best_time_diff_sec) {
            best_time_diff_sec = time_diff_sec;
            best_gga_index = gga_index;
            best_gst_index = gst_index;
          }
        }
      }

      if (best_gga_index < 0 || best_gst_index < 0) {
        return;
      }

      PublishFix(pending_ggas_[best_gga_index].gga,
                 &pending_gsts_[best_gst_index].covariance_data);

      pending_ggas_.erase(pending_ggas_.begin() + best_gga_index);
      pending_gsts_.erase(pending_gsts_.begin() + best_gst_index);
    }
  }

  void PruneExpiredCache() {
    const ros::Time now = ros::Time::now();

    while (!pending_ggas_.empty()) {
      const double age_sec =
          (now - pending_ggas_.front().receive_stamp).toSec();

      if (age_sec <= cache_timeout_sec_) {
        break;
      }

      if (publish_unmatched_gga_) {
        PublishFix(pending_ggas_.front().gga, nullptr);
      } else {
        ROS_WARN_STREAM_THROTTLE(
            5.0, "Drop unmatched GGA. UTC: " << pending_ggas_.front().gga.utc);
      }

      pending_ggas_.pop_front();
    }

    while (!pending_gsts_.empty()) {
      const double age_sec =
          (now - pending_gsts_.front().receive_stamp).toSec();

      if (age_sec <= cache_timeout_sec_) {
        break;
      }

      ROS_WARN_STREAM_THROTTLE(
          5.0, "Drop unmatched GST. UTC: " << pending_gsts_.front().gst.utc);

      pending_gsts_.pop_front();
    }
  }

  void PublishFix(const um982_nmea::GGA_Data &gga,
                  const um982_nmea::CovarianceData *covariance_data) {
    if (gga.fix_quality == 0 && !publish_invalid_) {
      return;
    }

    sensor_msgs::NavSatFix fix_msg;

    fix_msg.header.stamp = ros::Time::now();
    fix_msg.header.frame_id = frame_id_;

    fix_msg.status = ConvertFixQualityToStatus(gga.fix_quality);

    fix_msg.latitude = gga.latitude_deg;
    fix_msg.longitude = gga.longitude_deg;

    if (gga.has_altitude_msl) {
      fix_msg.altitude = gga.altitude_ellipsoid_m;
    } else {
      fix_msg.altitude = std::numeric_limits<double>::quiet_NaN();
    }

    FillCovariance(covariance_data, &fix_msg);

    fix_pub_.publish(fix_msg);
  }

  sensor_msgs::NavSatStatus ConvertFixQualityToStatus(int fix_quality) const {
    sensor_msgs::NavSatStatus status;

    status.service = sensor_msgs::NavSatStatus::SERVICE_GPS |
                     sensor_msgs::NavSatStatus::SERVICE_GLONASS |
                     sensor_msgs::NavSatStatus::SERVICE_COMPASS |
                     sensor_msgs::NavSatStatus::SERVICE_GALILEO;

    if (fix_quality == 0) {
      status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    } else if (fix_quality == 9) {
      status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
    } else if (fix_quality == 2 || fix_quality == 4 || fix_quality == 5) {
      status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
    } else {
      status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    }

    return status;
  }

  void FillCovariance(const um982_nmea::CovarianceData *covariance_data,
                      sensor_msgs::NavSatFix *fix_msg) const {
    if (fix_msg == nullptr) {
      return;
    }

    for (double &value : fix_msg->position_covariance) {
      value = 0.0;
    }

    fix_msg->position_covariance_type =
        sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

    if (covariance_data == nullptr) {
      return;
    }

    for (int i = 0; i < 9; ++i) {
      fix_msg->position_covariance[i] = covariance_data->covariance[i];
    }

    fix_msg->position_covariance_type =
        ConvertCovarianceType(covariance_data->type);
  }

  uint8_t ConvertCovarianceType(um982_nmea::CovarianceType type) const {
    switch (type) {
    case um982_nmea::CovarianceType::kApproximated:
      return sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

    case um982_nmea::CovarianceType::kDiagonalKnown:
      return sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    case um982_nmea::CovarianceType::kKnown:
      return sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;

    case um982_nmea::CovarianceType::kUnknown:
    default:
      return sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher fix_pub_;

  std::string port_;
  int baudrate_ = 115200;
  std::string frame_id_;
  std::string topic_;
  std::string covariance_mode_;
  double max_match_time_diff_sec_ = 0.02;
  double cache_timeout_sec_ = 1.0;
  bool publish_unmatched_gga_ = false;
  bool publish_invalid_ = false;

  boost::asio::io_service io_service_;
  boost::asio::serial_port serial_;

  um982_nmea::Um982NmeaParser parser_;

  std::deque<PendingGGA> pending_ggas_;
  std::deque<PendingGST> pending_gsts_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "um982_nmea_navsatfix_node");

  try {
    Um982NmeaNavSatFixNode node;
    node.Spin();
  } catch (const std::exception &error) {
    ROS_FATAL_STREAM("Failed to start UM982 node: " << error.what());
    return 1;
  }

  return 0;
}