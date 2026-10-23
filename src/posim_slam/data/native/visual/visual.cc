// Camera-only estimator. Outputs use arbitrary monocular scale and must not be
// forwarded to MAVROS vision inputs without a separately validated metric frame.
#include <stella_vslam/system.h>
#include <stella_vslam/config.h>
#include <stella_vslam/data/landmark.h>
#include <stella_vslam/publish/frame_publisher.h>
#include <stella_vslam/publish/map_publisher.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <filesystem>
#include <fstream>
#include <array>
#include <cstring>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <limits>
#include <iostream>

class VisualSlam : public rclcpp::Node {
 public:
  VisualSlam() : Node("wwos_visual_slam") {
    cv::setNumThreads(1);
    rcl_interfaces::msg::ParameterDescriptor startup_only; startup_only.read_only = true;
    rate_ = declare_parameter<double>("max_frame_hz", 5.0, startup_only);
    fixed_seed_ = declare_parameter<bool>("initialization_fixed_seed", false, startup_only);
    max_points_ = declare_parameter<int>("max_points", 1000, startup_only);
    if (!std::isfinite(rate_) || rate_ <= 0 || rate_ > 30 ||
        (max_points_ != 0 && max_points_ != 1000 && max_points_ != 2000 && max_points_ != 5000))
      throw std::invalid_argument("Invalid visual SLAM resource limits");
    robot_ = declare_parameter<std::string>("robot_name", "bluerov2", startup_only);
    if (robot_ != "bluerov2" && robot_ != "wamv") throw std::invalid_argument("Unsupported SLAM robot");
    const bool common_topics = declare_parameter<bool>("common_vehicle_topics", false, startup_only);
    prefix_ = common_topics ? "/vehicle/slam/" : "/model/" + robot_ + "/slam/";
    frame_ = (common_topics ? std::string("vehicle") : robot_) + "/visual_slam_unscaled";
    export_directory_ = declare_parameter<std::string>("export_directory", "", startup_only);
    session_nonce_ = declare_parameter<std::string>("session_nonce", "", startup_only);
    if (session_nonce_.size() > 128 || session_nonce_.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") != std::string::npos)
      throw std::invalid_argument("Invalid session nonce");
    export_service_ = create_service<std_srvs::srv::Trigger>(prefix_ + "visual/export",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) { export_map(*response); });
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(prefix_ + "visual/pose", qos);
    points_ = create_publisher<sensor_msgs::msg::PointCloud2>(prefix_ + "visual/points", qos);
    status_ = create_publisher<std_msgs::msg::String>(prefix_ + "visual/status", rclcpp::QoS(1).transient_local());
    const auto camera_qos = robot_ == "wamv" && !common_topics ? rclcpp::QoS(1).reliable() : rclcpp::QoS(rclcpp::SensorDataQoS().keep_last(1));
    info_ = create_subscription<sensor_msgs::msg::CameraInfo>(prefix_ + "camera_info", camera_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { configure(*msg); });
    image_ = create_subscription<sensor_msgs::msg::Image>(prefix_ + "camera", camera_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { process(*msg); });
    watchdog_ = create_wall_timer(std::chrono::seconds(1), [this] {
      if (slam_ && std::chrono::steady_clock::now() - last_received_ > std::chrono::seconds(2))
        report("waiting_image", map_count_, published_count_);
    });
    report("waiting_camera", 0, 0);
  }
  ~VisualSlam() override { if (slam_) slam_->shutdown(); }
 private:
  void export_map(std_srvs::srv::Trigger::Response &response) {
    if (!slam_ || export_directory_.empty()) { response.message = "slam_export_unavailable"; return; }
    bool paused = false;
    const auto destination = std::filesystem::path(export_directory_) / "visual-map.ply";
    const auto temporary = std::filesystem::path(export_directory_) / "visual-map.ply.tmp";
    try {
      slam_->disable_mapping_module(); paused = true;
      std::vector<std::shared_ptr<stella_vslam::data::landmark>> landmarks;
      std::set<std::shared_ptr<stella_vslam::data::landmark>> local;
      slam_->get_map_publisher()->get_landmarks(landmarks, local);
      if (landmarks.size() > 2000000) throw std::runtime_error("slam_export_point_limit");
      std::vector<std::array<float, 3>> snapshot;
      snapshot.reserve(landmarks.size());
      for (const auto &landmark : landmarks) {
        if (!landmark || landmark->will_be_erased()) continue;
        const auto point = landmark->get_pos_in_world();
        if (!point.allFinite() || point.cwiseAbs().maxCoeff() > std::numeric_limits<float>::max()) continue;
        snapshot.push_back({static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z())});
      }
      slam_->enable_mapping_module(); paused = false;
      if (snapshot.empty()) throw std::runtime_error("slam_map_empty");
      std::filesystem::create_directories(export_directory_);
      std::ofstream file(temporary, std::ios::binary | std::ios::trunc);
      file.exceptions(std::ios::failbit | std::ios::badbit);
      file << "ply\nformat binary_little_endian 1.0\ncomment scope complete_visual_map_snapshot\n"
           << "comment library stella_vslam " WWOS_SLAM_VERSION "\ncomment revision " WWOS_SLAM_REVISION "\n"
           << "comment session_nonce " << session_nonce_ << "\ncomment frame_id " << frame_ << "\ncomment units arbitrary\n"
           << "element vertex " << snapshot.size() << "\nproperty float x\nproperty float y\nproperty float z\nend_header\n";
      for (const auto &point : snapshot) for (float value : point) {
        uint32_t bits; std::memcpy(&bits, &value, sizeof(bits));
        const char bytes[] = {static_cast<char>(bits),static_cast<char>(bits >> 8),static_cast<char>(bits >> 16),static_cast<char>(bits >> 24)};
        file.write(bytes,4);
      }
      file.close(); std::filesystem::rename(temporary,destination);
      response.success = true;
      response.message = "{\"points\":" + std::to_string(snapshot.size()) + ",\"session_nonce\":\"" + session_nonce_ + "\"}";
    } catch (const std::exception &error) {
      if (paused) slam_->enable_mapping_module();
      std::error_code ignored; std::filesystem::remove(temporary, ignored);
      response.message = error.what();
    }
  }
  void report(const std::string &state, size_t count, size_t displayed) {
    map_count_ = count; published_count_ = displayed;
    std_msgs::msg::String msg;
    msg.data = "{\"library\":\"stella_vslam\",\"version\":\"" WWOS_SLAM_VERSION "\",\"revision\":\"" WWOS_SLAM_REVISION "\",\"session_nonce\":\"" + session_nonce_ + "\",\"mode\":\"visual\",\"state\":\"" + state +
      "\",\"scale\":\"arbitrary\",\"frame\":\"" + frame_ + "\",\"landmarks\":" +
      std::to_string(count) + ",\"processed_frames\":" + std::to_string(processed_frames_) +
      ",\"processed_stamp_ns\":" + std::to_string(processed_stamp_ns_) +
      ",\"initialization_fixed_seed\":" + (fixed_seed_ ? "true" : "false") + ",\"published_points\":" + std::to_string(displayed) + "}";
    status_->publish(msg);
  }
  void configure(const sensor_msgs::msg::CameraInfo &msg) {
    if (slam_) return;
    if (msg.width == 0 || msg.height == 0 || msg.width > 4096 || msg.height > 4096 ||
        !std::isfinite(msg.k[0]) || !std::isfinite(msg.k[4]) || msg.k[0] <= 0 || msg.k[4] <= 0 ||
        !std::isfinite(msg.k[2]) || !std::isfinite(msg.k[5]) ||
        (!msg.distortion_model.empty() && msg.distortion_model != "plumb_bob")) {
      report("invalid_camera_info", 0, 0); return;
    }
    for (double value : msg.d) if (!std::isfinite(value)) { report("invalid_camera_info", 0, 0); return; }
    YAML::Node config, camera;
    camera["name"] = "Selected calibrated vehicle camera";
    camera["setup"] = "monocular"; camera["model"] = "perspective";
    camera["cols"] = msg.width; camera["rows"] = msg.height;
    camera["fx"] = msg.k[0]; camera["fy"] = msg.k[4];
    camera["cx"] = msg.k[2]; camera["cy"] = msg.k[5];
    const char *distortion[] = {"k1", "k2", "p1", "p2", "k3"};
    for (size_t i = 0; i < 5; ++i) camera[distortion[i]] = i < msg.d.size() ? msg.d[i] : 0.;
    camera["fps"] = rate_; camera["color_order"] = "BGR";
    config["Camera"] = camera;
    config["Feature"]["name"] = "ORB";
    config["Mapping"]["backend"] = "g2o";
    config["Tracking"]["backend"] = "g2o";
    config["System"]["map_format"] = "msgpack";
    config["Initializer"]["use_fixed_seed"] = fixed_seed_;
    try {
      slam_ = std::make_unique<stella_vslam::system>(std::make_shared<stella_vslam::config>(config), "");
      slam_->startup(); width_ = msg.width; height_ = msg.height;
      report("waiting_image", 0, 0);
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "SLAM initialization failed: %s", error.what());
      throw;
    }
  }
  void process(const sensor_msgs::msg::Image &msg) {
    if (!slam_) return;
    const double stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
    if (stamp < last_input_) { slam_->request_reset(); last_processed_ = -1e30; report("resetting", 0, 0); }
    last_input_ = stamp;
    if (stamp - last_processed_ < 1. / rate_ - 1e-6) return;
    const bool mono = msg.encoding == "mono8";
    if ((!mono && msg.encoding != "bgr8" && msg.encoding != "rgb8") ||
        msg.width != width_ || msg.height != height_ || msg.step < msg.width * (mono ? 1u : 3u) ||
        msg.data.size() < static_cast<size_t>(msg.step) * msg.height) {
      report("invalid_image", 0, 0); return;
    }
    cv::Mat input(msg.height, msg.width, mono ? CV_8UC1 : CV_8UC3,
                  const_cast<unsigned char *>(msg.data.data()), msg.step), bgr;
    if (mono) cv::cvtColor(input, bgr, cv::COLOR_GRAY2BGR);
    else if (msg.encoding == "rgb8") cv::cvtColor(input, bgr, cv::COLOR_RGB2BGR);
    else bgr = input;
    last_received_ = std::chrono::steady_clock::now();
    last_processed_ = stamp;
    processed_stamp_ns_ = static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL + msg.header.stamp.nanosec;
    ++processed_frames_;
    auto transform = slam_->feed_monocular_frame(bgr, stamp);
    if (transform && transform->allFinite()) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header; pose.header.frame_id = frame_;
      pose.pose.position.x = (*transform)(0,3); pose.pose.position.y = (*transform)(1,3); pose.pose.position.z = (*transform)(2,3);
      const Eigen::Quaterniond q(transform->block<3,3>(0,0));
      pose.pose.orientation.x = q.x(); pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z(); pose.pose.orientation.w = q.w();
      pose_->publish(pose);
    }
    std::vector<std::shared_ptr<stella_vslam::data::landmark>> landmarks;
    std::set<std::shared_ptr<stella_vslam::data::landmark>> local;
    slam_->get_map_publisher()->get_landmarks(landmarks, local);
    const size_t count = landmarks.size();
    const size_t limit = std::min(count, max_points_ ? static_cast<size_t>(max_points_) : count);
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = msg.header; cloud.header.frame_id = frame_;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz"); modifier.resize(limit);
    size_t emitted = 0;
    if (limit) {
    sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"), y(cloud,"y"), z(cloud,"z");
    for (size_t i = 0; i < limit; ++i) {
      const auto &landmark = landmarks[i * count / limit];
      if (!landmark || landmark->will_be_erased()) continue;
      const auto position = landmark->get_pos_in_world();
      if (!position.allFinite() || position.cwiseAbs().maxCoeff() > std::numeric_limits<float>::max()) continue;
      *x = position.x(); *y = position.y(); *z = position.z(); ++x; ++y; ++z; ++emitted;
    }
    }
    modifier.resize(emitted); cloud.is_dense = true; points_->publish(cloud);
    report(slam_->get_frame_publisher()->get_tracking_state(), count, emitted);
  }
  std::string export_directory_, session_nonce_, robot_, prefix_, frame_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr export_service_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  std::chrono::steady_clock::time_point last_received_ = std::chrono::steady_clock::now();
  double rate_, last_input_ = -1e30, last_processed_ = -1e30;
  size_t map_count_ = 0, published_count_ = 0;
  bool fixed_seed_ = false;
  uint64_t processed_frames_ = 0; int64_t processed_stamp_ns_ = -1;
  int max_points_; uint32_t width_ = 0, height_ = 0;
  std::unique_ptr<stella_vslam::system> slam_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { auto node = std::make_shared<VisualSlam>(); rclcpp::spin(node); node.reset(); }
  catch (const std::exception &error) { std::cerr << error.what() << std::endl; rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
